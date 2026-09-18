/**
 * Minimal raw LSM → WiFi UDP streamer (ESP32-C6).
 * Intentionally tiny: no LoRa/MQTT/ESP-NOW/status tasks — those caused
 * boot loops on USB-powered boards when WiFi started.
 *
 * SSID LoRa-1 / 10941090 → UDP 109.248.247.168:9500 (this lab server).
 * Note: 192.168.1.1 on the PC is Ethernet 2 (base LAN), not reachable from WiFi.
 */
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "lsm6ds3trc.h"

static const char *TAG = "RAW";

#define RAW_SSID         "LoRa-1"
#define RAW_PASS         "10941090"
#define RAW_HOST         "109.248.247.168"
#define RAW_PORT         9500
#define RAW_MAGIC        0x31574152u
#define RAW_CHUNK        64
#define RAW_ODR          ACCEL_ODR_3330HZ
#define RAW_RANGE        ACCEL_RANGE_4G
#define GPIO_LORA_EN     GPIO_NUM_9

#define WIFI_OK_BIT      BIT0

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint32_t sensor_id;
    uint32_t seq;
    uint16_t fs_hz;
    uint16_t n_samples;
    uint16_t sens_mg_x1000;
    uint16_t reserved;
} raw_hdr_t;
#pragma pack(pop)

extern SemaphoreHandle_t g_fifo_semaphore;

static EventGroupHandle_t s_wifi_eg;
static esp_netif_t *s_sta_netif;
static volatile bool s_got_ip;
static volatile bool s_wifi_need_reconnect;
static uint32_t s_seq;
static int s_sock = -1;
static struct sockaddr_in s_dest;
static char s_host[32] = RAW_HOST;
static uint16_t s_port = RAW_PORT;

static void boot_say(const char *msg)
{
    esp_rom_printf("RAW: %s\n", msg);
    ESP_LOGI(TAG, "%s", msg);
}

static uint32_t sensor_id_from_mac(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
           ((uint32_t)mac[4] << 8) | (uint32_t)mac[5];
}

static void lora_power_off(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << GPIO_LORA_EN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(GPIO_LORA_EN, 0);
}

static const char *wifi_disc_reason_str(uint8_t r)
{
    switch (r) {
    case WIFI_REASON_AUTH_EXPIRE: return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_FAIL: return "AUTH_FAIL";
    case WIFI_REASON_NO_AP_FOUND: return "NO_AP_FOUND";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL: return "CONNECTION_FAIL";
    case WIFI_REASON_ASSOC_FAIL: return "ASSOC_FAIL";
    case WIFI_REASON_BEACON_TIMEOUT: return "BEACON_TIMEOUT";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "4WAY_TIMEOUT";
    default: return "OTHER";
    }
}

static void wifi_scan_dump(void)
{
    wifi_scan_config_t sc = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) {
        boot_say("scan failed");
        return;
    }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 24) {
        n = 24;
    }
    wifi_ap_record_t *recs = calloc(n, sizeof(*recs));
    if (!recs) {
        return;
    }
    uint16_t got = n;
    if (esp_wifi_scan_get_ap_records(&got, recs) != ESP_OK) {
        free(recs);
        return;
    }
    bool found = false;
    esp_rom_printf("RAW: scan %u APs:\n", (unsigned)got);
    for (uint16_t i = 0; i < got; i++) {
        esp_rom_printf("  [%u] '%s' rssi=%d ch=%u auth=%u\n",
                       (unsigned)i, (char *)recs[i].ssid, (int)recs[i].rssi,
                       (unsigned)recs[i].primary, (unsigned)recs[i].authmode);
        if (strcmp((char *)recs[i].ssid, RAW_SSID) == 0) {
            found = true;
        }
    }
    free(recs);
    boot_say(found ? "LoRa-1 SEEN in scan" : "LoRa-1 NOT in scan (2.4GHz only!)");
}

static void wifi_on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        boot_say("wifi STA_START");
        esp_wifi_set_ps(WIFI_PS_NONE);
        s_wifi_need_reconnect = true;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        boot_say("wifi ASSOC OK → DHCP…");
        /* Power-save breaks DHCP on some APs (was pm type:1 in logs). */
        esp_wifi_set_ps(WIFI_PS_NONE);
        if (s_sta_netif) {
            esp_netif_dhcpc_stop(s_sta_netif);
            esp_netif_dhcpc_start(s_sta_netif);
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        s_got_ip = false;
        if (s_wifi_eg) {
            xEventGroupClearBits(s_wifi_eg, WIFI_OK_BIT);
        }
        esp_rom_printf("RAW: wifi DISC reason=%u (%s)\n",
                       (unsigned)e->reason, wifi_disc_reason_str(e->reason));
        /* Do not connect() here — reconnect task does backoff. */
        s_wifi_need_reconnect = true;
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        esp_rom_printf("RAW: got IP " IPSTR " gw " IPSTR " -> %s:%u\n",
                       IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw),
                       s_host, (unsigned)s_port);
        s_got_ip = true;
        s_wifi_need_reconnect = false;
        xEventGroupSetBits(s_wifi_eg, WIFI_OK_BIT);
    }
}

static void wifi_reconnect_task(void *arg)
{
    (void)arg;
    int attempt = 0;
    int64_t assoc_since_us = 0;
    while (true) {
        wifi_ap_record_t ap = {0};
        bool assoc = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);

        if (assoc && !s_got_ip) {
            if (assoc_since_us == 0) {
                assoc_since_us = esp_timer_get_time();
            } else if ((esp_timer_get_time() - assoc_since_us) > 20000000LL) {
                /* Associated but no DHCP for 20s — force reconnect. */
                boot_say("DHCP timeout → disconnect/retry");
                assoc_since_us = 0;
                esp_wifi_disconnect();
                s_wifi_need_reconnect = true;
            }
        } else if (!assoc) {
            assoc_since_us = 0;
        }

        if (!s_got_ip && s_wifi_need_reconnect && !assoc) {
            s_wifi_need_reconnect = false;
            attempt++;
            if ((attempt % 5) == 1) {
                wifi_scan_dump();
            }
            esp_wifi_set_ps(WIFI_PS_NONE);
            esp_rom_printf("RAW: wifi connect try #%d -> '%s'\n", attempt, RAW_SSID);
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                esp_rom_printf("RAW: connect() %s\n", esp_err_to_name(err));
                s_wifi_need_reconnect = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static esp_err_t wifi_start_sta(void)
{
    s_wifi_eg = xEventGroupCreate();
    if (!s_wifi_eg) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "netif_init %s", esp_err_to_name(err));
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event_loop %s", esp_err_to_name(err));
        return err;
    }
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (!s_sta_netif) {
        ESP_LOGE(TAG, "create STA netif failed");
        return ESP_FAIL;
    }
    esp_netif_set_hostname(s_sta_netif, "vibro-raw");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_on_event, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_on_event, NULL, NULL);

    wifi_config_t wcfg = {0};
    strncpy((char *)wcfg.sta.ssid, RAW_SSID, sizeof(wcfg.sta.ssid) - 1);
    strncpy((char *)wcfg.sta.password, RAW_PASS, sizeof(wcfg.sta.password) - 1);
    /* Accept WPA/WPA2/WPA3 — do not require WPA2-only threshold. */
    wcfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wcfg.sta.pmf_cfg.capable = true;
    wcfg.sta.pmf_cfg.required = false;
    wcfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wcfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wcfg.sta.failure_retry_cnt = 3;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }
    /* Critical: disable modem sleep — DHCP often fails with pm type:1. */
    esp_wifi_set_max_tx_power(60);
    esp_wifi_set_ps(WIFI_PS_NONE);

    xTaskCreate(wifi_reconnect_task, "wifi_rc", 4096, NULL, 5, NULL);
    boot_say("wifi_start OK, waiting IP (DHCP, PS off)…");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_OK_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(60000));
    return (bits & WIFI_OK_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t udp_open(void)
{
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_sock < 0) {
        return ESP_FAIL;
    }
    memset(&s_dest, 0, sizeof(s_dest));
    s_dest.sin_family = AF_INET;
    s_dest.sin_port = htons(s_port);
    if (!inet_aton(s_host, &s_dest.sin_addr)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_rom_printf("RAW: UDP -> %s:%u\n", s_host, (unsigned)s_port);
    return ESP_OK;
}

static esp_err_t send_chunk(uint32_t sid, float sens_mg, uint16_t fs,
                            const lsm_sample_t *s, uint16_t n)
{
    uint8_t pkt[sizeof(raw_hdr_t) + RAW_CHUNK * 6];
    if (n > RAW_CHUNK) {
        n = RAW_CHUNK;
    }
    raw_hdr_t *h = (raw_hdr_t *)pkt;
    h->magic = RAW_MAGIC;
    h->sensor_id = sid;
    h->seq = s_seq++;
    h->fs_hz = fs;
    h->n_samples = n;
    h->sens_mg_x1000 = (uint16_t)lroundf(sens_mg * 1000.0f);
    h->reserved = 0;
    int16_t *body = (int16_t *)(pkt + sizeof(raw_hdr_t));
    for (uint16_t i = 0; i < n; i++) {
        body[i * 3 + 0] = s[i].x;
        body[i * 3 + 1] = s[i].y;
        body[i * 3 + 2] = s[i].z;
    }
    int len = (int)(sizeof(raw_hdr_t) + (size_t)n * 6);
    int sent = sendto(s_sock, pkt, len, 0, (struct sockaddr *)&s_dest, sizeof(s_dest));
    return sent == len ? ESP_OK : ESP_FAIL;
}

static void stream_task(void *arg)
{
    (void)arg;
    const uint32_t sid = sensor_id_from_mac();
    const float fs = accel_odr_to_hz(RAW_ODR);
    float sens = 0.122f;
    int log_n = 0;

    boot_say("stream task start");
    vTaskDelay(pdMS_TO_TICKS(500));

    while (true) {
        if (!s_got_ip) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (s_sock < 0 && udp_open() != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!lsm6ds3trc_is_initialized()) {
            if (lsm6ds3trc_init(RAW_RANGE, RAW_ODR, &sens) != ESP_OK) {
                ESP_LOGE(TAG, "LSM init fail");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            boot_say("LSM init OK");
        }

        const size_t want = RAW_CHUNK * 2;
        lsm_sample_t *buf = malloc(want * sizeof(lsm_sample_t));
        if (!buf) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (lsm6ds3trc_start_fifo_collection(buf, want) != ESP_OK) {
            free(buf);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        size_t total = 0;
        int64_t deadline = esp_timer_get_time() + 1500000LL;
        while (total < want && esp_timer_get_time() < deadline) {
            if (g_fifo_semaphore) {
                xSemaphoreTake(g_fifo_semaphore, pdMS_TO_TICKS(20));
            } else {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            uint32_t avail = lsm6ds3trc_get_fifo_samples();
            if (!avail) {
                continue;
            }
            size_t room = want - total;
            size_t nread = 0;
            size_t to_read = avail < room ? avail : room;
            if (lsm6ds3trc_read_fifo_samples(buf + total, &nread, to_read) == ESP_OK) {
                total += nread;
            }
        }
        lsm6ds3trc_stop_fifo_collection();

        int ok = 0;
        for (size_t off = 0; off + RAW_CHUNK <= total; off += RAW_CHUNK) {
            if (send_chunk(sid, sens, (uint16_t)fs, buf + off, RAW_CHUNK) == ESP_OK) {
                ok++;
            }
        }
        free(buf);

        if ((++log_n % 20) == 0) {
            ESP_LOGI(TAG, "tx chunks=%d seq=%lu", ok, (unsigned long)s_seq);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    esp_rom_printf("\n\n=== vibro-raw-wifi boot ===\n");
    boot_say("app_main");

    /* Let USB-Serial-JTAG enumerate before WiFi RF load. */
    vTaskDelay(pdMS_TO_TICKS(1500));

    lora_power_off();
    boot_say("LoRa EN off");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs %s (continue)", esp_err_to_name(err));
    }

    spi_bus_config_t buscfg = {
        .miso_io_num = LSM6DS3TRC_PIN_MISO,
        .mosi_io_num = LSM6DS3TRC_PIN_MOSI,
        .sclk_io_num = LSM6DS3TRC_PIN_SCK,
        .max_transfer_sz = 4096,
    };
    err = spi_bus_initialize(LSM6DS3TRC_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI %s", esp_err_to_name(err));
        /* Don't abort — keep printing so monitor shows the fault. */
        while (true) {
            boot_say("SPI init failed — halt loop");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    boot_say("SPI OK");

    gpio_install_isr_service(0);

    err = wifi_start_sta();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi not ready yet (%s) — stream waits for IP", esp_err_to_name(err));
    } else {
        boot_say("WiFi IP OK");
    }

    xTaskCreate(stream_task, "raw_tx", 8192, NULL, 5, NULL);
    boot_say("stream task created");
}
