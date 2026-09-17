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
#define RAW_ODR          ACCEL_ODR_1660HZ
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
static bool s_got_ip;
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

static void wifi_on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        boot_say("wifi STA_START → connect()");
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_got_ip = false;
        /* Never block in event handler — that caused reboot loops. */
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        esp_rom_printf("RAW: got IP " IPSTR "\n", IP2STR(&e->ip_info.ip));
        s_got_ip = true;
        xEventGroupSetBits(s_wifi_eg, WIFI_OK_BIT);
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
    if (!esp_netif_create_default_wifi_sta()) {
        ESP_LOGE(TAG, "create STA netif failed");
        return ESP_FAIL;
    }

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
    wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wcfg.sta.pmf_cfg.capable = true;
    wcfg.sta.pmf_cfg.required = false;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }
    /* After start: lower TX power — USB + WiFi brownouts were rebooting the board. */
    esp_wifi_set_max_tx_power(40); /* ~10 dBm */
    boot_say("wifi_start OK, waiting IP…");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_OK_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(30000));
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
