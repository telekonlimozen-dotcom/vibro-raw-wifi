#include "lsm6ds3trc.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "LSM6DS3TRC";

// === Глобальные переменные ===
static spi_device_handle_t g_spi = NULL;
static bool g_initialized = false;
static bool g_int1_isr_installed = false;
static float g_sensitivity_mg_lsb = 0.122f;
static lsm_sample_t *g_fifo_buffer = NULL;
static size_t g_fifo_buffer_size = 0;
static volatile bool g_fifo_active = false;
static lsm_fifo_callback_t g_fifo_callback = NULL;
static accel_odr_t g_current_odr = ACCEL_ODR_3330HZ;
static accel_range_t g_current_range = ACCEL_RANGE_4G;
static bool g_soft_sleeping = false;

SemaphoreHandle_t g_fifo_semaphore = NULL;

// === ODR to register value ===
static uint8_t lsm6ds3trc_odr_to_reg_value(accel_odr_t odr) {
    switch (odr) {
        case ACCEL_ODR_12_5HZ:  return 0x01 << 4;
        case ACCEL_ODR_26HZ:    return 0x02 << 4;
        case ACCEL_ODR_52HZ:    return 0x03 << 4;
        case ACCEL_ODR_104HZ:   return 0x04 << 4;
        case ACCEL_ODR_208HZ:   return 0x05 << 4;
        case ACCEL_ODR_416HZ:   return 0x06 << 4;
        case ACCEL_ODR_833HZ:   return 0x07 << 4;
        case ACCEL_ODR_1660HZ:  return 0x08 << 4;
        case ACCEL_ODR_3330HZ:  return 0x09 << 4;
        case ACCEL_ODR_6660HZ:  return 0x0A << 4;
        default:                return 0x04 << 4;
    }
}

// === FIFO ODR to register value ===
static uint8_t lsm6ds3trc_fifo_odr_to_reg_value(accel_odr_t odr) {
    switch (odr) {
        case ACCEL_ODR_12_5HZ:  return 0x01 << 3;
        case ACCEL_ODR_26HZ:    return 0x02 << 3;
        case ACCEL_ODR_52HZ:    return 0x03 << 3;
        case ACCEL_ODR_104HZ:   return 0x04 << 3;
        case ACCEL_ODR_208HZ:   return 0x05 << 3;
        case ACCEL_ODR_416HZ:   return 0x06 << 3;
        case ACCEL_ODR_833HZ:   return 0x07 << 3;
        case ACCEL_ODR_1660HZ:  return 0x08 << 3;
        case ACCEL_ODR_3330HZ:  return 0x09 << 3;
        case ACCEL_ODR_6660HZ:  return 0x0A << 3;
        default:                return 0x04 << 3;
    }
}

// === Get sensitivity from full scale ===
static float lsm6ds3trc_get_sensitivity(accel_range_t range) {
    switch (range) {
        case ACCEL_RANGE_2G:  return LSM6DS3TRC_SENSITIVITY_FS_2G;
        case ACCEL_RANGE_4G:  return LSM6DS3TRC_SENSITIVITY_FS_4G;
        case ACCEL_RANGE_8G:  return LSM6DS3TRC_SENSITIVITY_FS_8G;
        case ACCEL_RANGE_16G: return LSM6DS3TRC_SENSITIVITY_FS_16G;
        default:              return LSM6DS3TRC_SENSITIVITY_FS_4G;
    }
}

// === Get sample interval in microseconds ===
static uint32_t lsm6ds3trc_get_sample_interval_us(accel_odr_t odr) {
    switch (odr) {
        case ACCEL_ODR_12_5HZ:  return 80000;
        case ACCEL_ODR_26HZ:    return 38461;
        case ACCEL_ODR_52HZ:    return 19230;
        case ACCEL_ODR_104HZ:   return 9615;
        case ACCEL_ODR_208HZ:   return 4807;
        case ACCEL_ODR_416HZ:   return 2403;
        case ACCEL_ODR_833HZ:   return 1200;
        case ACCEL_ODR_1660HZ:  return 602;
        case ACCEL_ODR_3330HZ:  return 300;
        case ACCEL_ODR_6660HZ:  return 150;
        default:                return 9615;
    }
}

// === Temperature conversion ===
float lsm6ds3trc_temp_to_celsius(int16_t raw) {
    return ((float)raw / 256.0f) + 25.0f;
}

// === Check INT1 pin level ===
bool lsm6ds3trc_check_data_ready_pin(void) {
    return gpio_get_level(LSM6DS3TRC_PIN_INT1) != 0;
}

// === SPI Read (single) ===
static esp_err_t lsm6ds3trc_spi_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0 || !g_spi) return ESP_OK;
    
    spi_transaction_t t = {0};
    uint8_t cmd = reg_addr | 0x80;
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];
    
    tx_buf[0] = cmd;
    memset(&tx_buf[1], 0xFF, len);
    
    t.length = 8 * (len + 1);
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    
    gpio_set_level(LSM6DS3TRC_PIN_CS, 0);
    esp_err_t ret = spi_device_polling_transmit(g_spi, &t);
    gpio_set_level(LSM6DS3TRC_PIN_CS, 1);
    
    if (ret == ESP_OK) {
        memcpy(data, &rx_buf[1], len);
    }
    
    return ret;
}

// === SPI Write ===
static esp_err_t lsm6ds3trc_spi_write(uint8_t reg_addr, uint8_t data) {
    if (!g_spi) return ESP_ERR_INVALID_STATE;
    
    spi_transaction_t t = {0};
    uint8_t tx_buf[2] = {reg_addr & 0x7F, data};
    uint8_t rx_dummy[2];
    
    t.length = 16;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_dummy;
    
    gpio_set_level(LSM6DS3TRC_PIN_CS, 0);
    esp_err_t ret = spi_device_polling_transmit(g_spi, &t);
    gpio_set_level(LSM6DS3TRC_PIN_CS, 1);
    
    return ret;
}

// === SPI Read Burst (multiple bytes) ===
static esp_err_t lsm6ds3trc_spi_read_burst(uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0 || !g_spi) return ESP_OK;
    
    /* LSM6DS3TR-C SPI: bit7 = read. Auto-increment via CTRL3_C IF_INC (not LIS3DH MS/0xC0). */
    uint8_t cmd = reg_addr | 0x80;
    uint8_t *tx_buf = malloc(len + 1);
    uint8_t *rx_buf = malloc(len + 1);
    
    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }
    
    tx_buf[0] = cmd;
    memset(&tx_buf[1], 0xFF, len);
    
    spi_transaction_t t = {0};
    t.length = 8 * (len + 1);
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    
    gpio_set_level(LSM6DS3TRC_PIN_CS, 0);
    esp_err_t ret = spi_device_polling_transmit(g_spi, &t);
    gpio_set_level(LSM6DS3TRC_PIN_CS, 1);
    
    if (ret == ESP_OK) {
        memcpy(data, &rx_buf[1], len);
    }
    
    free(tx_buf);
    free(rx_buf);
    return ret;
}

// === INT1 Interrupt Handler ===
static void IRAM_ATTR lsm6ds3trc_int1_isr(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (g_fifo_semaphore != NULL) {
        xSemaphoreGiveFromISR(g_fifo_semaphore, &xHigherPriorityTaskWoken);
    }
    
    if (g_fifo_callback) {
        g_fifo_callback();
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// === Get FIFO fill level (number of 16-bit words) ===
uint32_t lsm6ds3trc_get_fifo_words(void) {
    if (!g_initialized) return 0;
    
    uint8_t status1, status2;
    
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS1, &status1, 1) != ESP_OK) return 0;
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS2, &status2, 1) != ESP_OK) return 0;
    
    uint32_t words = ((uint32_t)(status2 & 0x07) << 8) | status1;
    
    return words;
}

// === Get number of complete samples in FIFO ===
uint32_t lsm6ds3trc_get_fifo_samples(void) {
    uint32_t words = lsm6ds3trc_get_fifo_words();
    if (words == 0) {
        return 0;
    }
    /* Accel-only pattern: 0=XLx, 1=XLy, 2=XLz (AN5130 §8.5). */
    uint16_t pattern = lsm6ds3trc_get_fifo_pattern() % 3u;
    uint32_t skip = (3u - pattern) % 3u;
    if (words <= skip) {
        return 0;
    }
    return (words - skip) / 3u;
}

// === Get FIFO pattern (next data to read) ===
uint16_t lsm6ds3trc_get_fifo_pattern(void) {
    if (!g_initialized) return 0;
    
    uint8_t status3, status4;
    
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS3, &status3, 1) != ESP_OK) return 0;
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS4, &status4, 1) != ESP_OK) return 0;
    
    uint16_t pattern = ((uint16_t)(status4 & 0x03) << 8) | status3;
    
    return pattern;
}

/** Pop one 16-bit FIFO word (L then H). Accel-only: advances pattern 0→1→2→0. */
static esp_err_t lsm6ds3trc_fifo_pop_word(void)
{
    uint8_t dummy[2];
    return lsm6ds3trc_spi_read_burst(LSM6DS3TRC_ADDR_FIFO_DATA_OUT_L, dummy, sizeof(dummy));
}

/**
 * Align so next FIFO word is accelerometer X (pattern % 3 == 0).
 * AN5130: XL-only order is XLx, XLy, XLz.
 */
static esp_err_t lsm6ds3trc_fifo_align_xl_x(void)
{
    uint16_t pattern = lsm6ds3trc_get_fifo_pattern() % 3u;
    uint32_t skip = (3u - pattern) % 3u;
    for (uint32_t i = 0; i < skip; i++) {
        if (lsm6ds3trc_get_fifo_words() == 0) {
            return ESP_ERR_INVALID_STATE;
        }
        esp_err_t err = lsm6ds3trc_fifo_pop_word();
        if (err != ESP_OK) {
            return err;
        }
    }
    if (skip > 0) {
        ESP_LOGW(TAG, "FIFO realigned: discarded %u word(s) (was pattern=%u)",
                 (unsigned)skip, (unsigned)pattern);
    }
    return ESP_OK;
}

// === Get FIFO status flags ===
uint8_t lsm6ds3trc_get_fifo_status(void) {
    if (!g_initialized) return 0;
    
    uint8_t status2;
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS2, &status2, 1) != ESP_OK) return 0;
    
    return status2;
}

// === Check if watermark reached ===
bool lsm6ds3trc_is_watermark_reached(void) {
    if (!g_initialized) return false;
    
    uint8_t status2;
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS2, &status2, 1) != ESP_OK) return false;
    
    return (status2 & LSM6DS3TRC_FIFO_STATUS2_WATERM) != 0;
}

// === Check if FIFO full ===
bool lsm6ds3trc_is_fifo_full(void) {
    if (!g_initialized) return false;
    
    uint8_t status2;
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS2, &status2, 1) != ESP_OK) return false;
    
    return (status2 & LSM6DS3TRC_FIFO_STATUS2_FULL_SMART) != 0;
}

// === Очистка FIFO ===
esp_err_t lsm6ds3trc_clear_fifo(void) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    uint32_t fifo_words;
    int timeout = 0;
    const int MAX_TIMEOUT = 1000;
    
    do {
        fifo_words = lsm6ds3trc_get_fifo_words();
        if (fifo_words == 0) {
            break;
        }

        /* Drain word-by-word so a leftover axis cannot leave pattern mid-frame. */
        uint32_t to_pop = fifo_words;
        if (to_pop > 96) {
            to_pop = 96;
        }
        for (uint32_t i = 0; i < to_pop; i++) {
            if (lsm6ds3trc_fifo_pop_word() != ESP_OK) {
                break;
            }
        }
        
        timeout++;
        if (timeout > MAX_TIMEOUT) break;
        
    } while (lsm6ds3trc_get_fifo_words() > 0);
    
    return ESP_OK;
}

// === Read FIFO samples ===
esp_err_t lsm6ds3trc_read_fifo_samples(lsm_sample_t *buffer, size_t *num_samples, size_t max_samples) {
    if (!buffer || !num_samples || !g_initialized) return ESP_ERR_INVALID_ARG;
    
    *num_samples = 0;
    
    uint32_t fifo_words = lsm6ds3trc_get_fifo_words();
    if (fifo_words == 0) return ESP_OK;

    uint16_t pattern_raw = lsm6ds3trc_get_fifo_pattern();
    uint16_t pattern = pattern_raw % 3u;
    if (pattern != 0) {
        esp_err_t align_err = lsm6ds3trc_fifo_align_xl_x();
        if (align_err != ESP_OK) {
            return ESP_OK; /* not enough words left for a full XYZ */
        }
        fifo_words = lsm6ds3trc_get_fifo_words();
        pattern_raw = lsm6ds3trc_get_fifo_pattern();
        pattern = pattern_raw % 3u;
        if (pattern != 0 || fifo_words < 3) {
            ESP_LOGW(TAG, "FIFO still misaligned after realign (pat=%u words=%u)",
                     (unsigned)pattern_raw, (unsigned)fifo_words);
            return ESP_OK;
        }
    }
    
    uint32_t available_samples = fifo_words / 3;
    if (available_samples == 0) {
        ESP_LOGW(TAG, "Incomplete sample in FIFO (%d words, pattern=%u)",
                 (unsigned)fifo_words, (unsigned)pattern_raw);
        return ESP_OK;
    }
    
    ESP_LOGD(TAG, "FIFO pattern: %u, words: %u, samples: %u",
             (unsigned)pattern_raw, (unsigned)fifo_words, (unsigned)available_samples);
    
    size_t samples_to_read = (available_samples < max_samples) ? available_samples : max_samples;
    
    uint8_t *fifo_data = malloc(6 * samples_to_read);
    if (!fifo_data) return ESP_ERR_NO_MEM;
    
    esp_err_t ret = lsm6ds3trc_spi_read_burst(LSM6DS3TRC_ADDR_FIFO_DATA_OUT_L, 
                                               fifo_data, 6 * samples_to_read);
    
    if (ret != ESP_OK) {
        free(fifo_data);
        return ret;
    }
    
    uint32_t current_time = esp_timer_get_time();
    uint32_t sample_interval = lsm6ds3trc_get_sample_interval_us(g_current_odr);
    
    /* Accel-only: each frame is XLx, XLy, XLz (16-bit LE words). */
    for (size_t i = 0; i < samples_to_read; i++) {
        buffer[i].x = (int16_t)(fifo_data[i*6 + 1] << 8 | fifo_data[i*6]);
        buffer[i].y = (int16_t)(fifo_data[i*6 + 3] << 8 | fifo_data[i*6 + 2]);
        buffer[i].z = (int16_t)(fifo_data[i*6 + 5] << 8 | fifo_data[i*6 + 4]);
        
        uint32_t sample_age_us = (samples_to_read - i - 1) * sample_interval;
        buffer[i].timestamp_us = current_time - sample_age_us;
    }
    
    free(fifo_data);
    *num_samples = samples_to_read;
    
    return ESP_OK;
}

// === Configure FIFO ===
esp_err_t lsm6ds3trc_configure_fifo(uint16_t watermark_level) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGD(TAG, "Configuring FIFO: watermark=%d words", watermark_level);

    esp_err_t ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL1, watermark_level & 0xFF);
    if (ret != ESP_OK) return ret;
    
    uint8_t ctrl2_val = (watermark_level >> 8) & 0x07;
    ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL2, ctrl2_val);
    if (ret != ESP_OK) return ret;

    ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL3, 0x01);
    if (ret != ESP_OK) return ret;

    ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL4, 0x80);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t lsm6ds3trc_timestamp_enable(void)
{
    if (!g_initialized && g_spi == NULL) {
        /* Allow call during init after SPI is up. */
    }
    uint8_t dur = 0;
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_WAKE_UP_DUR, &dur, 1) != ESP_OK) {
        return ESP_FAIL;
    }
    /* TIMER_HR=1 → 25 µs/LSB (AN5130). */
    dur = (uint8_t)((dur & ~(1u << 4)) | (1u << 4));
    if (lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_WAKE_UP_DUR, dur) != ESP_OK) {
        return ESP_FAIL;
    }
    /* CTRL10_C: FUNC_EN | TIMER_EN */
    if (lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL10_C, (1u << 2) | (1u << 5)) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t lsm6ds3trc_read_timestamp_ticks(uint32_t *ticks)
{
    if (!ticks) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t b[3] = {0};
    /* IF_INC: burst TIMESTAMP0..2 */
    if (lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_TIMESTAMP0, b, 3) != ESP_OK) {
        return ESP_FAIL;
    }
    *ticks = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
    return ESP_OK;
}

// === Start FIFO collection ===
esp_err_t lsm6ds3trc_start_fifo_collection(lsm_sample_t *buffer, size_t max_samples) {
    if (!buffer || max_samples == 0 || !g_initialized) return ESP_ERR_INVALID_ARG;

    g_fifo_buffer = buffer;
    g_fifo_buffer_size = max_samples;
    g_fifo_active = true;

    ESP_LOGI(TAG, "Starting FIFO collection with max_samples=%d", max_samples);

    if (g_fifo_semaphore == NULL) {
        g_fifo_semaphore = xSemaphoreCreateBinary();
        if (!g_fifo_semaphore) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(g_fifo_semaphore, 0);

    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, 0x00);
    gpio_intr_disable(LSM6DS3TRC_PIN_INT1);

    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL5, LSM6DS3TRC_FIFO_MODE_BYPASS);
    vTaskDelay(pdMS_TO_TICKS(1));

    lsm6ds3trc_clear_fifo();

    uint16_t watermark_words = max_samples * 3;
    /* Hardware watermark field is 11-bit (max 2047 words ≈ 682 frames).
     * For N>682 use chunk watermark and let caller drain repeatedly. */
    if (watermark_words > 1536) {
        watermark_words = 1536; /* ~512 frames */
    }
    
    esp_err_t ret = lsm6ds3trc_configure_fifo(watermark_words);
    if (ret != ESP_OK) return ret;

    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_DRDY_PULSE_CFG_G, 0x80);

    uint8_t fifo_odr = lsm6ds3trc_fifo_odr_to_reg_value(g_current_odr);
    uint8_t fifo_ctrl5_val = fifo_odr | LSM6DS3TRC_FIFO_MODE_CONTINUOUS;
    ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL5, fifo_ctrl5_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set FIFO mode: %s", esp_err_to_name(ret));
        return ret;
    }

    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, LSM6DS3TRC_INT1_FIFO_THS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable FIFO interrupt: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_intr_enable(LSM6DS3TRC_PIN_INT1);

    return ESP_OK;
}

// === Stop FIFO collection ===
esp_err_t lsm6ds3trc_stop_fifo_collection(void) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    g_fifo_active = false;
    
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, 0x00);
    gpio_intr_disable(LSM6DS3TRC_PIN_INT1);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL5, LSM6DS3TRC_FIFO_MODE_BYPASS);
    lsm6ds3trc_clear_fifo();
    
    ESP_LOGI(TAG, "FIFO collection stopped");
    return ESP_OK;
}

// === Set FIFO callback ===
esp_err_t lsm6ds3trc_set_fifo_callback(lsm_fifo_callback_t callback) {
    g_fifo_callback = callback;
    return ESP_OK;
}

// === Check if data is ready via STATUS_REG ===
esp_err_t lsm6ds3trc_is_data_ready(bool *ready) {
    if (!ready || !g_initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t status;
    esp_err_t ret = lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_STATUS_REG, &status, 1);
    if (ret != ESP_OK) return ret;
    
    *ready = (status & LSM6DS3TRC_STATUS_XLDA) != 0;
    return ESP_OK;
}

// === Read raw accelerometer data ===
esp_err_t lsm6ds3trc_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) {
    if (!ax || !ay || !az || !g_initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t data[6];
    esp_err_t ret = lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_OUTX_L_XL, data, 6);
    if (ret != ESP_OK) return ret;
    
    *ax = (int16_t)(data[1] << 8 | data[0]);
    *ay = (int16_t)(data[3] << 8 | data[2]);
    *az = (int16_t)(data[5] << 8 | data[4]);
    
    return ESP_OK;
}

// === Read accelerometer data in g units ===
esp_err_t lsm6ds3trc_read_accel_g(float *ax_g, float *ay_g, float *az_g, float sensitivity_mg_lsb) {
    if (!ax_g || !ay_g || !az_g) return ESP_ERR_INVALID_ARG;
    
    int16_t x, y, z;
    esp_err_t ret = lsm6ds3trc_read_accel_raw(&x, &y, &z);
    if (ret != ESP_OK) return ret;
    
    *ax_g = x * sensitivity_mg_lsb / 1000.0f;
    *ay_g = y * sensitivity_mg_lsb / 1000.0f;
    *az_g = z * sensitivity_mg_lsb / 1000.0f;
    
    return ESP_OK;
}

// === Read temperature raw ===
esp_err_t lsm6ds3trc_read_temp_raw(int16_t *temp) {
    if (!temp || !g_initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t data[2];
    esp_err_t ret = lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_OUT_TEMP_L, data, 2);
    if (ret != ESP_OK) return ret;
    
    *temp = (int16_t)(data[1] << 8 | data[0]);
    
    return ESP_OK;
}

// === Wait for data ready and read ===
esp_err_t lsm6ds3trc_read_accel_raw_wait(int16_t *ax, int16_t *ay, int16_t *az, uint32_t timeout_ms) {
    if (!ax || !ay || !az || !g_initialized) return ESP_ERR_INVALID_STATE;
    
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    
    while (1) {
        if (lsm6ds3trc_check_data_ready_pin()) {
            return lsm6ds3trc_read_accel_raw(ax, ay, az);
        }
        
        bool ready;
        esp_err_t ret = lsm6ds3trc_is_data_ready(&ready);
        if (ret == ESP_OK && ready) {
            return lsm6ds3trc_read_accel_raw(ax, ay, az);
        }
        
        if ((xTaskGetTickCount() - start) > timeout) {
            return ESP_ERR_TIMEOUT;
        }
        taskYIELD();
    }
}

// === Software reset ===
static esp_err_t lsm6ds3trc_software_reset(void) {
    ESP_LOGI(TAG, "Performing software reset...");
    
    esp_err_t ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL3_C, LSM6DS3TRC_CTRL3_SW_RESET);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    uint8_t ctrl3;
    int timeout = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL3_C, &ctrl3, 1);
        timeout++;
        if (timeout > 50) {
            ESP_LOGW(TAG, "Software reset timeout");
            break;
        }
    } while (ctrl3 & LSM6DS3TRC_CTRL3_SW_RESET);
    
    return ESP_OK;
}

// === Initialize LSM6DS3TR-C ===
esp_err_t lsm6ds3trc_init(accel_range_t accel_range,
                          accel_odr_t accel_odr,
                          float *actual_sensitivity_mg_lsb) {
    
    if (g_initialized && g_spi != NULL) {
        /* Same ODR/FS and not soft-sleeping: do NOT rewrite CTRL1_XL.
         * Calib used to call init every 3 s → "reconfiguring" → XL digital
         * transient (~1 g AC, random f) — same desk garbage as soft-reset. */
        if (!g_soft_sleeping &&
            g_current_odr == accel_odr &&
            g_current_range == accel_range) {
            if (actual_sensitivity_mg_lsb) {
                *actual_sensitivity_mg_lsb = g_sensitivity_mg_lsb;
            }
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Already initialized, reconfiguring...");
        
        g_current_odr = accel_odr;
        g_current_range = accel_range;

        uint8_t fs_reg;
        switch (accel_range) {
            case ACCEL_RANGE_2G:  g_sensitivity_mg_lsb = 0.061f; fs_reg = 0x00; break;
            case ACCEL_RANGE_4G:  g_sensitivity_mg_lsb = 0.122f; fs_reg = 0x08; break;
            case ACCEL_RANGE_8G:  g_sensitivity_mg_lsb = 0.244f; fs_reg = 0x0C; break;
            case ACCEL_RANGE_16G: g_sensitivity_mg_lsb = 0.488f; fs_reg = 0x04; break;
            default:              g_sensitivity_mg_lsb = 0.122f; fs_reg = 0x08; break;
        }
        
        uint8_t odr_reg = lsm6ds3trc_odr_to_reg_value(accel_odr);
        /* BW0_XL=0, LPF1_BW_SEL=0 → digital BW = ODR/2 (keeps 1000 Hz @ 3.33k). */
        lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL1_XL, odr_reg | fs_reg);
        /* Composite filters off: no LPF2 / HP (AN5130 Table 9 → ODR/2). */
        lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL8_XL, 0x00);
        uint8_t ctrl6_c;
        lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL6_C, &ctrl6_c, 1);
        ctrl6_c &= ~(1 << 4); /* XL_HM_MODE=0 → high-performance */
        lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL6_C, ctrl6_c);
        (void)lsm6ds3trc_timestamp_enable();
        g_soft_sleeping = false;
        
        if (actual_sensitivity_mg_lsb) {
            *actual_sensitivity_mg_lsb = g_sensitivity_mg_lsb;
        }
        
        return ESP_OK;
    }

    g_current_odr = accel_odr;
    g_current_range = accel_range;
    g_soft_sleeping = false;

    ESP_LOGI(TAG, "Initializing LSM6DS3TR-C...");
    
    if (g_fifo_semaphore == NULL) {
        g_fifo_semaphore = xSemaphoreCreateBinary();
        if (!g_fifo_semaphore) return ESP_ERR_NO_MEM;
    }

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LSM6DS3TRC_PIN_CS),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    gpio_set_level(LSM6DS3TRC_PIN_CS, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_config_t int_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LSM6DS3TRC_PIN_INT1),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&int_conf);
    if (!g_int1_isr_installed) {
        esp_err_t isr_err = gpio_isr_handler_add(LSM6DS3TRC_PIN_INT1, lsm6ds3trc_int1_isr, NULL);
        if (isr_err == ESP_OK) {
            g_int1_isr_installed = true;
        } else if (isr_err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "INT1 ISR add failed: %s", esp_err_to_name(isr_err));
            return isr_err;
        } else {
            g_int1_isr_installed = true; /* already registered */
        }
    }
    gpio_intr_disable(LSM6DS3TRC_PIN_INT1);

    if (g_spi == NULL) {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 8 * 1000 * 1000,
            .mode = 0,
            .spics_io_num = -1,
            .queue_size = 7,
        };
        
        esp_err_t ret = spi_bus_add_device(LSM6DS3TRC_SPI_HOST, &devcfg, &g_spi);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* SoftAP/ESP-NOW + repeated power-cycle can leave SPI desynced; retry WHO_AM_I. */
    uint8_t who_am_i = 0;
    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < 5; attempt++) {
        gpio_set_level(LSM6DS3TRC_PIN_CS, 1);
        vTaskDelay(pdMS_TO_TICKS(5 + attempt * 10));
        ret = lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_WHO_AM_I, &who_am_i, 1);
        if (ret == ESP_OK && who_am_i == LSM6DS3TRC_WHO_AM_I_VALUE) {
            break;
        }
        ESP_LOGW(TAG, "WHO_AM_I try %d: ret=%s val=0x%02X",
                 attempt + 1, esp_err_to_name(ret), who_am_i);
    }
    if (ret != ESP_OK || who_am_i != LSM6DS3TRC_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x6A", who_am_i);
        if (g_spi) {
            spi_bus_remove_device(g_spi);
            g_spi = NULL;
        }
        if (g_int1_isr_installed) {
            gpio_isr_handler_remove(LSM6DS3TRC_PIN_INT1);
            g_int1_isr_installed = false;
        }
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who_am_i);

    ret = lsm6ds3trc_software_reset();
    if (ret != ESP_OK) return ret;

    ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL3_C, 
                               LSM6DS3TRC_CTRL3_BDU | LSM6DS3TRC_CTRL3_IF_INC);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set CTRL3_C");
        return ret;
    }

    ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL9_XL, 0x38);
    if (ret != ESP_OK) return ret;

    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL2_G, 0x00);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL7_G, 0x00);
    (void)lsm6ds3trc_timestamp_enable();

    g_sensitivity_mg_lsb = lsm6ds3trc_get_sensitivity(accel_range);
    
    uint8_t fs_reg;
    switch (accel_range) {
        case ACCEL_RANGE_2G:  fs_reg = 0x00; break;
        case ACCEL_RANGE_4G:  fs_reg = 0x08; break;
        case ACCEL_RANGE_8G:  fs_reg = 0x0C; break;
        case ACCEL_RANGE_16G: fs_reg = 0x04; break;
        default:              fs_reg = 0x08; break;
    }
    
    uint8_t odr_reg = lsm6ds3trc_odr_to_reg_value(accel_odr);
    /* BW0_XL=0, LPF1_BW_SEL=0 → ODR/2. Do NOT set LPF2 (ODR/4…/50 cuts Fmax). */
    ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL1_XL, odr_reg | fs_reg);
    if (ret != ESP_OK) return ret;
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL8_XL, 0x00);

    // Проверка установленного ODR
    uint8_t ctrl1_xl_check;
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL1_XL, &ctrl1_xl_check, 1);
    uint8_t odr_reg_value = (ctrl1_xl_check >> 4) & 0x0F;
    
    const char *odr_string;
    switch (odr_reg_value) {
        case 0x01: odr_string = "12.5 Hz"; break;
        case 0x02: odr_string = "26 Hz"; break;
        case 0x03: odr_string = "52 Hz"; break;
        case 0x04: odr_string = "104 Hz"; break;
        case 0x05: odr_string = "208 Hz"; break;
        case 0x06: odr_string = "416 Hz"; break;
        case 0x07: odr_string = "833 Hz"; break;
        case 0x08: odr_string = "1.66 kHz"; break;
        case 0x09: odr_string = "3.33 kHz"; break;
        case 0x0A: odr_string = "6.66 kHz"; break;
        default: odr_string = "UNKNOWN"; break;
    }
    
    ESP_LOGI(TAG, "CTRL1_XL = 0x%02X, ODR reg value = 0x%X (%s)", 
             ctrl1_xl_check, odr_reg_value, odr_string);

    uint8_t ctrl6_c;
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL6_C, &ctrl6_c, 1);
    ctrl6_c &= ~(1 << 4); /* high-performance mode */
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL6_C, ctrl6_c);

    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_DRDY_PULSE_CFG_G, 0x80);

    if (actual_sensitivity_mg_lsb) {
        *actual_sensitivity_mg_lsb = g_sensitivity_mg_lsb;
    }

    g_initialized = true;
    
    ESP_LOGI(TAG, "LSM6DS3TR-C initialized (FS=%d, ODR=%d, Sens=%.3f mg/LSB)",
             accel_range, accel_odr, g_sensitivity_mg_lsb);

    return ESP_OK;
}

bool lsm6ds3trc_is_initialized(void)
{
    return g_initialized && g_spi != NULL;
}

// === Deinitialize ===
esp_err_t lsm6ds3trc_deinit(void) {
    if (!g_initialized) return ESP_OK;

    if (g_fifo_active) lsm6ds3trc_stop_fifo_collection();

    if (g_int1_isr_installed) {
        gpio_isr_handler_remove(LSM6DS3TRC_PIN_INT1);
        g_int1_isr_installed = false;
    }

    /* Power-down: CTRL1_XL=0x00 (ODR off), CTRL2_G=0x00, disable INT/FIFO (~3 uA). */
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, 0x00);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL5, 0x00);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL1_XL, 0x00);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL2_G, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));

    if (g_spi) {
        spi_bus_remove_device(g_spi);
        g_spi = NULL;
    }

    if (g_fifo_semaphore) {
        vSemaphoreDelete(g_fifo_semaphore);
        g_fifo_semaphore = NULL;
    }

    g_initialized = false;
    g_soft_sleeping = false;
    ESP_LOGI(TAG, "LSM6DS3TR-C power-down");
    return ESP_OK;
}

esp_err_t lsm6ds3trc_soft_sleep(void)
{
    if (!g_initialized || !g_spi) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_fifo_active) {
        lsm6ds3trc_stop_fifo_collection();
    }
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, 0x00);
    gpio_intr_disable(LSM6DS3TRC_PIN_INT1);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL5, LSM6DS3TRC_FIFO_MODE_BYPASS);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL1_XL, 0x00);
    g_soft_sleeping = true;
    return ESP_OK;
}

esp_err_t lsm6ds3trc_soft_wake(void)
{
    if (!g_initialized || !g_spi) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t fs_reg;
    switch (g_current_range) {
        case ACCEL_RANGE_2G:  fs_reg = 0x00; break;
        case ACCEL_RANGE_4G:  fs_reg = 0x08; break;
        case ACCEL_RANGE_8G:  fs_reg = 0x0C; break;
        case ACCEL_RANGE_16G: fs_reg = 0x04; break;
        default:              fs_reg = 0x08; break;
    }
    uint8_t odr_reg = lsm6ds3trc_odr_to_reg_value(g_current_odr);
    esp_err_t ret = lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_CTRL1_XL, odr_reg | fs_reg);
    if (ret != ESP_OK) {
        return ret;
    }
    (void)lsm6ds3trc_timestamp_enable();
    /* XL digital filter settle ~14 samples (AN5130). */
    vTaskDelay(pdMS_TO_TICKS(20));
    g_soft_sleeping = false;
    return ESP_OK;
}

// === ДИАГНОСТИКА РЕЖИМОВ АКСЕЛЕРОМЕТРА ===
esp_err_t lsm6ds3trc_diagnostic_modes(lsm_diagnostic_t *diag) {
    if (!diag || !g_initialized) return ESP_ERR_INVALID_STATE;
    
    diag->xl_odr_raw = (diag->ctrl1_xl >> 4) & 0x0F;
    diag->xl_fs_raw = (diag->ctrl1_xl >> 2) & 0x03;
    diag->xl_lpf1_bw_sel = (diag->ctrl1_xl >> 1) & 0x01;
    diag->xl_bw0 = diag->ctrl1_xl & 0x01;
    diag->xl_hm_mode = (diag->ctrl6_c >> 4) & 0x01;
    
    switch (diag->xl_fs_raw) {
        case 0: diag->xl_actual_fs_g = 2.0f; break;
        case 1: diag->xl_actual_fs_g = 16.0f; break;
        case 2: diag->xl_actual_fs_g = 4.0f; break;
        case 3: diag->xl_actual_fs_g = 8.0f; break;
        default: diag->xl_actual_fs_g = 0.0f; break;
    }
    
    diag->is_xl_active = (diag->xl_odr_raw != 0);
    diag->xl_data_ready = (diag->status_reg & LSM6DS3TRC_STATUS_XLDA) != 0;
    
    if (diag->xl_hm_mode == 0) {
        diag->is_xl_high_performance = true;
        diag->xl_power_mode = XL_HIGH_PERFORMANCE;
        diag->xl_power_mode_str = "HIGH-PERFORMANCE";
        
        switch (diag->xl_odr_raw) {
            case 0x01: diag->xl_actual_odr_hz = 12.5f; break;
            case 0x02: diag->xl_actual_odr_hz = 26.0f; break;
            case 0x03: diag->xl_actual_odr_hz = 52.0f; break;
            case 0x04: diag->xl_actual_odr_hz = 104.0f; break;
            case 0x05: diag->xl_actual_odr_hz = 208.0f; break;
            case 0x06: diag->xl_actual_odr_hz = 416.0f; break;
            case 0x07: diag->xl_actual_odr_hz = 833.0f; break;
            case 0x08: diag->xl_actual_odr_hz = 1660.0f; break;
            case 0x09: diag->xl_actual_odr_hz = 3330.0f; break;
            case 0x0A: diag->xl_actual_odr_hz = 6660.0f; break;
            default: diag->xl_actual_odr_hz = 0.0f; 
                     diag->xl_power_mode = XL_MODE_UNKNOWN;
                     diag->xl_power_mode_str = "UNKNOWN";
                     break;
        }
    } else {
        diag->is_xl_high_performance = false;
        
        switch (diag->xl_odr_raw) {
            case 0x01: 
                diag->xl_actual_odr_hz = 12.5f;
                diag->xl_power_mode = XL_LOW_POWER;
                diag->xl_power_mode_str = "LOW-POWER (12.5Hz)";
                break;
            case 0x02:
                diag->xl_actual_odr_hz = 26.0f;
                diag->xl_power_mode = XL_LOW_POWER;
                diag->xl_power_mode_str = "LOW-POWER (26Hz)";
                break;
            case 0x03:
                diag->xl_actual_odr_hz = 52.0f;
                diag->xl_power_mode = XL_LOW_POWER;
                diag->xl_power_mode_str = "LOW-POWER (52Hz)";
                break;
            case 0x04:
                diag->xl_actual_odr_hz = 104.0f;
                diag->xl_power_mode = XL_NORMAL;
                diag->xl_power_mode_str = "NORMAL (104Hz)";
                break;
            case 0x05:
                diag->xl_actual_odr_hz = 208.0f;
                diag->xl_power_mode = XL_NORMAL;
                diag->xl_power_mode_str = "NORMAL (208Hz)";
                break;
            case 0x0B:
                diag->xl_actual_odr_hz = 1.6f;
                diag->xl_power_mode = XL_LOW_POWER;
                diag->xl_power_mode_str = "LOW-POWER (1.6Hz)";
                break;
            default:
                diag->xl_actual_odr_hz = 0.0f;
                diag->xl_power_mode = XL_MODE_UNKNOWN;
                diag->xl_power_mode_str = "UNKNOWN/POWER-DOWN";
                break;
        }
    }
    
    return ESP_OK;
}

// === ПОЛНАЯ ДИАГНОСТИКА ===
esp_err_t lsm6ds3trc_diagnostic_full(lsm_diagnostic_t *diag) {
    if (!diag || !g_initialized) return ESP_ERR_INVALID_STATE;
    
    memset(diag, 0, sizeof(lsm_diagnostic_t));
    
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_WHO_AM_I, &diag->who_am_i, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL1_XL, &diag->ctrl1_xl, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL2_G, &diag->ctrl2_g, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL3_C, &diag->ctrl3_c, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL4_C, &diag->ctrl4_c, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL5_C, &diag->ctrl5_c, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL6_C, &diag->ctrl6_c, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL7_G, &diag->ctrl7_g, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL8_XL, &diag->ctrl8_xl, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL9_XL, &diag->ctrl9_xl, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_CTRL10_C, &diag->ctrl10_c, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_STATUS_REG, &diag->status_reg, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_CTRL1, &diag->fifo_ctrl1, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_CTRL2, &diag->fifo_ctrl2, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_CTRL3, &diag->fifo_ctrl3, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_CTRL4, &diag->fifo_ctrl4, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_CTRL5, &diag->fifo_ctrl5, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_INT1_CTRL, &diag->int1_ctrl, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_INT2_CTRL, &diag->int2_ctrl, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS1, &diag->fifo_status1, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS2, &diag->fifo_status2, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS3, &diag->fifo_status3, 1);
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_FIFO_STATUS4, &diag->fifo_status4, 1);
    
    diag->fifo_words = ((uint32_t)(diag->fifo_status2 & 0x07) << 8) | diag->fifo_status1;
    diag->fifo_samples = diag->fifo_words / 3;
    diag->fifo_pattern = ((uint16_t)(diag->fifo_status4 & 0x03) << 8) | diag->fifo_status3;
    
    lsm6ds3trc_read_accel_raw(&diag->accel_x, &diag->accel_y, &diag->accel_z);
    lsm6ds3trc_read_temp_raw(&diag->temp_raw);
    diag->temp_celsius = lsm6ds3trc_temp_to_celsius(diag->temp_raw);
    
    diag->int1_pin_level = gpio_get_level(LSM6DS3TRC_PIN_INT1);
    diag->sample_interval_us = lsm6ds3trc_get_sample_interval_us(g_current_odr);
    diag->sensitivity_mg_lsb = g_sensitivity_mg_lsb;
    
    lsm6ds3trc_diagnostic_modes(diag);
    
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║           LSM6DS3TR-C ПОЛНАЯ ДИАГНОСТИКА                 ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════╝");
    
    ESP_LOGI(TAG, "\n📋 BASIC INFO:");
    ESP_LOGI(TAG, "  WHO_AM_I:     0x%02X %s", diag->who_am_i, 
             (diag->who_am_i == 0x6A) ? "✅" : "❌");
    ESP_LOGI(TAG, "  INT1 pin:     %d", diag->int1_pin_level);
    ESP_LOGI(TAG, "  Sample int:   %d us", diag->sample_interval_us);
    ESP_LOGI(TAG, "  Sensitivity:  %.3f mg/LSB", diag->sensitivity_mg_lsb);
    
    ESP_LOGI(TAG, "\n🌡️ TEMPERATURE:");
    ESP_LOGI(TAG, "  RAW:      %d", diag->temp_raw);
    ESP_LOGI(TAG, "  Celsius:  %.2f°C", diag->temp_celsius);
    
    ESP_LOGI(TAG, "\n📊 ACCELEROMETER DATA:");
    ESP_LOGI(TAG, "  RAW:      X=%6d Y=%6d Z=%6d", diag->accel_x, diag->accel_y, diag->accel_z);
    ESP_LOGI(TAG, "  (g):      X=%6.3f Y=%6.3f Z=%6.3f",
             diag->accel_x * diag->sensitivity_mg_lsb / 1000.0f,
             diag->accel_y * diag->sensitivity_mg_lsb / 1000.0f,
             diag->accel_z * diag->sensitivity_mg_lsb / 1000.0f);
    ESP_LOGI(TAG, "  DATA_READY:  %s", diag->xl_data_ready ? "✅" : "❌");
    
    ESP_LOGI(TAG, "\n⚙️ CONTROL REGISTERS:");
    ESP_LOGI(TAG, "  CTRL1_XL:     0x%02X (ODR=%d, FS=%d, LPF1=%d, BW0=%d)", 
             diag->ctrl1_xl, diag->xl_odr_raw, diag->xl_fs_raw, 
             diag->xl_lpf1_bw_sel, diag->xl_bw0);
    ESP_LOGI(TAG, "  CTRL2_G:      0x%02X", diag->ctrl2_g);
    ESP_LOGI(TAG, "  CTRL3_C:      0x%02X (BDU=%d, IF_INC=%d)", 
             diag->ctrl3_c, (diag->ctrl3_c >> 6) & 1, (diag->ctrl3_c >> 2) & 1);
    ESP_LOGI(TAG, "  CTRL4_C:      0x%02X", diag->ctrl4_c);
    ESP_LOGI(TAG, "  CTRL5_C:      0x%02X", diag->ctrl5_c);
    ESP_LOGI(TAG, "  CTRL6_C:      0x%02X (XL_HM_MODE=%d)", 
             diag->ctrl6_c, diag->xl_hm_mode);
    ESP_LOGI(TAG, "  CTRL7_G:      0x%02X", diag->ctrl7_g);
    ESP_LOGI(TAG, "  CTRL8_XL:     0x%02X", diag->ctrl8_xl);
    ESP_LOGI(TAG, "  CTRL9_XL:     0x%02X", diag->ctrl9_xl);
    ESP_LOGI(TAG, "  CTRL10_C:     0x%02X", diag->ctrl10_c);
    
    ESP_LOGI(TAG, "\n🎛️ FIFO CONTROL REGISTERS:");
    ESP_LOGI(TAG, "  FIFO_CTRL1:   0x%02X", diag->fifo_ctrl1);
    ESP_LOGI(TAG, "  FIFO_CTRL2:   0x%02X", diag->fifo_ctrl2);
    ESP_LOGI(TAG, "  FIFO_CTRL3:   0x%02X (DEC_FIFO_XL=%d)", 
             diag->fifo_ctrl3, diag->fifo_ctrl3 & 0x07);
    ESP_LOGI(TAG, "  FIFO_CTRL4:   0x%02X (STOP_ON_FTH=%d)", 
             diag->fifo_ctrl4, (diag->fifo_ctrl4 >> 7) & 1);
    ESP_LOGI(TAG, "  FIFO_CTRL5:   0x%02X (ODR=%d, MODE=%d)", 
             diag->fifo_ctrl5, (diag->fifo_ctrl5 >> 3) & 0x0F, diag->fifo_ctrl5 & 0x07);
    
    ESP_LOGI(TAG, "\n📊 FIFO STATUS:");
    ESP_LOGI(TAG, "  Words in FIFO:  %d", diag->fifo_words);
    ESP_LOGI(TAG, "  Samples in FIFO: %d", diag->fifo_samples);
    ESP_LOGI(TAG, "  WATERM: %d, FULL: %d", 
             (diag->fifo_status2 >> 7) & 1,
             (diag->fifo_status2 >> 5) & 1);
    
    return ESP_OK;
}

// === Test data ready in polling mode ===
esp_err_t lsm6ds3trc_test_data_ready(void) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Testing data-ready (polling mode)...");
    
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL5, LSM6DS3TRC_FIFO_MODE_BYPASS);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, 0x00);
    
    for (int i = 0; i < 10; i++) {
        bool ready;
        esp_err_t ret = lsm6ds3trc_is_data_ready(&ready);
        
        if (ret == ESP_OK && ready) {
            int16_t x, y, z;
            lsm6ds3trc_read_accel_raw(&x, &y, &z);
            ESP_LOGI(TAG, "Sample %d: X=%d, Y=%d, Z=%d, INT1=%d", 
                     i, x, y, z, gpio_get_level(LSM6DS3TRC_PIN_INT1));
        } else {
            ESP_LOGI(TAG, "Sample %d: data not ready, INT1=%d", 
                     i, gpio_get_level(LSM6DS3TRC_PIN_INT1));
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ESP_OK;
}

// === Test interrupt mode ===
esp_err_t lsm6ds3trc_test_interrupt(void) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Testing INT1 interrupt with data-ready...");
    
    if (g_fifo_semaphore == NULL) {
        g_fifo_semaphore = xSemaphoreCreateBinary();
        if (!g_fifo_semaphore) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(g_fifo_semaphore, 0);
    
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_FIFO_CTRL5, LSM6DS3TRC_FIFO_MODE_BYPASS);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_DRDY_PULSE_CFG_G, 0x80);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, LSM6DS3TRC_INT1_DRDY_XL);
    
    uint8_t int1_ctrl;
    lsm6ds3trc_spi_read(LSM6DS3TRC_ADDR_INT1_CTRL, &int1_ctrl, 1);
    ESP_LOGI(TAG, "INT1_CTRL: 0x%02X", int1_ctrl);
    
    gpio_intr_enable(LSM6DS3TRC_PIN_INT1);
    
    int interrupts_received = 0;
    for (int i = 0; i < 10; i++) {
        if (xSemaphoreTake(g_fifo_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            interrupts_received++;
            ESP_LOGI(TAG, "✅ Interrupt %d received!", interrupts_received);
            int16_t x, y, z;
            lsm6ds3trc_read_accel_raw(&x, &y, &z);
        }
    }
    
    gpio_intr_disable(LSM6DS3TRC_PIN_INT1);
    lsm6ds3trc_spi_write(LSM6DS3TRC_ADDR_INT1_CTRL, 0x00);
    
    if (interrupts_received > 0) {
        ESP_LOGI(TAG, "✅ Test PASSED: Received %d interrupts", interrupts_received);
    } else {
        ESP_LOGE(TAG, "❌ Test FAILED: No interrupts received!");
    }
    
    return ESP_OK;
}

// === Read register (public wrapper) ===
esp_err_t lsm6ds3trc_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    return lsm6ds3trc_spi_read(reg_addr, data, len);
}

// === Write register (public wrapper) ===
esp_err_t lsm6ds3trc_write_reg(uint8_t reg_addr, uint8_t data) {
    return lsm6ds3trc_spi_write(reg_addr, data);
}

// === Read register burst (public wrapper) ===
esp_err_t lsm6ds3trc_read_reg_burst(uint8_t reg_addr, uint8_t *data, size_t len) {
    return lsm6ds3trc_spi_read_burst(reg_addr, data, len);
}
// === Get current sensitivity ===
float lsm6ds3trc_get_current_sensitivity(void) {
    return g_sensitivity_mg_lsb;
}