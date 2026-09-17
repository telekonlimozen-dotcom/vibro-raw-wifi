#ifndef LSM6DS3TRC_H
#define LSM6DS3TRC_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "accel_types.h"
#include <stdint.h>
#include <stdbool.h>

// === SPI Configuration ===
#define LSM6DS3TRC_SPI_HOST   SPI2_HOST
#define LSM6DS3TRC_PIN_CS     GPIO_NUM_5
#define LSM6DS3TRC_PIN_SCK    GPIO_NUM_6
#define LSM6DS3TRC_PIN_MISO   GPIO_NUM_11
#define LSM6DS3TRC_PIN_MOSI   GPIO_NUM_7
#define LSM6DS3TRC_PIN_INT1   GPIO_NUM_20

// === Register Addresses ===
#define LSM6DS3TRC_ADDR_FUNC_CFG_ACCESS   0x01
#define LSM6DS3TRC_ADDR_FIFO_CTRL1        0x06
#define LSM6DS3TRC_ADDR_FIFO_CTRL2        0x07
#define LSM6DS3TRC_ADDR_FIFO_CTRL3        0x08
#define LSM6DS3TRC_ADDR_FIFO_CTRL4        0x09
#define LSM6DS3TRC_ADDR_FIFO_CTRL5        0x0A
#define LSM6DS3TRC_ADDR_DRDY_PULSE_CFG_G  0x0B
#define LSM6DS3TRC_ADDR_INT1_CTRL         0x0D
#define LSM6DS3TRC_ADDR_INT2_CTRL         0x0E
#define LSM6DS3TRC_ADDR_WHO_AM_I          0x0F
#define LSM6DS3TRC_ADDR_CTRL1_XL          0x10
#define LSM6DS3TRC_ADDR_CTRL2_G           0x11
#define LSM6DS3TRC_ADDR_CTRL3_C           0x12
#define LSM6DS3TRC_ADDR_CTRL4_C           0x13
#define LSM6DS3TRC_ADDR_CTRL5_C           0x14
#define LSM6DS3TRC_ADDR_CTRL6_C           0x15
#define LSM6DS3TRC_ADDR_CTRL7_G           0x16
#define LSM6DS3TRC_ADDR_CTRL8_XL          0x17
#define LSM6DS3TRC_ADDR_CTRL9_XL          0x18
#define LSM6DS3TRC_ADDR_CTRL10_C          0x19
#define LSM6DS3TRC_ADDR_STATUS_REG        0x1E
#define LSM6DS3TRC_ADDR_OUT_TEMP_L        0x20
#define LSM6DS3TRC_ADDR_OUT_TEMP_H        0x21
#define LSM6DS3TRC_ADDR_OUTX_L_G          0x22
#define LSM6DS3TRC_ADDR_OUTX_H_G          0x23
#define LSM6DS3TRC_ADDR_OUTY_L_G          0x24
#define LSM6DS3TRC_ADDR_OUTY_H_G          0x25
#define LSM6DS3TRC_ADDR_OUTZ_L_G          0x26
#define LSM6DS3TRC_ADDR_OUTZ_H_G          0x27
#define LSM6DS3TRC_ADDR_OUTX_L_XL         0x28
#define LSM6DS3TRC_ADDR_OUTX_H_XL         0x29
#define LSM6DS3TRC_ADDR_OUTY_L_XL         0x2A
#define LSM6DS3TRC_ADDR_OUTY_H_XL         0x2B
#define LSM6DS3TRC_ADDR_OUTZ_L_XL         0x2C
#define LSM6DS3TRC_ADDR_OUTZ_H_XL         0x2D

// === FIFO Status Registers ===
#define LSM6DS3TRC_ADDR_FIFO_STATUS1      0x3A
#define LSM6DS3TRC_ADDR_FIFO_STATUS2      0x3B
#define LSM6DS3TRC_ADDR_FIFO_STATUS3      0x3C
#define LSM6DS3TRC_ADDR_FIFO_STATUS4      0x3D
#define LSM6DS3TRC_ADDR_FIFO_DATA_OUT_L   0x3E
#define LSM6DS3TRC_ADDR_FIFO_DATA_OUT_H   0x3F
#define LSM6DS3TRC_ADDR_TIMESTAMP0        0x40
#define LSM6DS3TRC_ADDR_TIMESTAMP1        0x41
#define LSM6DS3TRC_ADDR_TIMESTAMP2        0x42
#define LSM6DS3TRC_ADDR_WAKE_UP_DUR       0x5C

/** LSM6DS3TR-C TIMESTAMP tick with TIMER_HR=1 (AN5130). */
#define LSM6DS3TRC_TIMESTAMP_LSB_US       25.0f

// === WHO_AM_I Value ===
#define LSM6DS3TRC_WHO_AM_I_VALUE          0x6A

// === CTRL3_C Bits ===
#define LSM6DS3TRC_CTRL3_SW_RESET          (1 << 0)
#define LSM6DS3TRC_CTRL3_IF_INC            (1 << 2)
#define LSM6DS3TRC_CTRL3_BDU               (1 << 6)

// === FIFO Modes ===
#define LSM6DS3TRC_FIFO_MODE_BYPASS        (0x00)
#define LSM6DS3TRC_FIFO_MODE_FIFO          (0x01)
#define LSM6DS3TRC_FIFO_MODE_CONTINUOUS    (0x06)

// === FIFO ODR Values ===
#define LSM6DS3TRC_FIFO_ODR_12_5HZ         (0x01 << 3)
#define LSM6DS3TRC_FIFO_ODR_26HZ           (0x02 << 3)
#define LSM6DS3TRC_FIFO_ODR_52HZ           (0x03 << 3)
#define LSM6DS3TRC_FIFO_ODR_104HZ          (0x04 << 3)
#define LSM6DS3TRC_FIFO_ODR_208HZ          (0x05 << 3)
#define LSM6DS3TRC_FIFO_ODR_416HZ          (0x06 << 3)
#define LSM6DS3TRC_FIFO_ODR_833HZ          (0x07 << 3)
#define LSM6DS3TRC_FIFO_ODR_1660HZ         (0x08 << 3)
#define LSM6DS3TRC_FIFO_ODR_3330HZ         (0x09 << 3)
#define LSM6DS3TRC_FIFO_ODR_6660HZ         (0x0A << 3)

// === INT1 Interrupt Bits ===
#define LSM6DS3TRC_INT1_DRDY_XL            (1 << 0)
#define LSM6DS3TRC_INT1_DRDY_G             (1 << 1)
#define LSM6DS3TRC_INT1_BOOT               (1 << 2)
#define LSM6DS3TRC_INT1_FIFO_THS           (1 << 3)
#define LSM6DS3TRC_INT1_FIFO_OVR           (1 << 4)
#define LSM6DS3TRC_INT1_FULL_FLAG          (1 << 5)
#define LSM6DS3TRC_INT1_SIGN_MOT           (1 << 6)
#define LSM6DS3TRC_INT1_STEP_DET           (1 << 7)

// === FIFO_STATUS2 Bits ===
#define LSM6DS3TRC_FIFO_STATUS2_WATERM     (1 << 7)
#define LSM6DS3TRC_FIFO_STATUS2_OVERRUN    (1 << 6)
#define LSM6DS3TRC_FIFO_STATUS2_FULL_SMART (1 << 5)
#define LSM6DS3TRC_FIFO_STATUS2_EMPTY      (1 << 4)

// === STATUS_REG Bits ===
#define LSM6DS3TRC_STATUS_XLDA             (1 << 0)
#define LSM6DS3TRC_STATUS_GDA              (1 << 1)
#define LSM6DS3TRC_STATUS_TDA              (1 << 2)

// === Sensitivity values ===
#define LSM6DS3TRC_SENSITIVITY_FS_2G       0.061f
#define LSM6DS3TRC_SENSITIVITY_FS_4G       0.122f
#define LSM6DS3TRC_SENSITIVITY_FS_8G       0.244f
#define LSM6DS3TRC_SENSITIVITY_FS_16G      0.488f

// === Ð ÐµÐ¶Ð¸Ð¼Ñ‹ Ð¿Ð¸Ñ‚Ð°Ð½Ð¸Ñ Ð°ÐºÑÐµÐ»ÐµÑ€Ð¾Ð¼ÐµÑ‚Ñ€Ð° ===
typedef enum {
    XL_POWER_DOWN = 0,
    XL_HIGH_PERFORMANCE = 1,
    XL_LOW_POWER = 2,
    XL_NORMAL = 3,
    XL_MODE_UNKNOWN = 4
} xl_power_mode_t;

// === Sample structure ===
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t timestamp_us;
} __attribute__((packed)) lsm_sample_t;

// === Callback type ===
typedef void (*lsm_fifo_callback_t)(void);

// === ÐŸÐžÐ›ÐÐÐ¯ Ð”Ð˜ÐÐ“ÐÐžÐ¡Ð¢Ð˜Ð§Ð•Ð¡ÐšÐÐ¯ Ð¡Ð¢Ð Ð£ÐšÐ¢Ð£Ð Ð ===
typedef struct {
    // Basic
    uint8_t who_am_i;
    uint8_t status_reg;
    
    // Control registers
    uint8_t ctrl1_xl;
    uint8_t ctrl2_g;
    uint8_t ctrl3_c;
    uint8_t ctrl4_c;
    uint8_t ctrl5_c;
    uint8_t ctrl6_c;
    uint8_t ctrl7_g;
    uint8_t ctrl8_xl;
    uint8_t ctrl9_xl;
    uint8_t ctrl10_c;
    
    // FIFO control
    uint8_t fifo_ctrl1;
    uint8_t fifo_ctrl2;
    uint8_t fifo_ctrl3;
    uint8_t fifo_ctrl4;
    uint8_t fifo_ctrl5;
    
    // Interrupt control
    uint8_t int1_ctrl;
    uint8_t int2_ctrl;
    
    // FIFO status registers
    uint8_t fifo_status1;
    uint8_t fifo_status2;
    uint8_t fifo_status3;
    uint8_t fifo_status4;
    
    // Computed values
    uint32_t fifo_words;
    uint32_t fifo_samples;
    uint16_t fifo_pattern;
    
    // Current accelerometer data
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    
    // Other info
    int int1_pin_level;
    uint32_t sample_interval_us;
    float sensitivity_mg_lsb;
    
    // Temperature
    int16_t temp_raw;
    float temp_celsius;
    
    // Ð”Ð¸Ð°Ð³Ð½Ð¾ÑÑ‚Ð¸ÐºÐ° Ñ€ÐµÐ¶Ð¸Ð¼Ð¾Ð² Ð°ÐºÑÐµÐ»ÐµÑ€Ð¾Ð¼ÐµÑ‚Ñ€Ð°
    uint8_t xl_odr_raw;
    uint8_t xl_fs_raw;
    uint8_t xl_hm_mode;
    uint8_t xl_lpf1_bw_sel;
    uint8_t xl_bw0;
    bool xl_data_ready;
    
    float xl_actual_odr_hz;
    float xl_actual_fs_g;
    xl_power_mode_t xl_power_mode;
    const char* xl_power_mode_str;
    bool is_xl_active;
    bool is_xl_high_performance;
} lsm_diagnostic_t;

// === Public API ===
esp_err_t lsm6ds3trc_init(accel_range_t accel_range,
                          accel_odr_t accel_odr,
                          float *actual_sensitivity_mg_lsb);

/** True after successful init until deinit. */
bool lsm6ds3trc_is_initialized(void);

esp_err_t lsm6ds3trc_deinit(void);
/** XL ODR off, keep SPI + ISR (FRF SoftAP loop â€” avoid SPI re-add desync). */
esp_err_t lsm6ds3trc_soft_sleep(void);
/** Restore XL ODR after soft_sleep (same FS/ODR as last init). */
esp_err_t lsm6ds3trc_soft_wake(void);

// === FIFO functions ===
esp_err_t lsm6ds3trc_configure_fifo(uint16_t watermark_level);
esp_err_t lsm6ds3trc_start_fifo_collection(lsm_sample_t *buffer, size_t max_samples);
esp_err_t lsm6ds3trc_stop_fifo_collection(void);
esp_err_t lsm6ds3trc_read_fifo_samples(lsm_sample_t *buffer, size_t *num_samples, size_t max_samples);
uint32_t lsm6ds3trc_get_fifo_words(void);
uint32_t lsm6ds3trc_get_fifo_samples(void);
uint16_t lsm6ds3trc_get_fifo_pattern(void);
uint8_t lsm6ds3trc_get_fifo_status(void);
bool lsm6ds3trc_is_watermark_reached(void);
bool lsm6ds3trc_is_fifo_full(void);
esp_err_t lsm6ds3trc_clear_fifo(void);
esp_err_t lsm6ds3trc_set_fifo_callback(lsm_fifo_callback_t callback);

/** Enable 24-bit sensor timestamp @ 25 Âµs/LSB (CTRL10 TIMER_EN). */
esp_err_t lsm6ds3trc_timestamp_enable(void);
/** Read TIMESTAMP0..2 as 24-bit tick count. */
esp_err_t lsm6ds3trc_read_timestamp_ticks(uint32_t *ticks);

// === Ð”Ð¸Ð°Ð³Ð½Ð¾ÑÑ‚Ð¸ÐºÐ° Ð¸ Ñ‚ÐµÑÑ‚Ð¸Ñ€Ð¾Ð²Ð°Ð½Ð¸Ðµ ===
esp_err_t lsm6ds3trc_diagnostic_full(lsm_diagnostic_t *diag);
esp_err_t lsm6ds3trc_diagnostic_modes(lsm_diagnostic_t *diag);
esp_err_t lsm6ds3trc_test_data_ready(void);
esp_err_t lsm6ds3trc_test_interrupt(void);
esp_err_t lsm6ds3trc_print_registers(void);

// === Direct read ===
esp_err_t lsm6ds3trc_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az);
esp_err_t lsm6ds3trc_read_accel_g(float *ax_g, float *ay_g, float *az_g, float sensitivity_mg_lsb);
esp_err_t lsm6ds3trc_is_data_ready(bool *ready);
esp_err_t lsm6ds3trc_read_accel_raw_wait(int16_t *ax, int16_t *ay, int16_t *az, uint32_t timeout_ms);
bool lsm6ds3trc_check_data_ready_pin(void);
float lsm6ds3trc_get_current_sensitivity(void);
// === Temperature read ===
esp_err_t lsm6ds3trc_read_temp_raw(int16_t *temp);
float lsm6ds3trc_temp_to_celsius(int16_t raw);

// === Register access ===
esp_err_t lsm6ds3trc_read_reg(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t lsm6ds3trc_write_reg(uint8_t reg_addr, uint8_t data);
esp_err_t lsm6ds3trc_read_reg_burst(uint8_t reg_addr, uint8_t *data, size_t len);

// === External semaphore ===
extern SemaphoreHandle_t g_fifo_semaphore;

#endif // LSM6DS3TRC_H
