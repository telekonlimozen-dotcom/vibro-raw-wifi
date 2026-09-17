#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "driver/gpio.h"

#define BOARD_LED_GPIO              GPIO_NUM_19
#define BOARD_LED_ACTIVE_LOW        0

#define BOARD_LED_BLINK_COUNT       3
#define BOARD_LED_BLINK_ON_MS       200
#define BOARD_LED_BLINK_OFF_MS      200
#define BOARD_LED_CYCLE_MS          10000

#define BOARD_RTC_I2C_PORT          I2C_NUM_0
#define BOARD_RTC_SDA_GPIO          GPIO_NUM_22
#define BOARD_RTC_SCL_GPIO          GPIO_NUM_21
#define BOARD_RTC_INT_GPIO          GPIO_NUM_15
#define BOARD_RTC_I2C_FREQ_HZ       100000

#define DEFAULT_SERVICE_LORA_FREQ   433000000UL
#define SERVICE_LISTEN_MS             5000
#define SERVICE_LISTEN_UNSYNCED_MS   30000
/** On-demand RTC: listen service channel after TX time request. */
#define TIME_REQUEST_LISTEN_MS       (60 * 1000)
/** Shorter listen for periodic re-sync when already "synced" but possibly skewed. */
#define TIME_REQUEST_RESYNC_LISTEN_MS (20 * 1000)
#define LORA_BASE_ADDRESS            0x01

/** Daily RTC sync: base TX and sensor listen ±15 min around 00:00 UTC, every 30 s. */
#define RTC_DAILY_SYNC_WINDOW_SEC     (15 * 60)
#define RTC_DAILY_SYNC_INTERVAL_SEC   30
/** Re-request time from base at least this often (sensor clock). */
#define RTC_PERIODIC_RESYNC_SEC       (30 * 60)

/** After data TX: listen service channel for TX_POWER / INTERVAL / RTC_SYNC. */
#define POST_TX_CMD_LISTEN_MS        3500

/** 1 = не уходить в deep sleep после цикла (отладка LED/USB). */
#define SENSOR_DEBUG_NO_DEEP_SLEEP    0

/** Проснуться за столько мс до LoRa-слота: измерение (~1 с) + запас. */
#define SENSOR_MEASURE_LEAD_MS        8200

#endif
