#ifndef ACCEL_TYPES_H
#define ACCEL_TYPES_H

typedef enum {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G = 1,
    ACCEL_RANGE_8G = 2,
    ACCEL_RANGE_16G = 3
} accel_range_t;

typedef enum {
    ACCEL_ODR_POWER_DOWN = 0,
    ACCEL_ODR_12_5HZ = 1,
    ACCEL_ODR_26HZ = 2,
    ACCEL_ODR_52HZ = 3,
    ACCEL_ODR_104HZ = 4,
    ACCEL_ODR_208HZ = 5,
    ACCEL_ODR_416HZ = 6,
    ACCEL_ODR_833HZ = 7,
    ACCEL_ODR_1660HZ = 8,
    ACCEL_ODR_3330HZ = 9,
    ACCEL_ODR_6660HZ = 10
} accel_odr_t;

static inline float accel_odr_to_hz(accel_odr_t odr)
{
    switch (odr) {
    case ACCEL_ODR_12_5HZ: return 12.5f;
    case ACCEL_ODR_26HZ: return 26.0f;
    case ACCEL_ODR_52HZ: return 52.0f;
    case ACCEL_ODR_104HZ: return 104.0f;
    case ACCEL_ODR_208HZ: return 208.0f;
    case ACCEL_ODR_416HZ: return 416.0f;
    case ACCEL_ODR_833HZ: return 833.0f;
    case ACCEL_ODR_1660HZ: return 1660.0f;
    case ACCEL_ODR_3330HZ: return 3330.0f;
    case ACCEL_ODR_6660HZ: return 6660.0f;
    default: return 3330.0f;
    }
}

#endif
