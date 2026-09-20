#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"


/* =========================================================
 * MMA7660FC
 * ========================================================= */

#define MMA7660_ADDR 0x4C


/* =========================================================
 * MMA7660 REGISTERS
 * ========================================================= */

#define MMA7660_REG_XOUT   0x00
#define MMA7660_REG_YOUT   0x01
#define MMA7660_REG_ZOUT   0x02
#define MMA7660_REG_TILT   0x03
#define MMA7660_REG_SRST   0x04
#define MMA7660_REG_SPCNT  0x05
#define MMA7660_REG_INTSU  0x06
#define MMA7660_REG_MODE   0x07
#define MMA7660_REG_SR     0x08
#define MMA7660_REG_PDET   0x09
#define MMA7660_REG_PD     0x0A


/* =========================================================
 * SENSOR 1
 *
 * LEFT SHOULDER
 * ========================================================= */

#define ACCEL1_SDA GPIO_NUM_32
#define ACCEL1_SCL GPIO_NUM_33


/* =========================================================
 * SENSOR 2
 *
 * RIGHT SHOULDER
 * ========================================================= */

#define ACCEL2_SDA GPIO_NUM_25
#define ACCEL2_SCL GPIO_NUM_26


/* =========================================================
 * SENSOR 3
 *
 * LEFT UPPER BACK
 * ========================================================= */

#define ACCEL3_SDA GPIO_NUM_27
#define ACCEL3_SCL GPIO_NUM_14


/* =========================================================
 * SENSOR 4
 *
 * RIGHT UPPER BACK
 * ========================================================= */

#define ACCEL4_SDA GPIO_NUM_16
#define ACCEL4_SCL GPIO_NUM_17


/* =========================================================
 * SENSOR IDs
 * ========================================================= */

typedef enum
{

    ACCEL_SENSOR_1 = 0,

    ACCEL_SENSOR_2,

    ACCEL_SENSOR_3,

    ACCEL_SENSOR_4

} accelerometer_id_t;


/* =========================================================
 * ACCELEROMETER DATA
 * ========================================================= */

typedef struct
{

    float x;

    float y;

    float z;

} accelerometer_data_t;


/* =========================================================
 * FUNCTIONS
 * ========================================================= */


/*
 * Initialize all four software-I2C buses
 * and all four MMA7660 accelerometers.
 */

esp_err_t accelerometers_init(void);


/*
 * Read one accelerometer.
 */

esp_err_t accelerometer_read(
    accelerometer_id_t sensor,
    accelerometer_data_t *data
);