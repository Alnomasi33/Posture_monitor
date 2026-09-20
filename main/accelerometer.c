#include "accelerometer.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include <stdint.h>


static const char *TAG = "ACCEL";


/* =========================================================
 * SOFTWARE I2C SETTINGS
 * ========================================================= */

/*
 * Half-period delay.
 *
 * 5 us gives us a relatively slow I2C bus, which is
 * preferable for getting the hackathon prototype working.
 */
#define I2C_DELAY_US 5


/* =========================================================
 * SOFTWARE I2C BUS
 * ========================================================= */

typedef struct
{
    gpio_num_t sda;
    gpio_num_t scl;

} soft_i2c_bus_t;


/*
 * Four independent buses.
 *
 * These GPIO definitions come from accelerometer.h.
 */

static soft_i2c_bus_t buses[4] =
{
    {
        .sda = ACCEL1_SDA,
        .scl = ACCEL1_SCL
    },

    {
        .sda = ACCEL2_SDA,
        .scl = ACCEL2_SCL
    },

    {
        .sda = ACCEL3_SDA,
        .scl = ACCEL3_SCL
    },

    {
        .sda = ACCEL4_SDA,
        .scl = ACCEL4_SCL
    }
};


/* =========================================================
 * DELAY
 * ========================================================= */

static void i2c_delay(void)
{
    esp_rom_delay_us(I2C_DELAY_US);
}


/* =========================================================
 * SDA HELPERS
 *
 * I2C uses open-drain signaling.
 *
 * LOW:
 * drive GPIO low
 *
 * HIGH:
 * release the line and allow pull-up resistor to bring
 * it high.
 * ========================================================= */

static void sda_low(
    soft_i2c_bus_t *bus
)
{
    gpio_set_direction(
        bus->sda,
        GPIO_MODE_OUTPUT_OD
    );

    gpio_set_level(
        bus->sda,
        0
    );
}


static void sda_release(
    soft_i2c_bus_t *bus
)
{
    gpio_set_direction(
        bus->sda,
        GPIO_MODE_INPUT
    );
}


static int sda_read(
    soft_i2c_bus_t *bus
)
{
    gpio_set_direction(
        bus->sda,
        GPIO_MODE_INPUT
    );

    return gpio_get_level(
        bus->sda
    );
}


/* =========================================================
 * SCL HELPERS
 * ========================================================= */

static void scl_low(
    soft_i2c_bus_t *bus
)
{
    gpio_set_direction(
        bus->scl,
        GPIO_MODE_OUTPUT_OD
    );

    gpio_set_level(
        bus->scl,
        0
    );
}


static void scl_release(
    soft_i2c_bus_t *bus
)
{
    gpio_set_direction(
        bus->scl,
        GPIO_MODE_INPUT
    );
}


/* =========================================================
 * I2C START CONDITION
 *
 * SDA goes HIGH -> LOW while SCL is HIGH.
 * ========================================================= */

static void i2c_start(
    soft_i2c_bus_t *bus
)
{
    sda_release(bus);
    scl_release(bus);

    i2c_delay();


    sda_low(bus);

    i2c_delay();


    scl_low(bus);

    i2c_delay();
}


/* =========================================================
 * I2C STOP CONDITION
 *
 * SDA goes LOW -> HIGH while SCL is HIGH.
 * ========================================================= */

static void i2c_stop(
    soft_i2c_bus_t *bus
)
{
    sda_low(bus);

    i2c_delay();


    scl_release(bus);

    i2c_delay();


    sda_release(bus);

    i2c_delay();
}


/* =========================================================
 * WRITE ONE BYTE
 *
 * Returns true if the device ACKs.
 * ========================================================= */

static bool i2c_write_byte(
    soft_i2c_bus_t *bus,
    uint8_t value
)
{

    /*
     * Send bits MSB first.
     */

    for (int bit = 7; bit >= 0; bit--)
    {

        if (
            value &
            (1 << bit)
        )
        {
            sda_release(bus);
        }
        else
        {
            sda_low(bus);
        }


        i2c_delay();


        scl_release(bus);

        i2c_delay();


        scl_low(bus);

        i2c_delay();
    }


    /* -----------------------------------------------------
     * ACK BIT
     * ----------------------------------------------------- */

    sda_release(bus);

    i2c_delay();


    scl_release(bus);

    i2c_delay();


    /*
     * Device pulls SDA LOW for ACK.
     */

    bool ack =
        (sda_read(bus) == 0);


    scl_low(bus);

    i2c_delay();


    return ack;
}


/* =========================================================
 * READ ONE BYTE
 *
 * send_ack:
 *
 * true  = send ACK after byte
 * false = send NACK after byte
 * ========================================================= */

static uint8_t i2c_read_byte(
    soft_i2c_bus_t *bus,
    bool send_ack
)
{

    uint8_t value = 0;


    sda_release(bus);


    for (int bit = 7; bit >= 0; bit--)
    {

        scl_release(bus);

        i2c_delay();


        if (sda_read(bus))
        {
            value |=
                (1 << bit);
        }


        scl_low(bus);

        i2c_delay();
    }


    /* -----------------------------------------------------
     * ACK / NACK
     * ----------------------------------------------------- */

    if (send_ack)
    {
        sda_low(bus);
    }
    else
    {
        sda_release(bus);
    }


    i2c_delay();


    scl_release(bus);

    i2c_delay();


    scl_low(bus);

    i2c_delay();


    sda_release(bus);


    return value;
}


/* =========================================================
 * WRITE MMA7660 REGISTER
 * ========================================================= */

static esp_err_t mma7660_write_register(
    soft_i2c_bus_t *bus,
    uint8_t reg,
    uint8_t value
)
{

    i2c_start(bus);


    /*
     * Address + WRITE bit
     */

    if (
        !i2c_write_byte(
            bus,
            (MMA7660_ADDR << 1) | 0
        )
    )
    {
        i2c_stop(bus);

        return ESP_FAIL;
    }


    /*
     * Register address
     */

    if (
        !i2c_write_byte(
            bus,
            reg
        )
    )
    {
        i2c_stop(bus);

        return ESP_FAIL;
    }


    /*
     * Register value
     */

    if (
        !i2c_write_byte(
            bus,
            value
        )
    )
    {
        i2c_stop(bus);

        return ESP_FAIL;
    }


    i2c_stop(bus);


    return ESP_OK;
}


/* =========================================================
 * READ MMA7660 REGISTER
 * ========================================================= */

static esp_err_t mma7660_read_register(
    soft_i2c_bus_t *bus,
    uint8_t reg,
    uint8_t *value
)
{

    /* -----------------------------------------------------
     * Tell sensor which register we want.
     * ----------------------------------------------------- */

    i2c_start(bus);


    if (
        !i2c_write_byte(
            bus,
            (MMA7660_ADDR << 1) | 0
        )
    )
    {
        i2c_stop(bus);

        return ESP_FAIL;
    }


    if (
        !i2c_write_byte(
            bus,
            reg
        )
    )
    {
        i2c_stop(bus);

        return ESP_FAIL;
    }


    /* -----------------------------------------------------
     * Repeated START
     * ----------------------------------------------------- */

    i2c_start(bus);


    /*
     * Address + READ bit
     */

    if (
        !i2c_write_byte(
            bus,
            (MMA7660_ADDR << 1) | 1
        )
    )
    {
        i2c_stop(bus);

        return ESP_FAIL;
    }


    /*
     * Read one byte.
     *
     * false = NACK because this is the final byte.
     */

    *value =
        i2c_read_byte(
            bus,
            false
        );


    i2c_stop(bus);


    return ESP_OK;
}


/* =========================================================
 * CONVERT MMA7660 6-BIT VALUE
 *
 * MMA7660 output is signed 6-bit two's complement.
 *
 * Range:
 *
 * -32 ... +31
 * ========================================================= */

static int8_t convert_6bit(
    uint8_t value
)
{

    /*
     * Bits 0-5 contain acceleration.
     */

    value &= 0x3F;


    /*
     * Bit 5 is sign bit.
     */

    if (
        value &
        0x20
    )
    {

        value |= 0xC0;
    }


    return (int8_t)value;
}


/* =========================================================
 * INITIALIZE ONE SOFTWARE I2C BUS
 * ========================================================= */

static esp_err_t init_bus(
    soft_i2c_bus_t *bus
)
{

    /*
     * Configure SDA.
     */

    gpio_config_t sda_config =
    {
        .pin_bit_mask =
            (1ULL << bus->sda),

        .mode =
            GPIO_MODE_INPUT_OUTPUT_OD,

        .pull_up_en =
            GPIO_PULLUP_ENABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    esp_err_t ret =
        gpio_config(
            &sda_config
        );


    if (ret != ESP_OK)
    {
        return ret;
    }


    /*
     * Configure SCL.
     */

    gpio_config_t scl_config =
    {
        .pin_bit_mask =
            (1ULL << bus->scl),

        .mode =
            GPIO_MODE_INPUT_OUTPUT_OD,

        .pull_up_en =
            GPIO_PULLUP_ENABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ret =
        gpio_config(
            &scl_config
        );


    if (ret != ESP_OK)
    {
        return ret;
    }


    /*
     * Release both lines.
     */

    gpio_set_level(
        bus->sda,
        1
    );

    gpio_set_level(
        bus->scl,
        1
    );


    sda_release(bus);

    scl_release(bus);


    i2c_delay();


    return ESP_OK;
}


/* =========================================================
 * INITIALIZE ONE MMA7660
 * ========================================================= */

static esp_err_t init_sensor(
    soft_i2c_bus_t *bus
)
{

    esp_err_t ret;


    /*
     * MMA7660 must be placed into standby mode
     * before changing configuration.
     */

    ret =
        mma7660_write_register(
            bus,
            MMA7660_REG_MODE,
            0x00
        );


    if (ret != ESP_OK)
    {
        return ret;
    }


    /*
     * Set sample rate.
     *
     * 0x00 is sufficient for our posture MVP.
     */

    ret =
        mma7660_write_register(
            bus,
            MMA7660_REG_SR,
            0x00
        );


    if (ret != ESP_OK)
    {
        return ret;
    }


    /*
     * Activate sensor.
     *
     * MODE bit 0 = 1
     */

    ret =
        mma7660_write_register(
            bus,
            MMA7660_REG_MODE,
            0x01
        );


    if (ret != ESP_OK)
    {
        return ret;
    }


    /*
     * Give accelerometer time to start.
     */

    esp_rom_delay_us(
        10000
    );


    return ESP_OK;
}


/* =========================================================
 * INITIALIZE ALL FOUR ACCELEROMETERS
 * ========================================================= */

esp_err_t accelerometers_init(void)
{

    ESP_LOGI(
        TAG,
        "Initializing four MMA7660 accelerometers"
    );


    for (int i = 0; i < 4; i++)
    {

        ESP_LOGI(
            TAG,
            "Initializing ACC%d...",
            i + 1
        );


        esp_err_t ret =
            init_bus(
                &buses[i]
            );


        if (ret != ESP_OK)
        {

            ESP_LOGE(
                TAG,
                "ACC%d bus initialization failed",
                i + 1
            );


            return ret;
        }


        ret =
            init_sensor(
                &buses[i]
            );


        if (ret != ESP_OK)
        {

            ESP_LOGE(
                TAG,
                "ACC%d did not respond at address 0x%02X",
                i + 1,
                MMA7660_ADDR
            );


            return ret;
        }


        ESP_LOGI(
            TAG,
            "ACC%d initialized successfully",
            i + 1
        );
    }


    ESP_LOGI(
        TAG,
        "All four accelerometers initialized"
    );


    return ESP_OK;
}


/* =========================================================
 * READ ONE ACCELEROMETER
 * ========================================================= */

esp_err_t accelerometer_read(
    accelerometer_id_t sensor,
    accelerometer_data_t *data
)
{

    /*
     * Validate arguments.
     */

    if (
        sensor < ACCEL_SENSOR_1 ||
        sensor > ACCEL_SENSOR_4 ||
        data == NULL
    )
    {
        return ESP_ERR_INVALID_ARG;
    }


    soft_i2c_bus_t *bus =
        &buses[sensor];


    uint8_t raw_x;

    uint8_t raw_y;

    uint8_t raw_z;


    /*
     * MMA7660 sets bit 6 when the measurement
     * is invalid because an update occurred while
     * the register was being read.
     *
     * Retry several times.
     */

    for (int attempt = 0; attempt < 10; attempt++)
    {

        esp_err_t ret;


        ret =
            mma7660_read_register(
                bus,
                MMA7660_REG_XOUT,
                &raw_x
            );


        if (ret != ESP_OK)
        {
            return ret;
        }


        ret =
            mma7660_read_register(
                bus,
                MMA7660_REG_YOUT,
                &raw_y
            );


        if (ret != ESP_OK)
        {
            return ret;
        }


        ret =
            mma7660_read_register(
                bus,
                MMA7660_REG_ZOUT,
                &raw_z
            );


        if (ret != ESP_OK)
        {
            return ret;
        }


        /*
         * Bit 6 is ALERT.
         *
         * If it is set, retry.
         */

        if (
            !(raw_x & 0x40) &&
            !(raw_y & 0x40) &&
            !(raw_z & 0x40)
        )
        {

            int8_t x =
                convert_6bit(
                    raw_x
                );


            int8_t y =
                convert_6bit(
                    raw_y
                );


            int8_t z =
                convert_6bit(
                    raw_z
                );


            /*
             * MMA7660 sensitivity is approximately
             * 21.33 counts per g.
             */

            data->x =
                (float)x /
                21.33f;


            data->y =
                (float)y /
                21.33f;


            data->z =
                (float)z /
                21.33f;


            return ESP_OK;
        }


        /*
         * Small delay before retry.
         */

        esp_rom_delay_us(
            500
        );
    }


    ESP_LOGW(
        TAG,
        "ACC%d could not get stable measurement",
        sensor + 1
    );


    return ESP_FAIL;
}