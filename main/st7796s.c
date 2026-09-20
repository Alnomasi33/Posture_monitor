#include "st7796s.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include <string.h>


static const char *TAG = "ST7796S";


/* =========================================================
 * 5x7 FONT
 *
 * Characters supported:
 * space
 * A-Z
 * 0-9
 * -
 * !
 * :
 * .
 *
 * Each character contains 5 columns.
 * ========================================================= */

typedef struct
{
    char character;
    uint8_t data[5];

} font_character_t;


static const font_character_t font[] =
{
    {' ', {0x00,0x00,0x00,0x00,0x00}},

    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'J', {0x20,0x40,0x41,0x3F,0x01}},
    {'K', {0x7F,0x08,0x14,0x22,0x41}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'Q', {0x3E,0x41,0x51,0x21,0x5E}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}},
    {'V', {0x1F,0x20,0x40,0x20,0x1F}},
    {'W', {0x3F,0x40,0x38,0x40,0x3F}},
    {'X', {0x63,0x14,0x08,0x14,0x63}},
    {'Y', {0x07,0x08,0x70,0x08,0x07}},
    {'Z', {0x61,0x51,0x49,0x45,0x43}},

    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x42,0x61,0x51,0x49,0x46}},
    {'3', {0x21,0x41,0x45,0x4B,0x31}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},

    {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'!', {0x00,0x00,0x5F,0x00,0x00}},
    {':', {0x00,0x36,0x36,0x00,0x00}},
    {'.', {0x00,0x60,0x60,0x00,0x00}}
};


#define FONT_COUNT \
    (sizeof(font) / sizeof(font[0]))


/* =========================================================
 * SPI WRITE
 * ========================================================= */

static esp_err_t lcd_spi_write(
    st7796s_t *lcd,
    const uint8_t *data,
    size_t length
)
{
    if (length == 0)
    {
        return ESP_OK;
    }


    spi_transaction_t transaction = {0};

    transaction.length =
        length * 8;

    transaction.tx_buffer =
        data;


    return spi_device_transmit(
        lcd->spi,
        &transaction
    );
}


/* =========================================================
 * SEND COMMAND
 * ========================================================= */

static esp_err_t lcd_command(
    st7796s_t *lcd,
    uint8_t command
)
{
    gpio_set_level(
        lcd->dc_pin,
        0
    );


    return lcd_spi_write(
        lcd,
        &command,
        1
    );
}


/* =========================================================
 * SEND DATA
 * ========================================================= */

static esp_err_t lcd_data(
    st7796s_t *lcd,
    const uint8_t *data,
    size_t length
)
{
    gpio_set_level(
        lcd->dc_pin,
        1
    );


    return lcd_spi_write(
        lcd,
        data,
        length
    );
}


/* =========================================================
 * SET DRAWING WINDOW
 * ========================================================= */

static esp_err_t st7796s_set_window(
    st7796s_t *lcd,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1
)
{
    uint8_t data[4];

    esp_err_t ret;


    /* Column */

    ret = lcd_command(
        lcd,
        ST7796S_CASET
    );

    if (ret != ESP_OK)
        return ret;


    data[0] = x0 >> 8;
    data[1] = x0 & 0xFF;
    data[2] = x1 >> 8;
    data[3] = x1 & 0xFF;


    ret = lcd_data(
        lcd,
        data,
        4
    );

    if (ret != ESP_OK)
        return ret;


    /* Row */

    ret = lcd_command(
        lcd,
        ST7796S_PASET
    );

    if (ret != ESP_OK)
        return ret;


    data[0] = y0 >> 8;
    data[1] = y0 & 0xFF;
    data[2] = y1 >> 8;
    data[3] = y1 & 0xFF;


    ret = lcd_data(
        lcd,
        data,
        4
    );

    if (ret != ESP_OK)
        return ret;


    return lcd_command(
        lcd,
        ST7796S_RAMWR
    );
}


/* =========================================================
 * BACKLIGHT
 * ========================================================= */

void st7796s_backlight(
    st7796s_t *lcd,
    bool on
)
{
    gpio_set_level(
        lcd->bl_pin,
        on ? 1 : 0
    );
}


/* =========================================================
 * INITIALIZE LCD
 * ========================================================= */

esp_err_t st7796s_init(
    st7796s_t *lcd
)
{
    ESP_LOGI(
        TAG,
        "Initializing ST7796S"
    );


    /* -----------------------------------------------------
     * GPIO configuration
     * ----------------------------------------------------- */

    gpio_config_t output_config =
    {
        .pin_bit_mask =
            (1ULL << lcd->dc_pin) |
            (1ULL << lcd->rst_pin) |
            (1ULL << lcd->bl_pin),

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &output_config
        )
    );


    /* Backlight off during initialization */

    gpio_set_level(
        lcd->bl_pin,
        0
    );


    /* -----------------------------------------------------
     * SPI BUS
     * ----------------------------------------------------- */

    spi_bus_config_t bus_config =
    {
        .mosi_io_num = LCD_MOSI,

        .miso_io_num = -1,

        .sclk_io_num = LCD_SCLK,

        .quadwp_io_num = -1,

        .quadhd_io_num = -1,

        .max_transfer_sz =
            ST7796S_WIDTH *
            ST7796S_HEIGHT *
            2
    };


    esp_err_t ret =
        spi_bus_initialize(
            SPI2_HOST,
            &bus_config,
            SPI_DMA_CH_AUTO
        );


    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    )
    {
        ESP_LOGE(
            TAG,
            "SPI bus initialization failed"
        );

        return ret;
    }


    /* -----------------------------------------------------
     * LCD SPI DEVICE
     *
     * 10 MHz is deliberately conservative.
     * Once everything works you can increase this.
     * ----------------------------------------------------- */

    spi_device_interface_config_t device_config =
    {
        .clock_speed_hz =
            10 * 1000 * 1000,

        .mode = 0,

        .spics_io_num =
            LCD_CS,

        .queue_size = 7
    };


    ret =
        spi_bus_add_device(
            SPI2_HOST,
            &device_config,
            &lcd->spi
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Could not add LCD SPI device"
        );

        return ret;
    }


    /* -----------------------------------------------------
     * HARDWARE RESET
     * ----------------------------------------------------- */

    gpio_set_level(
        lcd->rst_pin,
        1
    );

    vTaskDelay(
        pdMS_TO_TICKS(10)
    );


    gpio_set_level(
        lcd->rst_pin,
        0
    );

    vTaskDelay(
        pdMS_TO_TICKS(20)
    );


    gpio_set_level(
        lcd->rst_pin,
        1
    );

    vTaskDelay(
        pdMS_TO_TICKS(120)
    );


    /* -----------------------------------------------------
     * SOFTWARE RESET
     * ----------------------------------------------------- */

    lcd_command(
        lcd,
        ST7796S_SWRESET
    );

    vTaskDelay(
        pdMS_TO_TICKS(150)
    );


    /* -----------------------------------------------------
     * SLEEP OUT
     * ----------------------------------------------------- */

    lcd_command(
        lcd,
        ST7796S_SLPOUT
    );

    vTaskDelay(
        pdMS_TO_TICKS(120)
    );


    /* -----------------------------------------------------
     * RGB565 - 16 BIT COLOR
     * ----------------------------------------------------- */

    lcd_command(
        lcd,
        ST7796S_COLMOD
    );


    uint8_t color_mode =
        0x55;


    lcd_data(
        lcd,
        &color_mode,
        1
    );


    /* -----------------------------------------------------
     * DISPLAY ORIENTATION
     *
     * 0x28 gives 480 x 320 landscape orientation on many
     * ST7796S modules.
     * ----------------------------------------------------- */

    lcd_command(
        lcd,
        ST7796S_MADCTL
    );


    uint8_t madctl =
        0x28;


    lcd_data(
        lcd,
        &madctl,
        1
    );


    /* -----------------------------------------------------
     * INVERSION
     * ----------------------------------------------------- */

    lcd_command(
        lcd,
        ST7796S_INVON
    );


    /* -----------------------------------------------------
     * DISPLAY ON
     * ----------------------------------------------------- */

    lcd_command(
        lcd,
        ST7796S_DISPON
    );


    vTaskDelay(
        pdMS_TO_TICKS(100)
    );


    /* Backlight ON */

    st7796s_backlight(
        lcd,
        true
    );


    ESP_LOGI(
        TAG,
        "ST7796S initialized"
    );


    return ESP_OK;
}


/* =========================================================
 * FILL RECTANGLE
 * ========================================================= */

esp_err_t st7796s_fill_rect(
    st7796s_t *lcd,
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    uint16_t color
)
{
    if (
        x >= ST7796S_WIDTH ||
        y >= ST7796S_HEIGHT ||
        width == 0 ||
        height == 0
    )
    {
        return ESP_OK;
    }


    /*
     * Clip rectangle to screen.
     */

    if (
        x + width >
        ST7796S_WIDTH
    )
    {
        width =
            ST7796S_WIDTH - x;
    }


    if (
        y + height >
        ST7796S_HEIGHT
    )
    {
        height =
            ST7796S_HEIGHT - y;
    }


    esp_err_t ret =
        st7796s_set_window(
            lcd,
            x,
            y,
            x + width - 1,
            y + height - 1
        );


    if (ret != ESP_OK)
    {
        return ret;
    }


    /*
     * RGB565 is transmitted high byte first.
     */

    uint8_t high =
        color >> 8;

    uint8_t low =
        color & 0xFF;


    /*
     * Small reusable buffer.
     *
     * 128 pixels = 256 bytes.
     */

    uint8_t buffer[256];


    for (
        int i = 0;
        i < sizeof(buffer);
        i += 2
    )
    {
        buffer[i] =
            high;

        buffer[i + 1] =
            low;
    }


    uint32_t pixels =
        (uint32_t)width *
        (uint32_t)height;


    gpio_set_level(
        lcd->dc_pin,
        1
    );


    while (pixels > 0)
    {
        uint32_t pixels_this_time =
            pixels > 128 ?
            128 :
            pixels;


        ret =
            lcd_spi_write(
                lcd,
                buffer,
                pixels_this_time * 2
            );


        if (ret != ESP_OK)
        {
            return ret;
        }


        pixels -=
            pixels_this_time;
    }


    return ESP_OK;
}


/* =========================================================
 * FILL ENTIRE SCREEN
 * ========================================================= */

esp_err_t st7796s_fill_screen(
    st7796s_t *lcd,
    uint16_t color
)
{
    return st7796s_fill_rect(
        lcd,
        0,
        0,
        ST7796S_WIDTH,
        ST7796S_HEIGHT,
        color
    );
}


/* =========================================================
 * DRAW ONE PIXEL
 * ========================================================= */

esp_err_t st7796s_draw_pixel(
    st7796s_t *lcd,
    uint16_t x,
    uint16_t y,
    uint16_t color
)
{
    if (
        x >= ST7796S_WIDTH ||
        y >= ST7796S_HEIGHT
    )
    {
        return ESP_OK;
    }


    return st7796s_fill_rect(
        lcd,
        x,
        y,
        1,
        1,
        color
    );
}


/* =========================================================
 * FIND CHARACTER IN FONT
 * ========================================================= */

static const uint8_t *find_character(
    char character
)
{
    /*
     * Convert lowercase letters to uppercase.
     */

    if (
        character >= 'a' &&
        character <= 'z'
    )
    {
        character =
            character -
            'a' +
            'A';
    }


    for (
        size_t i = 0;
        i < FONT_COUNT;
        i++
    )
    {
        if (
            font[i].character ==
            character
        )
        {
            return font[i].data;
        }
    }


    /*
     * Unsupported character becomes a space.
     */

    return font[0].data;
}


/* =========================================================
 * DRAW ONE CHARACTER
 * ========================================================= */

void st7796s_draw_char(
    st7796s_t *lcd,
    uint16_t x,
    uint16_t y,
    char character,
    uint16_t color,
    uint8_t scale
)
{
    if (scale == 0)
    {
        scale = 1;
    }


    const uint8_t *character_data =
        find_character(
            character
        );


    /*
     * Font is 5 pixels wide and 7 pixels tall.
     */

    for (
        int column = 0;
        column < 5;
        column++
    )
    {

        uint8_t column_data =
            character_data[column];


        for (
            int row = 0;
            row < 7;
            row++
        )
        {

            if (
                column_data &
                (1 << row)
            )
            {

                st7796s_fill_rect(
                    lcd,

                    x +
                    column * scale,

                    y +
                    row * scale,

                    scale,
                    scale,

                    color
                );
            }
        }
    }
}


/* =========================================================
 * DRAW TEXT
 * ========================================================= */

void st7796s_draw_text(
    st7796s_t *lcd,
    uint16_t x,
    uint16_t y,
    const char *text,
    uint16_t color,
    uint8_t scale
)
{
    if (
        lcd == NULL ||
        text == NULL
    )
    {
        return;
    }


    if (scale == 0)
    {
        scale = 1;
    }


    uint16_t cursor_x =
        x;

    uint16_t cursor_y =
        y;


    /*
     * Character:
     *
     * 5 pixels wide
     * +
     * 1 pixel spacing
     *
     * = 6 pixels per character
     */

    uint16_t character_width =
        6 * scale;


    uint16_t character_height =
        8 * scale;


    while (*text)
    {

        /*
         * New line support.
         */

        if (*text == '\n')
        {
            cursor_x =
                x;

            cursor_y +=
                character_height;

            text++;

            continue;
        }


        /*
         * Stop if text reaches bottom.
         */

        if (
            cursor_y +
            (7 * scale) >=
            ST7796S_HEIGHT
        )
        {
            break;
        }


        /*
         * Wrap to next line if necessary.
         */

        if (
            cursor_x +
            (5 * scale) >=
            ST7796S_WIDTH
        )
        {
            cursor_x =
                x;

            cursor_y +=
                character_height;
        }


        st7796s_draw_char(
            lcd,
            cursor_x,
            cursor_y,
            *text,
            color,
            scale
        );


        cursor_x +=
            character_width;


        text++;
    }
}