#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"


/* =========================================================
 * ST7796S DISPLAY
 * ========================================================= */

#define ST7796S_WIDTH   480
#define ST7796S_HEIGHT  320


/* =========================================================
 * LCD GPIO PINS
 * ========================================================= */

/*
 * GPIO23 -> MOSI
 * GPIO18 -> SCLK
 * GPIO5  -> CS
 * GPIO21 -> DC
 * GPIO22 -> RST
 * GPIO4  -> BL
 */

#define LCD_MOSI  GPIO_NUM_23
#define LCD_SCLK  GPIO_NUM_18
#define LCD_CS    GPIO_NUM_5
#define LCD_DC    GPIO_NUM_21
#define LCD_RST   GPIO_NUM_22
#define LCD_BL    GPIO_NUM_4


/* =========================================================
 * ST7796S COMMANDS
 * ========================================================= */

#define ST7796S_SWRESET   0x01
#define ST7796S_SLPOUT    0x11
#define ST7796S_INVOFF    0x20
#define ST7796S_INVON     0x21
#define ST7796S_DISPOFF   0x28
#define ST7796S_DISPON    0x29
#define ST7796S_CASET     0x2A
#define ST7796S_PASET     0x2B
#define ST7796S_RAMWR     0x2C
#define ST7796S_MADCTL    0x36
#define ST7796S_COLMOD    0x3A


/* =========================================================
 * RGB565 COLORS
 * ========================================================= */

#define ST7796S_BLACK      0x0000
#define ST7796S_WHITE      0xFFFF

#define ST7796S_RED        0xF800
#define ST7796S_GREEN      0x07E0
#define ST7796S_BLUE       0x001F

#define ST7796S_YELLOW     0xFFE0
#define ST7796S_CYAN       0x07FF
#define ST7796S_MAGENTA    0xF81F

#define ST7796S_PURPLE     0x780F
#define ST7796S_ORANGE     0xFD20

#define ST7796S_GRAY       0x8410
#define ST7796S_DARKGRAY   0x4208


/* =========================================================
 * LCD STRUCT
 * ========================================================= */

typedef struct
{
    spi_device_handle_t spi;

    gpio_num_t dc_pin;
    gpio_num_t rst_pin;
    gpio_num_t bl_pin;

} st7796s_t;


/* =========================================================
 * LCD FUNCTIONS
 * ========================================================= */

esp_err_t st7796s_init(
    st7796s_t *lcd
);

void st7796s_backlight(
    st7796s_t *lcd,
    bool on
);

esp_err_t st7796s_fill_screen(
    st7796s_t *lcd,
    uint16_t color
);

esp_err_t st7796s_draw_pixel(
    st7796s_t *lcd,
    uint16_t x,
    uint16_t y,
    uint16_t color
);

esp_err_t st7796s_fill_rect(
    st7796s_t *lcd,
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    uint16_t color
);


/* =========================================================
 * TEXT FUNCTIONS
 * ========================================================= */

/*
 * Draw one character.
 *
 * scale:
 * 1 = small
 * 2 = medium
 * 3 = large
 * 4 = very large
 */

void st7796s_draw_char(
    st7796s_t *lcd,
    uint16_t x,
    uint16_t y,
    char character,
    uint16_t color,
    uint8_t scale
);


/*
 * Draw a text string.
 */

void st7796s_draw_text(
    st7796s_t *lcd,
    uint16_t x,
    uint16_t y,
    const char *text,
    uint16_t color,
    uint8_t scale
);