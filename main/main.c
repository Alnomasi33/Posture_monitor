/* easy pinout diagram for lcd - https://www.lcdwiki.com/4.0inch_SPI_Module_ST7796 */

#include "st7796s.h"
#include "accelerometer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include <math.h>
#include <stdbool.h>


static const char *TAG = "POSTURE";


/* =========================================================
 * BUTTON SETTINGS
 *
 * BUTTON PIN = GPIO13
 *
 * Recommended wiring:
 *
 * GPIO13 -------- BUTTON -------- GND
 *
 * The ESP32 internal pull-up resistor is enabled.
 *
 * NOT PRESSED = HIGH
 * PRESSED     = LOW
 *
 * Do NOT connect 5V directly to GPIO13.
 * ========================================================= */

#define BUTTON_PIN          GPIO_NUM_13
#define BUTTON_DEBOUNCE_MS  50


/* =========================================================
 * ACCELEROMETER PLACEMENT AND PURPOSE
 *
 * ACC1 = LEFT SHOULDER
 *        Detects movement/tilt of the left shoulder.
 *
 * ACC2 = RIGHT SHOULDER
 *        Detects movement/tilt of the right shoulder.
 *
 * ACC1 + ACC2 are averaged together to determine whether
 * the user's shoulders have moved far enough away from
 * their calibrated good posture to trigger the
 * ROUNDED SHOULDERS warning.
 *
 *
 * ACC3 = LEFT UPPER BACK
 *        Detects movement/tilt of the left upper back.
 *
 * ACC4 = RIGHT UPPER BACK
 *        Detects movement/tilt of the right upper back.
 *
 * ACC3 + ACC4 are averaged together to determine whether
 * the user's upper back has moved far enough away from
 * their calibrated good posture to trigger the
 * SLOUCHING warning.
 *
 *
 * IMPORTANT:
 *
 * Each accelerometer gets its own baseline angle during
 * calibration. The system measures how far each sensor
 * moves away from that calibrated good-posture angle.
 *
 * ACC1 + ACC2 -> Rounded shoulder detection
 * ACC3 + ACC4 -> Slouch detection
 * ========================================================= */


/* =========================================================
 * POSTURE SETTINGS
 * ========================================================= */

/*
 * If the average upper-back deviation is greater than
 * 15 degrees, the user may be slouching.
 */
#define BACK_SLOUCH_THRESHOLD   15.0f


/*
 * If the average shoulder deviation is greater than
 * 12 degrees, the user's shoulders may be rounded.
 */
#define SHOULDER_THRESHOLD      12.0f


/*
 * Main loop runs approximately every 100 ms.
 *
 * 20 bad samples = approximately 2 seconds.
 *
 * This prevents one quick movement from immediately
 * triggering a bad-posture warning.
 */
#define BAD_POSTURE_TIME        20


/*
 * Calibration:
 *
 * 30 samples x 100 ms = approximately 3 seconds.
 */
#define CALIBRATION_SAMPLES     30


/* =========================================================
 * POSTURE STATES
 * ========================================================= */

typedef enum
{
    POSTURE_GOOD = 0,
    POSTURE_SLOUCHING,
    POSTURE_SHOULDERS

} posture_state_t;


/* =========================================================
 * BUTTON INITIALIZATION
 * ========================================================= */

static void button_init(void)
{
    gpio_config_t button_config =
    {
        .pin_bit_mask = (1ULL << BUTTON_PIN),

        .mode = GPIO_MODE_INPUT,

        /*
         * Enable ESP32 internal pull-up.
         *
         * GPIO13 normally reads HIGH.
         * Pressing the button connects GPIO13 to GND,
         * causing it to read LOW.
         */
        .pull_up_en = GPIO_PULLUP_ENABLE,

        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(&button_config)
    );


    ESP_LOGI(
        TAG,
        "Button initialized on GPIO13"
    );
}


/* =========================================================
 * WAIT FOR BUTTON PRESS
 *
 * Used during startup.
 *
 * The program will remain inside this function until
 * the user presses and releases the button.
 * ========================================================= */

static void wait_for_button_press(void)
{
    ESP_LOGI(
        TAG,
        "Waiting for button press..."
    );


    /*
     * Wait for GPIO13 to become LOW.
     */
    while (gpio_get_level(BUTTON_PIN) == 1)
    {
        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }


    /*
     * Debounce the button.
     */
    vTaskDelay(
        pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)
    );


    /*
     * Make sure button is still pressed.
     */
    while (gpio_get_level(BUTTON_PIN) == 1)
    {
        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }


    ESP_LOGI(
        TAG,
        "Button pressed"
    );


    /*
     * Wait for user to release button.
     */
    while (gpio_get_level(BUTTON_PIN) == 0)
    {
        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }


    /*
     * Debounce release.
     */
    vTaskDelay(
        pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)
    );
}


/* =========================================================
 * CHECK FOR BUTTON PRESS
 *
 * Used while the posture monitor is running.
 *
 * Returns:
 *
 * true  = button was pressed and released
 * false = button was not pressed
 * ========================================================= */

static bool button_pressed(void)
{
    /*
     * Button is active LOW.
     */
    if (gpio_get_level(BUTTON_PIN) == 0)
    {
        /*
         * Debounce.
         */
        vTaskDelay(
            pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)
        );


        /*
         * Check again after debounce.
         */
        if (gpio_get_level(BUTTON_PIN) == 0)
        {
            ESP_LOGI(
                TAG,
                "Recalibration button pressed"
            );


            /*
             * Wait for button release.
             */
            while (gpio_get_level(BUTTON_PIN) == 0)
            {
                vTaskDelay(
                    pdMS_TO_TICKS(10)
                );
            }


            /*
             * Debounce release.
             */
            vTaskDelay(
                pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)
            );


            return true;
        }
    }


    return false;
}


/* =========================================================
 * CALCULATE SENSOR TILT ANGLE
 *
 * Converts accelerometer X/Y/Z data into an angle
 * measured in degrees.
 *
 * This angle is compared to the angle recorded during
 * calibration.
 * ========================================================= */

static float calculate_angle(
    accelerometer_data_t *accel
)
{
    return atan2f(
        accel->x,
        sqrtf(
            accel->y * accel->y +
            accel->z * accel->z
        )
    ) * 180.0f / M_PI;
}


/* =========================================================
 * DISPLAY READY SCREEN
 * ========================================================= */

static void display_ready(
    st7796s_t *lcd
)
{
    st7796s_fill_screen(
        lcd,
        ST7796S_WHITE
    );


    st7796s_draw_text(
        lcd,
        165,
        75,
        "READY",
        ST7796S_BLACK,
        4
    );


    st7796s_draw_text(
        lcd,
        110,
        145,
        "PRESS BUTTON",
        ST7796S_BLACK,
        3
    );


    st7796s_draw_text(
        lcd,
        105,
        190,
        "TO CALIBRATE",
        ST7796S_BLACK,
        3
    );
}


/* =========================================================
 * DISPLAY CALIBRATION SCREEN
 * ========================================================= */

static void display_calibration(
    st7796s_t *lcd
)
{
    st7796s_fill_screen(
        lcd,
        ST7796S_WHITE
    );


    st7796s_draw_text(
        lcd,
        100,
        100,
        "CALIBRATING",
        ST7796S_BLACK,
        4
    );


    st7796s_draw_text(
        lcd,
        120,
        170,
        "SIT STRAIGHT",
        ST7796S_BLACK,
        3
    );
}


/* =========================================================
 * DISPLAY SENSOR ERROR
 * ========================================================= */

static void display_sensor_error(
    st7796s_t *lcd
)
{
    st7796s_fill_screen(
        lcd,
        ST7796S_WHITE
    );


    st7796s_draw_text(
        lcd,
        115,
        110,
        "SENSOR ERROR",
        ST7796S_BLACK,
        3
    );


    st7796s_draw_text(
        lcd,
        115,
        165,
        "CHECK WIRING",
        ST7796S_BLACK,
        3
    );
}


/* =========================================================
 * DISPLAY POSTURE MESSAGE
 *
 * LCD uses:
 *
 * WHITE BACKGROUND
 * BLACK TEXT
 * ========================================================= */

static void display_posture(
    st7796s_t *lcd,
    posture_state_t posture
)
{
    /*
     * Clear previous message.
     */
    st7796s_fill_screen(
        lcd,
        ST7796S_WHITE
    );


    switch (posture)
    {

        /* =================================================
         * GOOD POSTURE
         * ================================================= */

        case POSTURE_GOOD:

            st7796s_draw_text(
                lcd,
                95,
                120,
                "GOOD POSTURE",
                ST7796S_BLACK,
                4
            );


            st7796s_draw_text(
                lcd,
                155,
                180,
                "KEEP IT UP!",
                ST7796S_BLACK,
                2
            );


            ESP_LOGI(
                TAG,
                "GOOD POSTURE"
            );

            break;


        /* =================================================
         * SLOUCHING
         *
         * Controlled by:
         *
         * ACC3 = LEFT UPPER BACK
         * ACC4 = RIGHT UPPER BACK
         * ================================================= */

        case POSTURE_SLOUCHING:

            st7796s_draw_text(
                lcd,
                65,
                100,
                "YOU ARE SLOUCHING",
                ST7796S_BLACK,
                3
            );


            st7796s_draw_text(
                lcd,
                65,
                165,
                "STRAIGHTEN YOUR BACK",
                ST7796S_BLACK,
                3
            );


            ESP_LOGW(
                TAG,
                "YOU ARE SLOUCHING - STRAIGHTEN YOUR BACK"
            );

            break;


        /* =================================================
         * ROUNDED SHOULDERS
         *
         * Controlled by:
         *
         * ACC1 = LEFT SHOULDER
         * ACC2 = RIGHT SHOULDER
         * ================================================= */

        case POSTURE_SHOULDERS:

            st7796s_draw_text(
                lcd,
                80,
                100,
                "ROUNDED SHOULDERS",
                ST7796S_BLACK,
                3
            );


            st7796s_draw_text(
                lcd,
                65,
                165,
                "SPREAD YOUR SHOULDERS",
                ST7796S_BLACK,
                3
            );


            ESP_LOGW(
                TAG,
                "ROUNDED SHOULDERS - SPREAD YOUR SHOULDERS"
            );

            break;


        default:

            break;
    }
}


/* =========================================================
 * CALIBRATE ALL FOUR ACCELEROMETERS
 *
 * The current position of each accelerometer becomes
 * that sensor's GOOD POSTURE baseline.
 *
 * ACC1 -> left shoulder baseline
 * ACC2 -> right shoulder baseline
 * ACC3 -> left upper-back baseline
 * ACC4 -> right upper-back baseline
 * ========================================================= */

static esp_err_t calibrate_sensors(
    st7796s_t *lcd,
    accelerometer_data_t sensors[4],
    float baseline[4]
)
{
    int calibration_count[4] =
    {
        0,
        0,
        0,
        0
    };


    /*
     * Erase old calibration values.
     */
    for (int i = 0; i < 4; i++)
    {
        baseline[i] = 0.0f;
    }


    /*
     * Tell the user to sit in their desired
     * good-posture position.
     */
    display_calibration(
        lcd
    );


    ESP_LOGI(
        TAG,
        "======================================"
    );

    ESP_LOGI(
        TAG,
        "CALIBRATION STARTING"
    );

    ESP_LOGI(
        TAG,
        "SIT STRAIGHT AND SPREAD YOUR SHOULDERS"
    );

    ESP_LOGI(
        TAG,
        "HOLD GOOD POSTURE FOR 3 SECONDS"
    );

    ESP_LOGI(
        TAG,
        "======================================"
    );


    /* =====================================================
     * COLLECT CALIBRATION DATA
     * ===================================================== */

    for (
        int sample = 0;
        sample < CALIBRATION_SAMPLES;
        sample++
    )
    {

        for (int i = 0; i < 4; i++)
        {
            esp_err_t ret =
                accelerometer_read(
                    (accelerometer_id_t)i,
                    &sensors[i]
                );


            if (ret == ESP_OK)
            {
                float angle =
                    calculate_angle(
                        &sensors[i]
                    );


                /*
                 * Add angle to running total.
                 */
                baseline[i] += angle;


                calibration_count[i]++;


                ESP_LOGI(
                    TAG,
                    "CAL ACC%d X:%.2f Y:%.2f Z:%.2f ANG:%.2f",
                    i + 1,
                    sensors[i].x,
                    sensors[i].y,
                    sensors[i].z,
                    angle
                );
            }

            else
            {
                ESP_LOGE(
                    TAG,
                    "Calibration read failed for ACC%d",
                    i + 1
                );
            }
        }


        /*
         * Wait 100 ms before next sample.
         */
        vTaskDelay(
            pdMS_TO_TICKS(100)
        );
    }


    /* =====================================================
     * CALCULATE NEW BASELINES
     * ===================================================== */

    for (int i = 0; i < 4; i++)
    {
        /*
         * Make sure sensor successfully produced data.
         */
        if (calibration_count[i] == 0)
        {
            ESP_LOGE(
                TAG,
                "ACC%d failed calibration",
                i + 1
            );


            display_sensor_error(
                lcd
            );


            return ESP_FAIL;
        }


        /*
         * Calculate average angle.
         *
         * This becomes the sensor's new good-posture
         * reference angle.
         */
        baseline[i] =
            baseline[i] /
            calibration_count[i];


        ESP_LOGI(
            TAG,
            "ACC%d NEW BASELINE = %.2f degrees",
            i + 1,
            baseline[i]
        );
    }


    ESP_LOGI(
        TAG,
        "======================================"
    );

    ESP_LOGI(
        TAG,
        "CALIBRATION COMPLETE"
    );

    ESP_LOGI(
        TAG,
        "======================================"
    );


    return ESP_OK;
}


/* =========================================================
 * MAIN
 * ========================================================= */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "Starting posture monitor..."
    );


    /* =====================================================
     * INITIALIZE BUTTON
     * ===================================================== */

    button_init();


    /* =====================================================
     * LCD INITIALIZATION
     * ===================================================== */

    st7796s_t lcd =
    {
        .dc_pin  = LCD_DC,
        .rst_pin = LCD_RST,
        .bl_pin  = LCD_BL
    };


    esp_err_t ret =
        st7796s_init(
            &lcd
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "LCD initialization failed"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "LCD initialized"
    );


    /* =====================================================
     * STARTING SCREEN
     * ===================================================== */

    st7796s_fill_screen(
        &lcd,
        ST7796S_WHITE
    );


    st7796s_draw_text(
        &lcd,
        85,
        120,
        "POSTURE MONITOR",
        ST7796S_BLACK,
        4
    );


    st7796s_draw_text(
        &lcd,
        175,
        180,
        "STARTING",
        ST7796S_BLACK,
        2
    );


    vTaskDelay(
        pdMS_TO_TICKS(1000)
    );


    /* =====================================================
     * ACCELEROMETER INITIALIZATION
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "Initializing accelerometers..."
    );


    ret =
        accelerometers_init();


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Accelerometer initialization failed"
        );


        display_sensor_error(
            &lcd
        );


        return;
    }


    ESP_LOGI(
        TAG,
        "All accelerometers initialized"
    );


    /* =====================================================
     * SENSOR STORAGE
     * ===================================================== */

    accelerometer_data_t sensors[4];


    /*
     * Each accelerometer has its own baseline.
     *
     * baseline[0] = ACC1 LEFT SHOULDER
     *
     * baseline[1] = ACC2 RIGHT SHOULDER
     *
     * baseline[2] = ACC3 LEFT UPPER BACK
     *
     * baseline[3] = ACC4 RIGHT UPPER BACK
     */
    float baseline[4] =
    {
        0.0f,
        0.0f,
        0.0f,
        0.0f
    };


    /* =====================================================
     * WAIT FOR FIRST CALIBRATION
     * ===================================================== */

    display_ready(
        &lcd
    );


    ESP_LOGI(
        TAG,
        "Press button on GPIO13 to begin calibration"
    );


    /*
     * Nothing happens until user presses button.
     */
    wait_for_button_press();


    /* =====================================================
     * INITIAL CALIBRATION
     * ===================================================== */

    ret =
        calibrate_sensors(
            &lcd,
            sensors,
            baseline
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Initial calibration failed"
        );

        return;
    }


    /* =====================================================
     * START POSTURE MONITOR
     * ===================================================== */

    posture_state_t current_posture =
        POSTURE_GOOD;


    int back_bad_count =
        0;


    int shoulder_bad_count =
        0;


    display_posture(
        &lcd,
        POSTURE_GOOD
    );


    ESP_LOGI(
        TAG,
        "POSTURE MONITOR ACTIVE"
    );


    /* =====================================================
     * MAIN MONITORING LOOP
     * ===================================================== */

    while (1)
    {

        /* =================================================
         * CHECK RECALIBRATION BUTTON
         *
         * Press GPIO13 at any time while monitoring.
         *
         * This erases the old baselines and records the
         * user's current straight posture as the new
         * good-posture position.
         * ================================================= */

        if (button_pressed())
        {
            ESP_LOGI(
                TAG,
                "======================================"
            );

            ESP_LOGI(
                TAG,
                "RECALIBRATION REQUESTED"
            );


            /*
             * Clear old bad-posture timers.
             */
            back_bad_count = 0;

            shoulder_bad_count = 0;


            /*
             * Recalibrate all four accelerometers.
             */
            ret =
                calibrate_sensors(
                    &lcd,
                    sensors,
                    baseline
                );


            if (ret != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "Recalibration failed"
                );


                /*
                 * Keep checking button so the user can
                 * try calibration again instead of
                 * completely stopping the program.
                 */
                display_sensor_error(
                    &lcd
                );


                vTaskDelay(
                    pdMS_TO_TICKS(500)
                );


                continue;
            }


            /*
             * The position used during recalibration is
             * now considered GOOD POSTURE.
             */
            current_posture =
                POSTURE_GOOD;


            display_posture(
                &lcd,
                POSTURE_GOOD
            );


            ESP_LOGI(
                TAG,
                "RECALIBRATION COMPLETE"
            );


            /*
             * Start next loop with new baseline values.
             */
            continue;
        }


        /* =================================================
         * SENSOR DATA
         * ================================================= */

        float angle[4];

        float difference[4];


        int sensors_read =
            0;


        /* =================================================
         * READ ALL FOUR ACCELEROMETERS
         * ================================================= */

        for (int i = 0; i < 4; i++)
        {
            ret =
                accelerometer_read(
                    (accelerometer_id_t)i,
                    &sensors[i]
                );


            if (ret == ESP_OK)
            {
                sensors_read++;


                /*
                 * Current sensor angle.
                 */
                angle[i] =
                    calculate_angle(
                        &sensors[i]
                    );


                /*
                 * Calculate how far the sensor has moved
                 * away from its calibrated position.
                 */
                difference[i] =
                    fabsf(
                        angle[i] -
                        baseline[i]
                    );


                ESP_LOGI(
                    TAG,
                    "ACC%d ANG:%.2f DIFF:%.2f",
                    i + 1,
                    angle[i],
                    difference[i]
                );
            }

            else
            {
                ESP_LOGE(
                    TAG,
                    "ACC%d READ FAILED",
                    i + 1
                );
            }
        }


        /* =================================================
         * ONLY DETECT POSTURE IF ALL 4 SENSORS WORKED
         * ================================================= */

        if (sensors_read == 4)
        {

            /* =================================================
             * SHOULDER DETECTION
             *
             * ACC1 = LEFT SHOULDER
             * ACC2 = RIGHT SHOULDER
             *
             * We calculate how far each shoulder has moved
             * from its calibrated good-posture position.
             *
             * Then we average the two shoulder deviations.
             *
             * If this average exceeds 12 degrees for about
             * 2 seconds, the system considers the shoulders
             * to be outside the calibrated posture.
             * ================================================= */

            float shoulder_difference =
                (
                    difference[0] +
                    difference[1]
                ) / 2.0f;


            /* =================================================
             * BACK / SLOUCH DETECTION
             *
             * ACC3 = LEFT UPPER BACK
             * ACC4 = RIGHT UPPER BACK
             *
             * We calculate how far each upper-back sensor
             * has moved from its calibrated position.
             *
             * Then we average the two deviations.
             *
             * If this average exceeds 15 degrees for about
             * 2 seconds, the system considers the user to
             * be slouching.
             * ================================================= */

            float back_difference =
                (
                    difference[2] +
                    difference[3]
                ) / 2.0f;


            ESP_LOGI(
                TAG,
                "SHOULDERS: %.2f | BACK: %.2f",
                shoulder_difference,
                back_difference
            );


            /* =================================================
             * CHECK FOR SLOUCHING
             *
             * ACC3 + ACC4
             * ================================================= */

            if (
                back_difference >
                BACK_SLOUCH_THRESHOLD
            )
            {
                back_bad_count++;


                if (
                    back_bad_count >
                    BAD_POSTURE_TIME
                )
                {
                    back_bad_count =
                        BAD_POSTURE_TIME;
                }
            }

            else
            {
                /*
                 * Back returned to acceptable position.
                 */
                back_bad_count = 0;
            }


            /* =================================================
             * CHECK FOR ROUNDED SHOULDERS
             *
             * ACC1 + ACC2
             * ================================================= */

            if (
                shoulder_difference >
                SHOULDER_THRESHOLD
            )
            {
                shoulder_bad_count++;


                if (
                    shoulder_bad_count >
                    BAD_POSTURE_TIME
                )
                {
                    shoulder_bad_count =
                        BAD_POSTURE_TIME;
                }
            }

            else
            {
                /*
                 * Shoulders returned to acceptable position.
                 */
                shoulder_bad_count = 0;
            }


            /* =================================================
             * DETERMINE CURRENT POSTURE
             * ================================================= */

            posture_state_t new_posture;


            /*
             * SLOUCHING:
             *
             * ACC3 + ACC4 average deviation > 15 degrees
             * for approximately 2 seconds.
             *
             * Slouching currently has priority if both
             * conditions happen at the same time.
             */
            if (
                back_bad_count >=
                BAD_POSTURE_TIME
            )
            {
                new_posture =
                    POSTURE_SLOUCHING;
            }


            /*
             * ROUNDED SHOULDERS:
             *
             * ACC1 + ACC2 average deviation > 12 degrees
             * for approximately 2 seconds.
             */
            else if (
                shoulder_bad_count >=
                BAD_POSTURE_TIME
            )
            {
                new_posture =
                    POSTURE_SHOULDERS;
            }


            /*
             * GOOD POSTURE:
             *
             * Neither condition has remained above its
             * threshold long enough to trigger a warning.
             */
            else
            {
                new_posture =
                    POSTURE_GOOD;
            }


            /* =================================================
             * UPDATE LCD ONLY WHEN POSTURE CHANGES
             * ================================================= */

            if (
                new_posture !=
                current_posture
            )
            {
                current_posture =
                    new_posture;


                display_posture(
                    &lcd,
                    current_posture
                );
            }
        }


        /* =================================================
         * SAMPLE APPROXIMATELY EVERY 100 ms
         * ================================================= */

        vTaskDelay(
            pdMS_TO_TICKS(100)
        );
    }
}