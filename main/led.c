#include "led.h"
#include <esp_log.h>
#include <esp_err.h>
#include "driver/rmt.h"
#include <math.h>

#define BM_USE_RGB_LED false
#define BM_USE_NEOPIXEL false

#define TAG "led"

#if BM_USE_RGB_LED
#include <driver/ledc.h>

#define LED_PIN_RED (6)
#define LED_PIN_GREEN (5)
#define LED_PIN_BLUE (4)

#define LEDC_MODE LEDC_LOW_SPEED_MODE

#define LED_PWM_MAX_VAL 256U

#define LED_RED_MAX_VAL 20U
#define LED_GREEN_MAX_VAL 20U
#define LED_BLUE_MAX_VAL 20U

typedef enum {
    LedChannelRed,
    LedChannelGreen,
    LedChannelBlue,
} ledc_channel;
#endif

#if BM_USE_NEOPIXEL
#define WS2812_RMT_CHANNEL  (RMT_CHANNEL_3)
#define WS2812_POWER_PIN    (GPIO_NUM_38)
#define WS2812_LED_PIN      (GPIO_NUM_39)

#define WS2812_NUM_LEDS (1u)
#define WS2812_BRIGHTNESS (50u) // Out of 255

#include "ws2812_control.h"

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led_colors_t;

led_colors_t np_colors;
#endif

void led_init() {
    ESP_LOGI(TAG, "init");

#if BM_USE_RGB_LED
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_red = {
        .speed_mode = LEDC_MODE,
        .channel = LedChannelRed,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LED_PIN_RED,
        .duty = LED_PWM_MAX_VAL, // Set duty to 100%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red));

    ledc_channel_config_t ledc_channel_green = {
        .speed_mode = LEDC_MODE,
        .channel = LedChannelGreen,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LED_PIN_GREEN,
        .duty = LED_PWM_MAX_VAL, // Set duty to 100%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green));

    ledc_channel_config_t ledc_channel_blue = {
        .speed_mode = LEDC_MODE,
        .channel = LedChannelBlue,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LED_PIN_BLUE,
        .duty = LED_PWM_MAX_VAL, // Set duty to 100%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_blue));
#endif

#if BM_USE_NEOPIXEL
    gpio_set_level(WS2812_POWER_PIN, 1);
	ws2812_control_init();
#endif

    ESP_LOGI(TAG, "init done");
}

void led_set(uint8_t red, uint8_t green, uint8_t blue) {
#if BM_USE_RGB_LED
    led_set_red(red);
    led_set_green(green);
    led_set_blue(blue);
#endif

#if BM_USE_NEOPIXEL
    np_colors.green = green;
    np_colors.red = red;
    np_colors.blue = blue;
    struct led_state state;
    for (uint i = 0; i < WS2812_NUM_LEDS; i++) {
        state.leds[i] = (uint32_t)(
            ((uint32_t)ceil((green + WS2812_BRIGHTNESS) & 0xFF) << 16) &
            ((uint32_t)ceil((red + WS2812_BRIGHTNESS) & 0xFF) << 8) &
            (uint32_t)ceil((blue + WS2812_BRIGHTNESS) & 0xFF)
        );
    }
    ws2812_write_leds(state);
#endif
}

void led_set_red(uint8_t value) {
#if BM_USE_RGB_LED
    uint32_t pwm_value = ((uint32_t)value * LED_RED_MAX_VAL) / 255;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LedChannelRed, LED_PWM_MAX_VAL - pwm_value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LedChannelRed));
#endif

#if BM_USE_NEOPIXEL
    led_set(value, np_colors.green, np_colors.blue);
#endif
}

void led_set_green(uint8_t value) {
#if BM_USE_RGB_LED
    uint32_t pwm_value = ((uint32_t)value * LED_GREEN_MAX_VAL) / 255;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LedChannelGreen, LED_PWM_MAX_VAL - pwm_value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LedChannelGreen));
#endif

#if BM_USE_NEOPIXEL
    led_set(np_colors.red, value, np_colors.blue);
#endif
}

void led_set_blue(uint8_t value) {
#if BM_USE_RGB_LED
    uint32_t pwm_value = ((uint32_t)value * LED_BLUE_MAX_VAL) / 255;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LedChannelBlue, LED_PWM_MAX_VAL - pwm_value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LedChannelBlue));
#endif

#if BM_USE_NEOPIXEL
    led_set(np_colors.red, np_colors.green, value);
#endif
}
