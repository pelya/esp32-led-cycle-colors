#include <algorithm>
#include <sys/time.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <esp_sleep.h>
#include <esp_random.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "led_strip.h"

#include "sdkconfig.h"

#if (CONFIG_IDF_TARGET_ESP32C3) || (CONFIG_IDF_TARGET_ESP32C6)
#define CONFIG_LED_GPIO 8
#define CONFIG_BOOT_BTN_GPIO 9
#elif (CONFIG_IDF_TARGET_ESP32S3)
#define CONFIG_LED_GPIO 21
#define CONFIG_BOOT_BTN_GPIO 0
#else
#error "Unsupported board type"
#endif

static const char *TAG = "main";

typedef struct rgb {
  float r, g, b;
} RGB;

/*
 * Converts an HUE to r, g or b.
 * returns float in the set [0, 1].
 */
float hue2rgb(float p, float q, float t) {
  if (t < 0)
    t += 1;
  if (t > 1)
    t -= 1;
  if (t < 1./6) 
    return p + (q - p) * 6 * t;
  if (t < 1./2)
    return q;
  if (t < 2./3)
    return p + (q - p) * (2./3 - t) * 6;

  return p;
}

////////////////////////////////////////////////////////////////////////

/*
 * Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes h, s, and l are contained in the set [0, 1] and
 * returns RGB in the set [0, 255].
 */
RGB hsl2rgb(float h, float s, float l) {
  RGB result;

  if (0 == s) {
    result.r = result.g = result.b = l * 255; // achromatic
  } else {
    float q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    float p = 2 * l - q;
    result.r = hue2rgb(p, q, h + 1./3) * 255;
    result.g = hue2rgb(p, q, h) * 255;
    result.b = hue2rgb(p, q, h - 1./3) * 255;
  }

  return result;
}

extern "C" {
  void app_main(void);
}

void app_main(void)
{
  led_strip_handle_t rgb_led;

  led_strip_config_t strip_config = {
    .strip_gpio_num = (gpio_num_t)CONFIG_LED_GPIO,
    .max_leds = 1,
    .led_model = LED_MODEL_WS2812,
    .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    .flags = { .invert_out = 0 },
  };

  led_strip_rmt_config_t rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 10 * 1000 * 1000, // 10MHz
    .mem_block_symbols = 0,
    .flags = { .with_dma = 0 },
  };

  led_strip_new_rmt_device(&strip_config, &rmt_config, &rgb_led);

  /* Set all LED off to clear all pixels */
  led_strip_clear(rgb_led);

  ESP_LOGI(TAG, "RGB LED initialized");

  ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t)CONFIG_BOOT_BTN_GPIO));
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)CONFIG_BOOT_BTN_GPIO, GPIO_MODE_INPUT));
  ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)CONFIG_BOOT_BTN_GPIO, GPIO_PULLUP_ONLY));

  RGB prev = { 0, 0, 0 }, next = { 0, 0, 0 };

  const int LOOP_DELAY_MS = 50;
  // Turn off in 5 minutes
  const int SHUT_OFF_TIME = 5 * 60 * 1000 / LOOP_DELAY_MS;
  const float COLOR_SHIFT_SPEED = 0.4f;
  const float COLOR_SHIFT_MAX_STEP = 15.0f;

  for (int counter = 0; counter < SHUT_OFF_TIME; counter++) {
    vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));

    if (counter > 500 / LOOP_DELAY_MS && gpio_get_level((gpio_num_t)CONFIG_BOOT_BTN_GPIO) == 0) {
      // BOOT button pressed, power off early
      break;
    }

    if (fabsf(prev.r - next.r) + fabsf(prev.g - next.g) + fabsf(prev.b - next.b) < 0.5f) {
      float rndf = (esp_random() % 65536) / 65535.0f;
      next = hsl2rgb(rndf, 1.0f, 0.5f);
      ESP_LOGI(TAG, "Next color: %3d %3d %3d hsv %f 1 0.5", (int)nearbyintf(next.r), (int)nearbyintf(next.g), (int)nearbyintf(next.b), rndf);
    }

    float diff;

    diff = (next.r - prev.r) * COLOR_SHIFT_SPEED;
    if (fabsf(diff) > COLOR_SHIFT_MAX_STEP) {
      diff = diff > 0 ? COLOR_SHIFT_MAX_STEP : -COLOR_SHIFT_MAX_STEP;
    }
    prev.r += diff;

    diff = (next.g - prev.g) * COLOR_SHIFT_SPEED;
    if (fabsf(diff) > COLOR_SHIFT_MAX_STEP) {
      diff = diff > 0 ? COLOR_SHIFT_MAX_STEP : -COLOR_SHIFT_MAX_STEP;
    }
    prev.g += diff;

    diff = (next.b - prev.b) * COLOR_SHIFT_SPEED;
    if (fabsf(diff) > COLOR_SHIFT_MAX_STEP) {
      diff = diff > 0 ? COLOR_SHIFT_MAX_STEP : -COLOR_SHIFT_MAX_STEP;
    }
    prev.b += diff;

    ESP_LOGI(TAG, "%3d %3d %3d", (int)nearbyintf(prev.r), (int)nearbyintf(prev.g), (int)nearbyintf(prev.b));
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color, 2 is almost invisible, 4 is dim */
    led_strip_set_pixel(rgb_led, 0,
      std::max(0, std::min(255, (int)nearbyintf(prev.r))),
      std::max(0, std::min(255, (int)nearbyintf(prev.g))),
      std::max(0, std::min(255, (int)nearbyintf(prev.b))));
    /* Refresh the strip to send data */
    led_strip_refresh(rgb_led);
  }

  ESP_LOGI(TAG, "Power off");

  /* Set all LED off to clear all pixels */
  led_strip_clear(rgb_led);

  esp_deep_sleep_start();
}
