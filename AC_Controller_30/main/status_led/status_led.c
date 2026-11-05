#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "wifiConnect.h"
#include "status_led.h"

static const char *TAG = "STATUS_LED";

#define STATUS_LED_TICK_MS  10
#define STATUS_LED_GAP_MS   100   // minimum off-time between two blinks

#define LED_LEVEL(on) ((on) ? STATUS_LED_ON_LEVEL : !STATUS_LED_ON_LEVEL)
#define DUE(now, t)   ((int32_t)((now) - (t)) >= 0)

/* Written by the pass-through TCP task, read by the LED task. Each is a
   single 32-bit word, so loads/stores are atomic on the S3 and no lock is
   needed. */
static volatile bool s_server_listening;
static volatile TickType_t s_last_activity;
static volatile uint32_t s_response_count;

typedef enum {
    MODE_NOT_LISTENING,   // blue solid on
    MODE_ACTIVE,          // blue heartbeat + response double-blinks
    MODE_IDLE,            // no command for the timeout: red blinks
} led_mode_t;

typedef struct {
    gpio_num_t gpio;
    bool lit;
    TickType_t off_at;    // when the current blink ends
    TickType_t ready_at;  // earliest start of the next blink
} led_t;

void status_led_set_listening(bool listening)
{
    s_server_listening = listening;
}

void status_led_tcp_activity(void)
{
    s_last_activity = xTaskGetTickCount();
}

void status_led_tcp_response(void)
{
    s_response_count++;
}

static void led_flash(led_t *led, TickType_t now)
{
    gpio_set_level(led->gpio, LED_LEVEL(true));
    led->lit = true;
    led->off_at = now + pdMS_TO_TICKS(STATUS_LED_FLASH_MS);
}

static void led_update(led_t *led, TickType_t now)
{
    if (led->lit && DUE(now, led->off_at)) {
        gpio_set_level(led->gpio, LED_LEVEL(false));
        led->lit = false;
        led->ready_at = now + pdMS_TO_TICKS(STATUS_LED_GAP_MS);
    }
}

static void led_off(led_t *led, TickType_t now)
{
    gpio_set_level(led->gpio, LED_LEVEL(false));
    led->lit = false;
    led->ready_at = now;
}

static void status_led_task(void *arg)
{
    led_t red  = { .gpio = STATUS_LED_RED_GPIO };
    led_t blue = { .gpio = STATUS_LED_BLUE_GPIO };
    led_mode_t mode = MODE_NOT_LISTENING;
    uint32_t seen_count = s_response_count;
    int cmd_blinks_left = 0;
    TickType_t next_beat = xTaskGetTickCount();

    /* Boot state: server not up yet. */
    gpio_set_level(STATUS_LED_BLUE_GPIO, LED_LEVEL(true));

    while (1) {
        TickType_t now = xTaskGetTickCount();
        bool listening = s_server_listening && wifi_is_connected;
        led_mode_t new_mode;

        if (!listening) {
            new_mode = MODE_NOT_LISTENING;
        } else if (mode == MODE_NOT_LISTENING) {
            /* Just started listening: the no-command timeout counts from
               here, not from boot. */
            s_last_activity = now;
            new_mode = MODE_ACTIVE;
        } else {
            bool idle = (now - s_last_activity) >= pdMS_TO_TICKS(STATUS_LED_IDLE_TIMEOUT_MS);
            new_mode = idle ? MODE_IDLE : MODE_ACTIVE;
        }

        if (new_mode != mode) {
            switch (new_mode) {
            case MODE_NOT_LISTENING:
                ESP_LOGW(TAG, "Pass-through server not listening");
                break;
            case MODE_ACTIVE:
                if (mode == MODE_IDLE) {
                    ESP_LOGI(TAG, "TCP commands resumed");
                } else {
                    ESP_LOGI(TAG, "Pass-through server listening");
                }
                break;
            case MODE_IDLE:
                ESP_LOGW(TAG, "No TCP command for %d ms", STATUS_LED_IDLE_TIMEOUT_MS);
                break;
            }
            mode = new_mode;
            led_off(&red, now);
            led_off(&blue, now);
            if (mode == MODE_NOT_LISTENING) {
                gpio_set_level(STATUS_LED_BLUE_GPIO, LED_LEVEL(true));
            }
            cmd_blinks_left = 0;
            seen_count = s_response_count;
            next_beat = now;
        }

        switch (mode) {
        case MODE_NOT_LISTENING:
            /* Blue held on by the transition above; nothing to animate. */
            break;

        case MODE_IDLE:
            led_update(&red, now);
            if (DUE(now, next_beat)) {
                led_flash(&red, now);
                next_beat = now + pdMS_TO_TICKS(STATUS_LED_PERIOD_MS);
            }
            break;

        case MODE_ACTIVE:
            led_update(&blue, now);
            if (blue.lit || !DUE(now, blue.ready_at)) {
                break;
            }
            /* Commands arriving mid-burst are folded into the current
               burst rather than extending it. */
            if (cmd_blinks_left == 0 && s_response_count != seen_count) {
                seen_count = s_response_count;
                cmd_blinks_left = STATUS_LED_CMD_BLINKS;
            }
            if (cmd_blinks_left > 0) {
                /* Response burst; restart the heartbeat so it resumes a
                   full period after the burst. */
                cmd_blinks_left--;
                led_flash(&blue, now);
                next_beat = now + pdMS_TO_TICKS(STATUS_LED_PERIOD_MS);
            } else if (DUE(now, next_beat)) {
                led_flash(&blue, now);
                next_beat = now + pdMS_TO_TICKS(STATUS_LED_PERIOD_MS);
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(STATUS_LED_TICK_MS));
    }
}

void status_led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << STATUS_LED_RED_GPIO) | (1ULL << STATUS_LED_BLUE_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(STATUS_LED_RED_GPIO, LED_LEVEL(false));
    gpio_set_level(STATUS_LED_BLUE_GPIO, LED_LEVEL(false));

    xTaskCreate(status_led_task, "status_led_task", 2048, NULL, 3, NULL);
}
