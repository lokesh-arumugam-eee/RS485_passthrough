#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include "soc/gpio_num.h"

/* Red/blue status LEDs, wired active-low (anode to 3.3 V).
   - Not listening : pass-through server not up (not started yet, bind/listen
                     failed, or Wi-Fi down): blue solid ON, red off.
   - Listening     : blue flashes once every STATUS_LED_PERIOD_MS.
   - Response      : quick blue double-blink each time a pass-through TCP
                     command is answered.
   - No command    : listening, but no pass-through TCP command for
                     STATUS_LED_IDLE_TIMEOUT_MS (counted from when listening
                     started): blue off, red flashes once every
                     STATUS_LED_PERIOD_MS. */

#define STATUS_LED_RED_GPIO         GPIO_NUM_10
#define STATUS_LED_BLUE_GPIO        GPIO_NUM_12
#define STATUS_LED_ON_LEVEL         0             // active low: anode to 3.3 V
#define STATUS_LED_IDLE_TIMEOUT_MS  60000
#define STATUS_LED_PERIOD_MS        2000          // heartbeat / idle blink period
#define STATUS_LED_FLASH_MS         100           // on-time of every blink
#define STATUS_LED_CMD_BLINKS       2             // blinks per answered command

void status_led_init(void);
void status_led_set_listening(bool listening); // pass-through server up/down
void status_led_tcp_activity(void);   // command received: resets idle timer
void status_led_tcp_response(void);   // response sent: blue double-blink

#endif
