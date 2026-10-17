#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    EVT_TIMER = 0,          /* data : timer id */
    EVT_BTN_USER_CLICK,     /* User button (PC13) debounced click */
    EVT_BTN_PWR_CLICK,      /* Power button (PA11) debounced click */
    EVT_BLE_CMD,            /* data : ble_command_type_t */
    EVT_TOF_OBJECT,         /* data : nearest ToF distance in mm */
    EVT_CAN_COLOR,          /* data : detected can color id */
    EVT_BATTERY_LOW,        /* data : battery voltage in mV */
} app_event_type_t;

typedef struct {
    app_event_type_t type;
    int32_t data;
} app_event_t;

#endif /* APP_EVENTS_H */
