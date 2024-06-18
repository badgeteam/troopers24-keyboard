#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/cdefs.h>

#include "freertos/event_groups.h"
#include "pca9555.h"

__BEGIN_DECLS

/* Front */
#define PIN_BTN_DOWN   0
#define PIN_BTN_LEFT   1
#define PIN_BTN_RIGHT  3
#define PIN_BTN_UP     2
#define PIN_BTN_PUSH   4
#define PIN_BTN_A      5
#define PIN_BTN_B      6
#define PIN_BTN_START  7
#define PIN_BTN_SELECT 8

/* Keys */

typedef enum {
    BUTTON_ACCEPT = 0,
    BUTTON_BACK,
    BUTTON_START,
    BUTTON_SELECT,
    JOYSTICK_UP,
    JOYSTICK_DOWN,
    JOYSTICK_LEFT,
    JOYSTICK_RIGHT,
    JOYSTICK_PUSH,
} Key;

typedef void (*sao_detect_fn_t)(bool);

/* Structs */

typedef struct _keyboard_input_message {
    uint8_t input;
    bool    state;
} keyboard_input_message_t;

typedef struct Keyboard {
    // Pins
    int     i2c_bus;
    int     intr_pin;
    uint8_t         pca_addr;
    uint8_t pin_sao_presence;
    sao_detect_fn_t sao_presence_cb;
    // Internal state
    PCA9555*      pca;
    QueueHandle_t queue;
    TaskHandle_t  intr_task_handle;
    // Mutex
    SemaphoreHandle_t intr_trigger;
    SemaphoreHandle_t i2c_semaphore;
} Keyboard;

typedef void (*send_fn_t)(Keyboard*, uint8_t, bool);

/* Public methods */

esp_err_t keyboard_init(Keyboard* device);
bool keyboard_key_was_pressed(Keyboard* device, Key key);

__END_DECLS
