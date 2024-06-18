#include "keyboard.h"

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdbool.h>

#include "driver/adc.h"
#include "pca9555.h"

static const char* TAG = "keyboard";

PCA9555 pca = {0};

int get_key(uint8_t pin) {
    switch (pin) {
        case PIN_BTN_DOWN:
            return JOYSTICK_DOWN;
        case PIN_BTN_LEFT:
            return JOYSTICK_LEFT;
        case PIN_BTN_RIGHT:
            return JOYSTICK_RIGHT;
        case PIN_BTN_UP:
            return JOYSTICK_UP;
        case PIN_BTN_SELECT:
            return BUTTON_SELECT;
        case PIN_BTN_A:
            return BUTTON_ACCEPT;
        case PIN_BTN_B:
            return BUTTON_BACK;
        case PIN_BTN_START:
            return BUTTON_START;
        case PIN_BTN_PUSH:
            return JOYSTICK_PUSH;
    }
    return -1;
}

bool keyboard_key_was_pressed(Keyboard* device, Key key) {
    keyboard_input_message_t buttonMessage = {0};

    if (xQueueReceive(device->queue, &buttonMessage, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (buttonMessage.state && buttonMessage.input == key) {
            return true;
        }
    }
    return false;
}

void send_key_to_queue(Keyboard* device, int key, bool state) {
    if (key < 0) return;
    keyboard_input_message_t message;
    message.input = key;
    message.state = state;
    ESP_LOGD(TAG, "Key event %d, %d", key, state);
    xQueueSend(device->queue, &message, portMAX_DELAY);
}

void handle_key(Keyboard* device, uint8_t pin, bool state) { send_key_to_queue(device, get_key(pin), state); }

/* Interrupt handling */
esp_err_t get_pca9555_state(PCA9555* device, uint16_t* current_state) {
    esp_err_t res = pca9555_get_gpio_values(device, current_state);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "failed to read input state of device %d", device->i2c_addr);
        return res;
    }
    return ESP_OK;
}

void handle_pca9555_input_change(Keyboard* keyboard, PCA9555* device, uint16_t current_state, send_fn_t send_fn, int start_pin, int num_pins) {
    for (int i = start_pin; i < num_pins; i++) {
        if (((current_state >> i) & 0x01) != (device->previous_state & (1 << i))) {
            bool value = (current_state >> i) & 0x01;
            send_fn(keyboard, i, value);
        }
    }
    device->previous_state = current_state;
}

_Noreturn static void intr_task(void* arg) {
    esp_err_t res;
    uint16_t  state;
    Keyboard* device = (Keyboard*) arg;

    while (1) {
        if (xSemaphoreTake(device->intr_trigger, portMAX_DELAY)) {
            ESP_LOGD(TAG, "Received interrupt");

            res = get_pca9555_state(device->pca, &state);

            if (res == ESP_OK) {
                bool sao_was_connected = ((device->pca->previous_state >> device->pin_sao_presence) & 1);
                bool sao_is_connected = ((state >> device->pin_sao_presence) & 1);

                if (sao_was_connected != sao_is_connected && device->sao_presence_cb != NULL) {
                    device->sao_presence_cb(sao_is_connected);
                }

                handle_pca9555_input_change(device, device->pca, state, &handle_key, 0, 9);
            } else {
                ESP_LOGE(TAG, "error while processing front pca9555 data");
            }
        }
    }
}

static void intr_handler(void* arg) { /* in interrupt handler */
    Keyboard* device = (Keyboard*) arg;
    xSemaphoreGiveFromISR(device->intr_trigger, NULL);
    portYIELD_FROM_ISR();
}

/* Initialization */

esp_err_t keyboard_init(Keyboard* device) {
    esp_err_t res;
    ESP_LOGD(TAG, "Initializing keyboard");

    device->queue = xQueueCreate(8, sizeof(keyboard_input_message_t));

    pca.i2c_bus       = device->i2c_bus;
    pca.i2c_addr      = device->pca_addr;
    pca.i2c_semaphore = device->i2c_semaphore;
    device->pca         = &pca;
    pca9555_init(device->pca);

    if (device->intr_pin) {
        // Create interrupt trigger
        device->intr_trigger = xSemaphoreCreateBinary();
        if (device->intr_trigger == NULL) return ESP_ERR_NO_MEM;

        // Attach interrupt to interrupt pin (if available)
        res = gpio_isr_handler_add(device->intr_pin, intr_handler, (void*) device);
        if (res != ESP_OK) return res;

        gpio_config_t io_conf = {
            .intr_type    = GPIO_INTR_NEGEDGE,
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = 1LL << device->intr_pin,
            .pull_down_en = 0,
            .pull_up_en   = 1,
        };

        res = gpio_config(&io_conf);
        if (res != ESP_OK) return res;

        adc_power_acquire();

        xTaskCreate(&intr_task, "PCA9555 interrupt task", 4096, device, 10, &device->intr_task_handle);
        xSemaphoreGive(device->intr_trigger);
    }

    ESP_LOGD(TAG, "Done initializing keyboard");
    return ESP_OK;
}