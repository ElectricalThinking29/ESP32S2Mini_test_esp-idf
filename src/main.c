#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include <string.h>
#include "esp_log.h"

#define BUILTIN_LED_GPIO 15 // GPIO15 is used for the onboard LED of the ESP32-S2 Mini

void app_main()
{
    // GPIO configuration
    esp_rom_gpio_pad_select_gpio(BUILTIN_LED_GPIO);
    gpio_set_direction(BUILTIN_LED_GPIO, GPIO_MODE_OUTPUT);

    // Uart configuration for USB CDC
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);

    // Initialize UART1 for USB CDC
    uart_set_pin(UART_NUM_1, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);

    // LED control logic
    bool led_state = true;

    while (1)
    {
        gpio_set_level(BUILTIN_LED_GPIO, led_state); // Turn LED on

        // Toggle LED state
        led_state = !led_state;

        // Send descriptive message about LED state to the USB CDC port
        const char *led_message = (led_state) ? "LED is ON" : "LED is OFF";
        uart_write_bytes(UART_NUM_1, led_message, strlen(led_message));

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}
