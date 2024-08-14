/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
/* STEP 3 - Include the header file of the UART driver in main.c */
#include <zephyr/drivers/uart.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* STEP 9.1.1 - Define the size of the receive buffer */
#define RECEIVE_BUFF_SIZE 10

/* STEP 9.2 - Define the receiving timeout period */
#define RECEIVE_TIMEOUT 100

/* STEP 5.1 - Get C identifiers for the DeviceTree labels and properties' values related to LEDs */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* STEP 8.1 - Define the transmission buffer, which is a buffer to hold the data to be sent over UART */
static uint8_t tx_buf[] = "Async UART Demo: enter 1 to toggle LED.\n\r";

/* STEP 9.1.2 - Define the receive buffer */
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = { 0 };

/* STEP 6 - Define the callback function for UART */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    (void)user_data;

    switch (evt->type)
    {
        case UART_RX_RDY:
            if (1 == evt->data.rx.len &&
                '1' == evt->data.rx.buf[evt->data.rx.offset]) {
                gpio_pin_toggle_dt(&led0);
            }
            break;

        case UART_RX_DISABLED:
            uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
            break;

        default:
            break;
    }
}

int main(void)
{
	int ret;

    /* STEP 4 - Get the device struct of the UART hardware */
    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart)) {
        printk("UART device is not ready!\n\r");
        return 1;
    }

    /* STEP 5.2 - Configure the LEDs */
    if (!device_is_ready(led0.port)) {
        printk("GPIO devie is not ready!\r\n");
        return 1;
    }
	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 1;
	}

    /* STEP 7 - Register the UART callback function */
    ret = uart_callback_set(uart, uart_cb, NULL);
    if (ret) {
        return 1;
    }

    /* STEP 8.2 - Send the data over UART by calling uart_tx() */
    ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
    if (ret) {
        return 1;
    }

    /* STEP 9.3  - Start receiving by calling uart_rx_enable() and pass it the address of the receive  buffer */
    ret = uart_rx_enable(uart, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
    if (ret) {
        return 1;
    }

	while (1) {
		k_msleep(SLEEP_TIME_MS);
	}
}
