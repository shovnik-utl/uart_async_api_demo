/*
    What happens when a pre-emptive thread (B) tries to use the
    uart_tx function to start a transfer when another thread (A)
    has already started a transfer that is still ongoing?

    Answer: uart_tx returns -EBUSY if a transfer is already ongoing
    which may be used to hold off the second thread until the first
    is complete.
*/

#include <stdio.h>
#include <inttypes.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(async_uart, LOG_LEVEL_INF);

/* Handle for UART peripheral obtained from devicetree. */
static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

/* UART configuration: 8N1, 115200 bps, no flow control (hardware or software). */
static const struct uart_config uart_cfg = {
    .baudrate   = 115200,
    .parity     = UART_CFG_PARITY_NONE,
    .stop_bits  = UART_CFG_STOP_BITS_1,
    .data_bits  = UART_CFG_DATA_BITS_8,
    .flow_ctrl  = UART_CFG_FLOW_CTRL_NONE
};

/* Buffers to be used by the UART. */
static char uart_tx_buff[128] = { 0 };

static K_SEM_DEFINE(uart_tx_sem, 0, 1);

/* Application callback function for UART asynchronous events. */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    (void)user_data;

    switch (evt->type)
    {
        case UART_TX_DONE:
            LOG_HEXDUMP_INF(evt->data.tx.buf, evt->data.tx.len, "Last sent:");
            /* Release the binary semaphore to whichever thread is ready to take it.
               Note: giving a sempahore is a non-blocking operation so should be
               safe to call from a callback/isr context. */
            k_sem_give(&uart_tx_sem);
            break;

        default:
            break;
    }
}

static int my_uart_init(void)
{
    int err = 0;
    if (!device_is_ready(uart)) {
        LOG_ERR("UART device %s not ready.", uart->name);
        return -ENODEV;
    }

    err = uart_configure(uart, &uart_cfg);
    if (err == -ENOSYS) {
        LOG_ERR("UART port %s config failed.", uart->name);
        return -ENOSYS;
    }

    err = uart_callback_set(uart, uart_cb, NULL);
    if (err) {
        LOG_ERR("UART port %s cb reg failed (err %d).", uart->name, err);
    }
    return err;
}

int main(void) /* Main thread = thread A. */
{
    int err = 0;

    /* Prepare UART for operation. */
    err = my_uart_init();
    if (err) {
        LOG_ERR("UART init failed (err %d).", err);
        return err;
    }
    LOG_INF("UART init success.");

    uint32_t counter = 0;

    for (;;) {
        snprintf(uart_tx_buff, sizeof(uart_tx_buff), "[Thread A] Hello World %d", ++counter);
        err = uart_tx(uart, uart_tx_buff, strlen(uart_tx_buff), 0);
        if (err == -EBUSY) {
            LOG_WRN("[Thread A] Thread B's transfer is still ongoing...");
            k_sem_take(&uart_tx_sem, K_FOREVER);
        }
        /* Note: no blocking call like a sleep in this while forever loop!
           This is because we are relying instead on the above
           semaphore-take operation (which is forever blocking) to avoid
           busywaiting in thread or starving other threads. */
    }

	return 0;
}

int threadB_target_fn(void*, void*, void*)
{
    int err = 0;
    uint32_t counter = 0;
    for (;;) {
        snprintf(uart_tx_buff, sizeof(uart_tx_buff), "[Thread B] Hello World %d", ++counter);
        err = uart_tx(uart, uart_tx_buff, strlen(uart_tx_buff), 0);
        if (err == -EBUSY) {
            LOG_WRN("[Thread B] UART-TX busy, wait...");
            k_sem_take(&uart_tx_sem, K_FOREVER);
        }
    }

    return 0;
}

/*
    TODO:
    1. Start these two threads.
    2. Figure out a UART-TX access policy with regards to 2 operations:
        + Checking if return value of uart_tx is -EBUSY
        + Waiting on a semaphore is busy. 
    3. Add possible more threads to test out the policy and figure out 
       which one works best for which siutation.
*/