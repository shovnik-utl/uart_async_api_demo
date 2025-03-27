/*
    What happens when a pre-emptive thread (B) tries to use the
    uart_tx function to start a transfer when another thread (A)
    has already started a transfer that is still on-going?

    Answer: uart_tx returns -EBUSY if a transfer is already ongoing
    which may be used to hold off the second thread until the first
    is complete. An alternative to error code handling is to use
    synchronization mechanisms.

    In this demo we launch two threads that try to send data over UART
    by accessing the same buffer and inspect mechanisms to avoid race
    conditions when accessing that buffer.
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

/* Buffer to be used by the UART. 
   Because this is a shared buffer, there is a possibility of race conditions.
   Need synchronization mechanisms to protect from race. */
static char uart_tx_buff[16] = { 0 };

/* Binary semaphore to protect shared buffer.
   We prefer a semaphore to a mutex because mutexes are not designed for use in ISRs. */
static K_SEM_DEFINE(uart_tx_sem, 1, 1); /* Initially available for first thread to take. */

/* Application callback function for UART asynchronous events. */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    (void)user_data;

    switch (evt->type)
    {
        /* We are only interested in UART-TX events. */
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

/* Generic thread params. */
#define THREAD_PRIORITY         5
#define THREAD_STACKSIZE        512
#define THREAD_SLEEP_DURATION   K_MSEC(1000)
#define THREAD_NUM_LOOPS        10
static void thread_target_fn(void*, void*, void*);

/* For this demo, we'll launch two threads: A and B. */
static char thread_names[] = { 'A', 'B' };
static k_tid_t thread_ids[] = { NULL, NULL };
K_THREAD_STACK_DEFINE(threadA_stack_area, THREAD_STACKSIZE);
K_THREAD_STACK_DEFINE(threadB_stack_area, THREAD_STACKSIZE);
static struct k_thread threadA_data;
static struct k_thread threadB_data;

int main(void)
{
    int err = 0;

    /*I nitialize peripherals (UART) for other threads to use. */
    err = my_uart_init();
    if (err) {
        LOG_ERR("UART init failed (err %d).", err);
        return err;
    }
    LOG_INF("UART init success.");

    /* Start two identical threads so they compete equally for the same shared resource(s). */
    thread_ids[0] = k_thread_create(&threadA_data, threadA_stack_area, /* Thread A. */
                                    K_THREAD_STACK_SIZEOF(threadA_stack_area),
                                    thread_target_fn,
                                    &thread_names[0], NULL, NULL,
                                    THREAD_PRIORITY, 0, K_NO_WAIT);

    thread_ids[1] = k_thread_create(&threadB_data, threadB_stack_area, /* Thread B. */
                                    K_THREAD_STACK_SIZEOF(threadB_stack_area),
                                    thread_target_fn,
                                    &thread_names[1], NULL, NULL,
                                    THREAD_PRIORITY, 0, K_NO_WAIT);

    /* Wait for threads to finish (join), then exit. */
    k_thread_join(thread_ids[0], K_FOREVER);
    k_thread_join(thread_ids[1], K_FOREVER);
    LOG_INF("Threads joined, exiting."); 

	return 0;
}

void thread_target_fn(void *param, void*, void*)
{
    /* Variables below are local (private) to this thread's stack - no race here. */
    char *name = param;
    int loops_left = THREAD_NUM_LOOPS;

    while (loops_left--)
    {
        /* Wait for binary semaphore to be available before attempting to access shared buffer, to avoid race. */
        k_sem_take(&uart_tx_sem, K_FOREVER);

        /* If control reaches here, shared buffer is available to you. Write to it. 
           Note that the line below is the so called "critical section" that we are protecting. */
        memset(uart_tx_buff, *name, sizeof(uart_tx_buff));
        uart_tx_buff[sizeof(uart_tx_buff) - 2] = '\n';
        uart_tx_buff[sizeof(uart_tx_buff) - 1] = '\r';

        /* Below is a non-blocking call - will return immediately.
           Note: should return -EBUSY if there is an on-going transfer already but that case can
           never arise here because we have reached here after taking the binary semaphore which
           could only have been given by the UART callback/isr's UART_TX_DONE event, meaning there
           cannot be an on-going transfer at this point! :) */
        uart_tx(uart, uart_tx_buff, sizeof(uart_tx_buff), 0);

        /* Sleep. */
        k_sleep(THREAD_SLEEP_DURATION);
    }
}

/*
    TODO:
    2. Figure out a UART-TX access policy with regards to 2 operations:
        + Checking if return value of uart_tx is -EBUSY
        + Waiting on a semaphore is busy. 
    3. Add possible more threads to test out the policy and figure out 
       which one works best for which siutation:
        + not sure if possible to do programmatically without compile time macros?
*/