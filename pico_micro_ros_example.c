#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
//=================================
#if 0
#include "pico_uart_transports.h"
#else
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "picow_udp_transports.h"

#define VREF        (3.25f)              // Voltage of Vref
#define CONV_FACTOR (VREF / (1 << 12))   // conversion factor
#define TEMP_ADC    (4)                  // ADC number of temperature sensor

extern bool picow_udp_transport_open(struct uxrCustomTransport * transport);
extern bool picow_udp_transport_close(struct uxrCustomTransport * transport);
extern size_t picow_udp_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t picow_udp_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
#endif
//=================================

//=================================
#if 0
const uint LED_PIN = 25;
#else
#endif
//=================================

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    uint16_t raw_data;
    float temp, voltage;

    // read raw data from ADC
    raw_data = adc_read();
    // convert raw to voltage
    voltage = raw_data * CONV_FACTOR;
    // convert voltage to temperature
    temp = 27 - (voltage - 0.706) / 0.001721;
    msg.data = temp;

    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
}

int main()
{
//=================================
#if 0
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
#else
    stdio_init_all();

    // initialize ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);  // set temperature sensor enabled
    adc_select_input(TEMP_ADC);         // temperature sensor selected

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }

    rmw_uros_set_custom_transport(
        false,          // must be false for UDP
        &picow_params,
        picow_udp_transport_open,
        picow_udp_transport_close,
        picow_udp_transport_write,
        picow_udp_transport_read
    );
#endif
//=================================

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Wait for agent successful ping for 1 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 60;
    rcl_ret_t ret = 0;
    uint8_t blink_led = 1;

    int loop = 0;
    for ( ; loop < attempts; loop++) {
        printf("uros_ping_agent: loop[%d]\n", loop);
        ret = rmw_uros_ping_agent(timeout_ms, 1);

//=================================
#if PICO_CYW43_ARCH_POLL
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for WiFi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        sleep_ms(1);
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        sleep_ms(1000);
#endif
//=================================
        if (ret == RCL_RET_OK)
        {
            // Reachable agent, exiting loop.
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            break;
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, blink_led);
        blink_led = blink_led ? 0 : 1;
    }
    if (loop == attempts) {
        // Unreachable agent, exiting program.
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "picow_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "picow_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

//=================================
#if 0
    gpio_put(LED_PIN, 1);
#else
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
#endif
//=================================

    printf("main loop start\n");

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//=================================
#if PICO_CYW43_ARCH_POLL
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for WiFi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        sleep_ms(1);
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        sleep_ms(1000);
#endif
//=================================

    }
//=================================
    cyw43_arch_deinit();
//=================================
    return 0;
}
