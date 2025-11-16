#include <stdio.h>
#include <unistd.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

#include "driver/gpio.h"

#define LED_PIN 2

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    if (msg->data == 1){
        gpio_set_level(LED_PIN, 1);
    } else {
        gpio_set_level(LED_PIN, 0);
    }
}

int appMain(void)
{
    // Init GPIO
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Init micro-ROS
    rmw_uros_set_custom_transport(
    true,
    (void *) &default_xtcp_fd,
    xtcp_transport_open,
    xtcp_transport_close,
    xtcp_transport_write,
    xtcp_transport_read
    );
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Init executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // Init subscriber
    rcl_subscription_t subscriber;
    rclc_subscription_init_default(
    &subscriber,
    &support.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "microROS/blink"
    );
    rclc_executor_add_subscription(&executor, &subscriber, &int32_msg, &subscription_callback, ON_NEW_DATA);

    // Spin
    rclc_executor_spin(&executor);

    // Clean up
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return 0;
}
