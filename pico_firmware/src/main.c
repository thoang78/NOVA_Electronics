/*
 * Pico 2 micro-ROS firmware
 * -------------------------
 * - Publishes IMU data + 4x wheel encoder counts
 * - Subscribes to 4x wheel motor commands + 2x fan commands
 * - Drives 2x Cytron MDD10A dual-channel motor drivers (4 motors total)
 *
 * FILL IN YOUR REAL GPIO NUMBERS BELOW before building.
 * Board pin numbers on your diagram != GPIO numbers -- check the Pico 2 pinout.
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>

// ---------------------------------------------------------------------
// ===================== EDIT THIS BLOCK ================================
// GPIO numbers -- NOT board pin numbers. Replace -1 placeholders.
// ---------------------------------------------------------------------
#define WHEEL_COUNT 4
enum { FL = 0, FR = 1, BL = 2, BR = 3 };
// Front Cytron driver (from diagram: PWM1 DIR1 PWM2 DIR2 -> FL, FR)
static const int PIN_PWM[WHEEL_COUNT] = { 16, -1, -1, -1 }; // FL, FR, BL, BR
static const int PIN_DIR[WHEEL_COUNT] = { 17, -1, -1, -1 };

// Quadrature encoder A/B channels per wheel
static const int PIN_ENC_A[WHEEL_COUNT] = { 1, -1, -1, -1 };
static const int PIN_ENC_B[WHEEL_COUNT] = { 2, -1, -1, -1 };

// Fans (simple PWM outputs)
//static const int PIN_FAN[2] = { -1, -1 };

// IMU I2C bus (BNO085: SDA/SCL per diagram Pin26/Pin27) --> leave as -1 for now, don't have IMU yet
#define IMU_I2C_INSTANCE i2c1
static const int PIN_IMU_SDA = -1;
static const int PIN_IMU_SCL = -1;


#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

static volatile int32_t encoder_counts[WHEEL_COUNT] = {0};

static rcl_publisher_t encoder_pub;
static rcl_publisher_t imu_pub;
static rcl_subscription_t wheel_cmd_sub;
static rcl_subscription_t fan_cmd_sub;

static std_msgs__msg__Int32MultiArray encoder_msg;
static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__Float32MultiArray wheel_cmd_msg;
static std_msgs__msg__Float32MultiArray fan_cmd_msg;

static void error_loop(void) {
    while (1) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }
}

// Motor output
static void motor_init(void) {
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (PIN_PWM[i] < 0) continue; // not yet configured -- skip safely
        gpio_set_function(PIN_PWM[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(PIN_PWM[i]);
        pwm_set_wrap(slice, 1000);
        pwm_set_enabled(slice, true);

        gpio_init(PIN_DIR[i]);
        gpio_set_dir(PIN_DIR[i], GPIO_OUT);
    }
}

// duty: -1.0 .. 1.0
static void motor_set(int wheel, float duty) {
    if (PIN_PWM[wheel] < 0) return;
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    gpio_put(PIN_DIR[wheel], duty < 0);
    uint slice = pwm_gpio_to_slice_num(PIN_PWM[wheel]);
    uint chan = pwm_gpio_to_channel(PIN_PWM[wheel]);
    pwm_set_chan_level(slice, chan, (uint16_t)(fabsf(duty) * 1000));
}


// Quadrature encoders (shared IRQ handler, decode on edge of channel A)
static void encoder_irq_handler(uint gpio, uint32_t events) {
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (gpio == (uint)PIN_ENC_A[i]) {
            bool a = gpio_get(PIN_ENC_A[i]);
            bool b = gpio_get(PIN_ENC_B[i]);
            encoder_counts[i] += (a == b) ? 1 : -1;
            return;
        }
    }
}

static void encoders_init(void) {
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (PIN_ENC_A[i] < 0) continue;
        gpio_init(PIN_ENC_A[i]);
        gpio_set_dir(PIN_ENC_A[i], GPIO_IN);
        gpio_pull_up(PIN_ENC_A[i]);
        gpio_init(PIN_ENC_B[i]);
        gpio_set_dir(PIN_ENC_B[i], GPIO_IN);
        gpio_pull_up(PIN_ENC_B[i]);

        gpio_set_irq_enabled_with_callback(
            PIN_ENC_A[i], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
            true, &encoder_irq_handler);
    }
}

// ---------------------------------------------------------------------
// IMU -- STUB. See README.md "TODO -- IMU driver".
// ---------------------------------------------------------------------
static void imu_read_stub(sensor_msgs__msg__Imu *msg) {
    msg->orientation.w = 1.0;
    msg->orientation.x = 0.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = 0.0;
    msg->angular_velocity.x = 0.0;
    msg->angular_velocity.y = 0.0;
    msg->angular_velocity.z = 0.0;
    msg->linear_acceleration.x = 0.0;
    msg->linear_acceleration.y = 0.0;
    msg->linear_acceleration.z = 9.81;
}

// ---------------------------------------------------------------------
// ROS callbacks
// ---------------------------------------------------------------------
static void wheel_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size < WHEEL_COUNT) return;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        motor_set(i, msg->data.data[i]);
    }
}

static void fan_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size < 2) return;
    for (int i = 0; i < 2; i++) {
        if (PIN_FAN[i] < 0) continue;
        uint slice = pwm_gpio_to_slice_num(PIN_FAN[i]);
        uint chan = pwm_gpio_to_channel(PIN_FAN[i]);
        float d = msg->data.data[i];
        if (d < 0) d = 0; if (d > 1) d = 1;
        pwm_set_chan_level(slice, chan, (uint16_t)(d * 1000));
    }
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        encoder_msg.data.data[i] = encoder_counts[i];
    }
    RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));

    imu_read_stub(&imu_msg);
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

// ---------------------------------------------------------------------
int main(void) {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    motor_init();
    encoders_init();

    for (int i = 0; i < 2; i++) {
        if (PIN_FAN[i] < 0) continue;
        gpio_set_function(PIN_FAN[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(PIN_FAN[i]);
        pwm_set_wrap(slice, 1000);
        pwm_set_enabled(slice, true);
    }

    if (PIN_IMU_SDA >= 0) {
        i2c_init(IMU_I2C_INSTANCE, 400 * 1000);
        gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
        gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(PIN_IMU_SDA);
        gpio_pull_up(PIN_IMU_SCL);
    }

    // --- micro-ROS transport over USB ---
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open, pico_serial_transport_close,
        pico_serial_transport_write, pico_serial_transport_read
    );
    // NOTE: pico_serial_transport_* comes from the micro_ros_raspberrypi_pico_sdk
    // examples' pico_uart_transport.c/.h -- copy that file in alongside main.c.
    // See README.md build step for where this repo lives.

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "pico_hw_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &encoder_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "wheel_encoders"));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    RCCHECK(rclc_subscription_init_default(
        &wheel_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_cmd"));

    RCCHECK(rclc_subscription_init_default(
        &fan_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "fan_cmd"));

    // static allocation for the encoder message payload (4 wheels)
    static int32_t encoder_data[WHEEL_COUNT];
    encoder_msg.data.data = encoder_data;
    encoder_msg.data.size = WHEEL_COUNT;
    encoder_msg.data.capacity = WHEEL_COUNT;

    static float wheel_cmd_data[WHEEL_COUNT];
    wheel_cmd_msg.data.data = wheel_cmd_data;
    wheel_cmd_msg.data.capacity = WHEEL_COUNT;

    static float fan_cmd_data[2];
    fan_cmd_msg.data.data = fan_cmd_data;
    fan_cmd_msg.data.capacity = 2;

    rcl_timer_t timer;
    const unsigned int timer_period_ms = 20; // 50 Hz
    RCCHECK(rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(timer_period_ms), timer_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &wheel_cmd_sub, &wheel_cmd_msg, &wheel_cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &fan_cmd_sub, &fan_cmd_msg, &fan_cmd_callback, ON_NEW_DATA));

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }

    return 0;
}
