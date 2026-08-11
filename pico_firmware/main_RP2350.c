/*
    Raspberry Pi RP2350 MicroROS tests - The ROS Robot Project Research.
    Basic dual-core publisher/subscriber example with FreeRTOS for RP2350.
    Copyright 2025 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2025.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https: www.gnu.org/licenses/>.
*/

#define PRINTF_BUFFER_SIZE  256
#define TASK_STACK_SIZE     1024
#define USE_UART_TRANSPORT  0   // Set to 1 to use UART transport, 0 for USB transport.
#define ENABLE_BNO055        0  // Set to 1 to compile in BNO055 I2C code
#define BNO055_VALIDATE_ONLY 0  // Set to 1 to run a standalone chip-ID check at boot, before micro-ROS starts



// ---- Libraries ----
#include <stdio.h>
#include <random>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transport.c"
#include "uros_allocators.c"
#include "pico/stdio/driver.h"
#include <stdarg.h>
#include <ctime>
#include <hardware/watchdog.h>

//Motor + Encoder
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#define WHEEL_COUNT 4
enum { FL = 0, FR = 1, BL = 2, BR = 3 };

static const int PIN_PWM[WHEEL_COUNT]   = { 16, -1, -1, -1 };
static const int PIN_DIR[WHEEL_COUNT]   = { 17, -1, -1, -1 };
static const int PIN_ENC_A[WHEEL_COUNT] = { 3,  -1, -1, -1 };
static const int PIN_ENC_B[WHEEL_COUNT] = { 4,  -1, -1, -1 };


//IMU
#if ENABLE_BNO055
#include "hardware/i2c.h"

#define I2C_PORT      i2c0
#define I2C_SDA_PIN   4     // TODO: your actual GPIO
#define I2C_SCL_PIN   5     // TODO: your actual GPIO
#define BNO055_ADDR   0x28  // 0x29 if COM3 pin pulled high
#define BNO055_REG_CHIP_ID  0x00
#define BNO055_REG_OPR_MODE 0x3D
#define BNO055_REG_QUAT     0x20
#define BNO055_MODE_NDOF    0x0C
#endif





// ---- Logging ----
void log_printf(const char *format, ...) {
    char buffer[PRINTF_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    
    int length = vsnprintf(buffer, PRINTF_BUFFER_SIZE, format, args);
    if (length > 0) {
        #if USE_UART_TRANSPORT
            stdio_usb.out_chars(buffer, length);
        #else
            stdio_uart.out_chars(buffer, length);
        #endif
    }
    
    va_end(args);
}

#define LOG_DEBUG(format, ...) log_printf("[DEBUG][%s:%d] " format "\r\n", __func__, __LINE__, ##__VA_ARGS__)


// ---- Return Checkers ----
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)) { \
        LOG_DEBUG("Failed status (RCL) with error code %d", (int)temp_rc); \
        return false; \
    } \
}

#define RMWCHECK(fn) { \
    rmw_ret_t temp_rc = fn; \
    if((temp_rc != RMW_RET_OK)) { \
        LOG_DEBUG("Failed status (RMW) with error code %d", (int)temp_rc); \
        return false; \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)) { \
        LOG_DEBUG("Non-critical failure (RCL) with error code %d", (int)temp_rc); \
    } \
}


// ---- Global Variables ----

// Agent state enum
enum AGENT_STATE {
    WAITING_FOR_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

// RCL variables
rcl_publisher_t publisher_encoders, publisher_imu;
rcl_subscription_t subscriber_wheel_cmd;
std_msgs__msg__Int32MultiArray msg_encoders;
std_msgs__msg__Float32MultiArray msg_wheel_cmd;
sensor_msgs__msg__Imu msg_imu;
rclc_executor_t executor_core0, executor_core1;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// FreeRTOS task handles
TaskHandle_t task_handle_core0 = NULL, task_handle_core1 = NULL, task_handle_uros_state = NULL;


// ---- Functions ----
//Still need to implement against actual motor driver -- PWM + DIR pins
void set_motor_duty(int wheel_index, float duty) {
    LOG_DEBUG("Motor %d duty -> %.2f", wheel_index, duty);
}



#if ENABLE_BNO055
void init_i2c() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

bool bno055_read_reg(uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_blocking(I2C_PORT, BNO055_ADDR, &reg, 1, true) < 0) return false;
    if (i2c_read_blocking(I2C_PORT, BNO055_ADDR, buf, len, false) < 0) return false;
    return true;
}

bool bno055_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(I2C_PORT, BNO055_ADDR, buf, 2, false) >= 0;
}

// Standalone validation: read chip ID register, should return 0xA0
bool bno055_check_chip_id() {
    uint8_t id = 0;
    if (!bno055_read_reg(BNO055_REG_CHIP_ID, &id, 1)) {
        LOG_DEBUG("BNO055: I2C read failed (no ACK / wiring issue?)");
        return false;
    }
    LOG_DEBUG("BNO055: chip ID = 0x%02X (expected 0xA0)", id);
    return id == 0xA0;
}

bool bno055_init() {
    if (!bno055_check_chip_id()) return false;
    if (!bno055_write_reg(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF)) {
        LOG_DEBUG("BNO055: failed to set NDOF mode");
        return false;
    }
    sleep_ms(20);  // mode-switch settle time per datasheet
    return true;
}

bool bno055_read_quaternion(float *w, float *x, float *y, float *z) {
    uint8_t buf[8];
    if (!bno055_read_reg(BNO055_REG_QUAT, buf, 8)) return false;

    int16_t raw_w = (buf[1] << 8) | buf[0];
    int16_t raw_x = (buf[3] << 8) | buf[2];
    int16_t raw_y = (buf[5] << 8) | buf[4];
    int16_t raw_z = (buf[7] << 8) | buf[6];

    const float scale = 1.0f / (1 << 14);
    *w = raw_w * scale;
    *x = raw_x * scale;
    *y = raw_y * scale;
    *z = raw_z * scale;
    return true;
}
#endif  // ENABLE_BNO055

static volatile int32_t encoder_counts[WHEEL_COUNT] = {0};

static void motor_init(void) {
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (PIN_PWM[i] < 0) continue; //Skip unassignments
        gpio_set_function(PIN_PWM[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(PIN_PWM[i]);
        pwm_set_wrap(slice, 1000); // Set PWM frequency (1 kHz)
        pwm_set_enabled(slice, true); 
        gpio_init(PIN_DIR[i]);
        gpio_set_dir(PIN_DIR[i], true); // Set direction pin as output
    }
}

static void motor_set(int wheel, float duty) {
    if (PIN_PWM[wheel] < 0) return; //Skip unassignments
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    gpio_put(PIN_DIR[wheel], duty < 0);
    uint slice = pwm_gpio_to_slice_num(PIN_PWM[wheel]);
    uint chan = pwm_gpio_to_channel(PIN_PWM[wheel]);
    pwm_set_chan_level(slice, chan, (uint16_t)(fabsf(duty) * 1000));
}

static void encoder_irq_handler(uint gpio, uint32_t events) {
    // Handles Encoder Interrup
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
        if (PIN_ENC_A[i] < 0) continue; //Skip unassignments
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

void wheel_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size < WHEEL_COUNT) return;  // Not enough data
    for (int i = 0; i < WHEEL_COUNT; i++) {
        motor_set(i, msg->data.data[i]);
    }
}

bool init_microros() {
    LOG_DEBUG("Initializing MicroROS...");

    allocator = rcl_get_default_allocator();
    LOG_DEBUG("Allocator initialized.");

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "pico_node", "", &support));
    LOG_DEBUG("ROS node and context initialized successfully.");

    // ---- Message memory allocation ----
    std_msgs__msg__Int32MultiArray__init(&msg_encoders);
    if (!rosidl_runtime_c__int32__Sequence__init(&msg_encoders.data, 4)) {
        LOG_DEBUG("Failed to allocate wheel_encoders message array!");
        return false;
    }

    std_msgs__msg__Float32MultiArray__init(&msg_wheel_cmd);
    if (!rosidl_runtime_c__float32__Sequence__init(&msg_wheel_cmd.data, 4)) {
        LOG_DEBUG("Failed to allocate wheel_cmd message array!");
        return false;
    }

    sensor_msgs__msg__Imu__init(&msg_imu);
    rosidl_runtime_c__String__assign(&msg_imu.header.frame_id, "imu_link");
    LOG_DEBUG("Message memory allocated.");

    // ---- Executors ----
    // Core 0: 1 handle (wheel_cmd subscription)
    RCCHECK(rclc_executor_init(&executor_core0, &support.context, 1, &allocator));
    // Core 1: no subscriptions, IMU is publish-only
    RCCHECK(rclc_executor_init(&executor_core1, &support.context, 0, &allocator));
    LOG_DEBUG("Executors initialized successfully.");

    // ---- Publishers ----
    const rosidl_message_type_support_t *encoders_type_supp = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray);
    const rosidl_message_type_support_t *imu_type_supp = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    const rosidl_message_type_support_t *wheel_cmd_type_supp = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);

    RCCHECK(rclc_publisher_init_default(&publisher_encoders, &node, encoders_type_supp, "wheel_encoders"));
    RCCHECK(rclc_publisher_init_default(&publisher_imu, &node, imu_type_supp, "imu/data"));
    LOG_DEBUG("Publishers created successfully.");

    // ---- Subscriptions ----
    RCCHECK(rclc_subscription_init_default(&subscriber_wheel_cmd, &node, wheel_cmd_type_supp, "wheel_cmd"));
    LOG_DEBUG("Subscription created successfully.");

    RCCHECK(rclc_executor_add_subscription(&executor_core0, &subscriber_wheel_cmd, &msg_wheel_cmd, &wheel_cmd_callback, ON_NEW_DATA));
    LOG_DEBUG("Subscription added to executor successfully.");

    return true;
}

bool ping_agent() {
    return rmw_uros_ping_agent(100, 10) == RMW_RET_OK;
}

void reset() {
    watchdog_disable();
    watchdog_enable(1, true);
    while(1);  // Wait for the watchdog to reset the system.
}


// ---- FreeRTOS Hook Callbacks ----
void vApplicationMallocFailedHook(void) {
    LOG_DEBUG("ERROR: Malloc failed! Resetting...");
    configASSERT(false);
    reset();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    LOG_DEBUG("ERROR: Stack overflow in task: '%s'! Resetting...", pcTaskName);
    configASSERT(false);
    reset();
}


// ---- Task Functions ----
void task_core0(void *params) {
    LOG_DEBUG("Core %d task starting (motors/encoders)...", get_core_num());

    while (true) {
        for (int i = 0; i < WHEEL_COUNT; i++) {
            msg_encoders.data.data[i] = encoder_counts[i];
        }

        rcl_ret_t pub_ret = rcl_publish(&publisher_encoders, &msg_encoders, NULL);
        if (pub_ret != RCL_RET_OK) {
            LOG_DEBUG("Core %d failed to publish encoders, error: %d", get_core_num(), (int)pub_ret);
        }

        RCSOFTCHECK(rclc_executor_spin_some(&executor_core0, RCL_MS_TO_NS(20)));
        vTaskDelay(pdMS_TO_TICKS(20));  // ~50 Hz
    }
}

void task_core1(void *params) {
    LOG_DEBUG("Core %d task starting (IMU)...", get_core_num());

    while (true) {
        #if ENABLE_BNO055 && !BNO055_VALIDATE_ONLY
            if (bno055_read_quaternion(&msg_imu.orientation.w, &msg_imu.orientation.x,
                                        &msg_imu.orientation.y, &msg_imu.orientation.z)) {
                msg_imu.orientation_covariance[0] = 0.01;
            } else {
                LOG_DEBUG("Failed to read IMU");
                msg_imu.orientation_covariance[0] = -1.0;
            }
        #else
            msg_imu.orientation.w = 1.0;
            msg_imu.orientation.x = 0.0;
            msg_imu.orientation.y = 0.0;
            msg_imu.orientation.z = 0.0;
            msg_imu.orientation_covariance[0] = -1.0;
        #endif

        rcl_ret_t pub_ret = rcl_publish(&publisher_imu, &msg_imu, NULL);
        if (pub_ret != RCL_RET_OK) {
            LOG_DEBUG("Core %d failed to publish IMU, error: %d", get_core_num(), (int)pub_ret);
        }

        RCSOFTCHECK(rclc_executor_spin_some(&executor_core1, RCL_MS_TO_NS(20)));
        vTaskDelay(pdMS_TO_TICKS(50));  // ~20 Hz
    }
}

#if ENABLE_BNO055 && BNO055_VALIDATE_ONLY
    init_i2c();
    LOG_DEBUG("Running BNO055 standalone validation...");
    if (bno055_init()) {
        LOG_DEBUG("BNO055 init OK. Reading quaternion for 10 seconds...");
        for (int i = 0; i < 100; i++) {
            float w, x, y, z;
            if (bno055_read_quaternion(&w, &x, &y, &z)) {
                LOG_DEBUG("Quat: w=%.3f x=%.3f y=%.3f z=%.3f", w, x, y, z);
            } else {
                LOG_DEBUG("Quaternion read failed");
            }
            sleep_ms(100);
        }
    }
    LOG_DEBUG("Validation complete. Halting (not starting micro-ROS).");
    while (true) { tight_loop_contents(); }
#endif

void uros_state_task(void *params) {
    AGENT_STATE current_agent_state = WAITING_FOR_AGENT;
    LOG_DEBUG("Waiting for agent...");

    while (true) {
        switch (current_agent_state) {
            case WAITING_FOR_AGENT:
                if (ping_agent()) {
                    current_agent_state = AGENT_AVAILABLE;
                }
                break;

            case AGENT_AVAILABLE:
                LOG_DEBUG("Agent available!");

                if (init_microros()) {
                    current_agent_state = AGENT_CONNECTED;
                    LOG_DEBUG("MicroROS initialized successfully.");

                    BaseType_t task0_created = xTaskCreate(task_core0, "core0_task", TASK_STACK_SIZE, NULL, 1, &task_handle_core0);
                    BaseType_t task1_created = xTaskCreate(task_core1, "core1_task", TASK_STACK_SIZE, NULL, 1, &task_handle_core1);
                    vTaskCoreAffinitySet(task_handle_core0, (1 << 0));
                    vTaskCoreAffinitySet(task_handle_core1, (1 << 1));

                    if (task0_created != pdPASS || task1_created != pdPASS) {
                        LOG_DEBUG("Failed to create tasks!");
                        current_agent_state = AGENT_DISCONNECTED;
                    }
                } else {
                    current_agent_state = AGENT_DISCONNECTED;
                }
                break;

            case AGENT_CONNECTED:
                if (!ping_agent()) {
                    current_agent_state = AGENT_DISCONNECTED;
                    LOG_DEBUG("Agent disconnected!");
                }
                break;

            case AGENT_DISCONNECTED:
                LOG_DEBUG("Cleaning up resources and preparing for reset...");
                
                if (task_handle_core0 != NULL) {
                    vTaskDelete(task_handle_core0);
                    task_handle_core0 = NULL;
                }

                if (task_handle_core1 != NULL) {
                    vTaskDelete(task_handle_core1);
                    task_handle_core1 = NULL;
                }

                portDISABLE_INTERRUPTS();
                vTaskSuspendAll();
                LOG_DEBUG("Scheduler suspended.");

                // Here we have to set-up a watchdog timer to reset the system if any of the
                // _fini() calls hang for more than 1000ms. This is because these functions will
                // hang if they are called after the trasnport has disconnected from the micro-ROS agent.
                watchdog_enable(1000, true);

                RCSOFTCHECK(rcl_publisher_fini(&publisher_encoders, &node)); watchdog_update();
                RCSOFTCHECK(rcl_publisher_fini(&publisher_imu, &node)); watchdog_update();
                RCSOFTCHECK(rcl_subscription_fini(&subscriber_wheel_cmd, &node)); watchdog_update();
                RCSOFTCHECK(rclc_executor_fini(&executor_core0)); watchdog_update();
                RCSOFTCHECK(rclc_executor_fini(&executor_core1)); watchdog_update();
                RCSOFTCHECK(rcl_node_fini(&node)); watchdog_update();
                RCSOFTCHECK(rclc_support_fini(&support)); watchdog_update();
                
                LOG_DEBUG("Cleanup completed, resetting...");
                reset();
        }
    
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// ---- Main Function ----
int main() 
{
    stdio_usb_init();
    while (!stdio_usb_connected());
    stdio_uart_init();

    #if USE_UART_TRANSPORT
        stdio_filter_driver(&stdio_uart);
    #else
        stdio_filter_driver(&stdio_usb);
    #endif

    if (cyw43_arch_init()) {
        LOG_DEBUG("Failed to initialize CYW43 Wi-Fi chip!");
        while (true) { tight_loop_contents(); }
    }

    LOG_DEBUG("STDIO initialized.");

    srand(to_us_since_boot(get_absolute_time()));

    rcl_allocator_t rtos_allocators = rcutils_get_zero_initialized_allocator();
    rtos_allocators.allocate = uros_rtos_allocate;
    rtos_allocators.deallocate = uros_rtos_deallocate;
    rtos_allocators.reallocate = uros_rtos_reallocate;
    rtos_allocators.zero_allocate = uros_rtos_zero_allocate;
    
    if (!rcutils_set_default_allocator(&rtos_allocators)) {
        LOG_DEBUG("Failed to set allocators!");
        return 0;
    }

    RMWCHECK(rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    ));

    BaseType_t task_created = xTaskCreate(uros_state_task, "uros_state_task", TASK_STACK_SIZE, NULL, 1, &task_handle_uros_state);
    vTaskCoreAffinitySet(task_handle_uros_state, (1 << 0));

    if (task_created != pdPASS) {
        LOG_DEBUG("Failed to create MicroROS state task!");
    } else {
        LOG_DEBUG("Starting FreeRTOS scheduler...");
        motor_init();
        encoders_init();
        vTaskStartScheduler();
    }

    return 0;
}