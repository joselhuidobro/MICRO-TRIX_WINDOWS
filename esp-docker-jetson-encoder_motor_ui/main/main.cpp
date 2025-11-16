// app_main.cpp
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cmath>

// ----- ESP-IDF -----
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

// ----- micro-ROS -----
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_espidf_transport.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw/qos_profiles.h>
#include <rmw/types.h>

// ----- TRIX -----
#include "trix_wifi_connection.h"
#include "pid_ctrl.h"
#include "encoder.h"
#include "motor.h"   // ← NUEVO

// ======================= Ajustes =======================
#define CONTROL_HZ                 200   // lazo PID a 200 Hz (5 ms)
#define EXECUTOR_SPIN_US           2000  // executor spin_some ≈ 2 ms
#define TELEMETRY_HZ               10    // telemetría 10 Hz

/* ─ Puente H (posición) ─
   PINS: 21 = IZQ, 22 = DER. Nunca activos a la vez.
*/
#define HBRIDGE_LEFT_PIN           GPIO_NUM_21
#define HBRIDGE_RIGHT_PIN          GPIO_NUM_22
#define HBRIDGE_DEADTIME_US        5       // dead-time al invertir
#define HBRIDGE_DEADBAND_U         0.02f   // banda muerta 2%
#define SETPOINT_EPS_DEG           0.5f    // ±0.5° apaga PWM

/* LED onboard */
#define LED_ONBOARD                GPIO_NUM_2

/* micro-ROS config */
#ifndef UROS_AGENT_DEFAULT_IP
#define UROS_AGENT_DEFAULT_IP "192.168.0.82"   // ← IP LAN del host con Kong
#endif
#ifndef UROS_AGENT_DEFAULT_PORT
#define UROS_AGENT_DEFAULT_PORT "9999"         // ← Kong UDP listener
#endif


/* Encoder */
#define BUTTON_PIN GPIO_NUM_23
#define DIR_PIN1   GPIO_NUM_26
#define DIR_PIN2   GPIO_NUM_27
static constexpr int PPR = 1200;

// Macro helper
#define RCCHECK(fn) { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { ESP_LOGE(TAG, "RCL err %d @%d", (int)_rc, __LINE__); abort(); } }

// ======================= Globals =======================
static const char *TAG = "ROBOT_APP";

// micro-ROS
rcl_node_t            node;
rcl_publisher_t       telem_pub;
std_msgs__msg__String telem_msg;

rclc_executor_t       executor;

// Relevadores (tal cual lo tenías)
#define RELE_LED1  GPIO_NUM_18
#define RELE_LED2  GPIO_NUM_19
volatile int relay1_state = 0;
volatile int relay2_state = 0;

// Subs
rcl_subscription_t        relay_sub;
std_msgs__msg__String     relay_msg;
static char               relay_rx_buf[32];

rcl_subscription_t        setpoint_sub;
std_msgs__msg__Float32    setpoint_msg;

// PID
static pid_ctrl_block_handle_t pid = NULL;
static float setpoint_deg = 0.0f;   // actualizado por sub de ROS

// ======================= PID helpers ===================
static inline void pid_setup()
{
    pid_ctrl_parameter_t p{};
    p.kp = 0.032f;         // afina en campo
    p.ki = 0.30f;
    p.kd = 0.25f;

    p.cal_type     = PID_CAL_TYPE_POSITIONAL;
    p.max_output   = 1.0f;  p.min_output   = -1.0f;
    p.max_integral = 0.5f;  p.min_integral = -0.5f;

    pid_ctrl_config_t cfg{};
    cfg.init_param = p;
    ESP_ERROR_CHECK(pid_new_control_block(&cfg, &pid));
}

static inline float pid_effort(float set_deg, float meas_deg)
{
    float out = 0.0f;
    ESP_ERROR_CHECK(pid_compute(pid, set_deg - meas_deg, &out));
    if (out >  1.0f) out =  1.0f;
    if (out < -1.0f) out = -1.0f;
    return out;
}

// ======================= Callbacks ROS =================
static void relay_callback(const void * msgin)
{
    const auto * msg = (const std_msgs__msg__String *)msgin;
    const char *cmd = (msg && msg->data.data) ? msg->data.data : nullptr;
    ESP_LOGI(TAG, "RX cmd: %s", cmd ? cmd : "(null)");
    if (!cmd) return;

    if      (strcmp(cmd, "relay1_on")  == 0) { gpio_set_level(RELE_LED1, 1); relay1_state = 1; }
    else if (strcmp(cmd, "relay1_off") == 0) { gpio_set_level(RELE_LED1, 0); relay1_state = 0; }
    else if (strcmp(cmd, "relay2_on")  == 0) { gpio_set_level(RELE_LED2, 1); relay2_state = 1; }
    else if (strcmp(cmd, "relay2_off") == 0) { gpio_set_level(RELE_LED2, 0); relay2_state = 0; }
}

static void setpoint_callback(const void * msgin)
{
    const auto * msg = (const std_msgs__msg__Float32 *)msgin;
    const float sp = msg ? msg->data : 0.0f;
    setpoint_deg = sp;
    ESP_LOGI(TAG, "Setpoint: %.2f deg", setpoint_deg);
    // Si tu lib lo soporta: pid_reset_integral(pid);
}

// ======================= Tareas ========================
static void control_pid_task(void* /*arg*/)
{
    pid_setup();

    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
    TickType_t last = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last, period);

        const double degrees = encoder_get_degrees();

        const float err_deg = setpoint_deg - (float)degrees;
        if (fabsf(err_deg) <= SETPOINT_EPS_DEG) {
            motor_coast();  // ← del módulo motor
        } else {
            const float u = pid_effort(setpoint_deg, (float)degrees); // [-1..+1]
            motor_drive_u(u); // ← del módulo motor
        }

        vTaskDelay(1);
    }
}

static void ros_spin_task(void* /*arg*/)
{
    while (true) {
        rclc_executor_spin_some(&executor, EXECUTOR_SPIN_US * 1000); // ns
        // vTaskDelay(1); // opcional
    }
}

// ======================= app_main ======================
extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // Wi-Fi
    trix_wifi_init();
    esp_wifi_set_ps(WIFI_PS_NONE);

    // micro-ROS (transporte UDP)
    const char* agent_ip   = std::getenv("UROS_AGENT_HOST");
    const char* agent_port = std::getenv("UROS_AGENT_PORT");
    if (!agent_ip)   agent_ip   = UROS_AGENT_DEFAULT_IP;
    if (!agent_port) agent_port = UROS_AGENT_DEFAULT_PORT;
    ESP_LOGI(TAG, "uROS Agent: %s:%s", agent_ip, agent_port);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(agent_ip, agent_port, rmw_options));

    rclc_support_t support;
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Nodo y tópico con sufijo MAC
    char node_name[32]  = "Encoder_node";
    char topic_name[48] = "RELAY_telemetry";
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(node_name,  sizeof(node_name),  "Encoder_node_%02X%02X%02X", mac[3], mac[4], mac[5]);
        snprintf(topic_name, sizeof(topic_name), "RELAY_telemetry_%02X%02X%02X", mac[3], mac[4], mac[5]);
    } else {
        ESP_LOGW(TAG, "esp_read_mac() falló, uso nombres por defecto");
    }
    ESP_LOGI(TAG, "Nodo: %s | Tópico: %s", node_name, topic_name);

    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

    // Publisher BEST_EFFORT (telemetría)
    RCCHECK(rclc_publisher_init_best_effort(
        &telem_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        topic_name
    ));

    // QoS robusto para comandos
    rmw_qos_profile_t qos_cmd = rmw_qos_profile_default;
    qos_cmd.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_cmd.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_cmd.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_cmd.depth       = 10;

    rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
    sub_opts.qos = qos_cmd;

    // Suscriptores
    RCCHECK(rcl_subscription_init(
        &relay_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "relevadores", &sub_opts));
    RCCHECK(rcl_subscription_init(
        &setpoint_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "servo_setpoint_deg", &sub_opts));

    // Mensajes
    std_msgs__msg__String__init(&telem_msg);
    std_msgs__msg__String__init(&relay_msg);
    relay_msg.data.data     = relay_rx_buf;
    relay_msg.data.capacity = sizeof(relay_rx_buf);
    relay_msg.data.size     = 0;
    std_msgs__msg__Float32__init(&setpoint_msg);

    // Executor + tarea spin
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &relay_sub,    &relay_msg,    &relay_callback,    ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &setpoint_sub, &setpoint_msg, &setpoint_callback, ON_NEW_DATA));
    xTaskCreatePinnedToCore(ros_spin_task, "ros_spin", 4096, NULL, 5, NULL, 1);

    // LED onboard
    gpio_reset_pin(LED_ONBOARD);
    gpio_set_direction(LED_ONBOARD, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_ONBOARD, 0);

    // Relevadores (como antes)
    gpio_reset_pin(RELE_LED1);
    gpio_reset_pin(RELE_LED2);
    gpio_set_direction(RELE_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELE_LED2, GPIO_MODE_OUTPUT);
    gpio_set_level(RELE_LED1, 0);
    gpio_set_level(RELE_LED2, 0);
    relay1_state = 0; relay2_state = 0;

    // Encoder (módulo ya creado)
    encoder_init(BUTTON_PIN, DIR_PIN1, DIR_PIN2, PPR);

    // ===== Motor (NUEVO: usando el módulo) =====
    MotorConfig mcfg{
        .pin_left        = HBRIDGE_LEFT_PIN,
        .pin_right       = HBRIDGE_RIGHT_PIN,
        .ledc_mode       = LEDC_HIGH_SPEED_MODE,
        .ledc_timer      = LEDC_TIMER_0,
        .ch_left         = LEDC_CHANNEL_0,
        .ch_right        = LEDC_CHANNEL_1,
        .pwm_freq_hz     = 20000,
        .resolution_bits = 10,
        .deadtime_us     = HBRIDGE_DEADTIME_US,
        .deadband_u      = HBRIDGE_DEADBAND_U
    };
    motor_init(mcfg);   // ← configura timer, canales y deja el motor en “coast”

    // Tarea PID (core 1)
    xTaskCreatePinnedToCore(control_pid_task, "control_pid", 4096, NULL, 4, NULL, 1);

    // ===== Telemetría (10 Hz) =====
    const TickType_t telem_period = pdMS_TO_TICKS(1000 / TELEMETRY_HZ);
    TickType_t telem_last = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&telem_last, telem_period);

        const long   p   = encoder_get_pulses();
        const double deg = encoder_get_degrees();

        char telemetry_str[160];
        snprintf(telemetry_str, sizeof(telemetry_str),
                 "Degrees: %.2f, Pulses: %ld, R1:%s, R2:%s",
                 deg, p,
                 relay1_state ? "ON" : "OFF",
                 relay2_state ? "ON" : "OFF");

        rosidl_runtime_c__String__assign(&telem_msg.data, telemetry_str);
        const rcl_ret_t ret = rcl_publish(&telem_pub, &telem_msg, NULL);
        if (ret != RCL_RET_OK) {
            ESP_LOGE(TAG, "Error al publicar telemetría: %d", (int)ret);
        } else {
            ESP_LOGI(TAG, "Telemetría publicada: %s", telemetry_str);
        }
    }
}

