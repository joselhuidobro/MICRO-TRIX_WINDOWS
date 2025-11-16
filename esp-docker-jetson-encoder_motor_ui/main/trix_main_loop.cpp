#include "trix_main_loop.h"
#include <esp_timer.h>
#include <esp_log.h>
#include <cmath>
#include <cstdio>  // snprintf
#include "trix_imu.h"
#include "trix_kalman_filter.h"
#include "trix_PID.h"
#include "trix_esc.h"
#include "trix_udp_telemetria.h"

// ---- micro-ROS (C) ----
#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===== Externs creados en app_main (asegúrate de que *sí* existen) =====
extern trix_kalman_axis_t kPitch, kRoll;
extern trix_pid_t pidPitchAng, pidPitchRate, pidRollAng, pidRollRate, pidYawAng, pidYawRate;
extern float setpoint_pitch_deg, setpoint_roll_deg, setpoint_yaw_deg;

extern const char *TAG;

extern rcl_publisher_t       telem_pub;   // creado en app_main
extern std_msgs__msg__String telem_msg;   // inicializado en app_main con std_msgs__msg__String__init

// ===== Config =====
#ifndef TRIX_CONFIG_H
#define TRIX_CONFIG_H
#define THR_MIN  1000
#define THR_MAX  2000
#define THR_BASE 1300
#endif

// ===== Helpers =====
#define RCCHECK(fn) do { \
  rcl_ret_t _rc = (fn); \
  if (_rc != RCL_RET_OK) { \
    ESP_LOGE(TAG, "rcl error %d at %s:%d", (int)_rc, __FILE__, __LINE__); \
  } \
} while (0)

typedef struct { float pitch, roll, yaw; } pid_outputs_t;

static inline float constrain_value(float v, float a, float b) {
  return v < a ? a : (v > b ? b : v);
}

static inline const char* rcl_ret_to_str(rcl_ret_t rc) {
  switch (rc) {
    case RCL_RET_OK: return "RCL_RET_OK";
    case RCL_RET_ERROR: return "RCL_RET_ERROR";
    case RCL_RET_INVALID_ARGUMENT: return "RCL_RET_INVALID_ARGUMENT";
    case RCL_RET_TIMEOUT: return "RCL_RET_TIMEOUT";
    case RCL_RET_NODE_INVALID: return "RCL_RET_NODE_INVALID";
    case RCL_RET_PUBLISHER_INVALID: return "RCL_RET_PUBLISHER_INVALID";
    default: return "RCL_RET_?";
  }
}

// ===== Estado local =====
static uint64_t lastMicros;
static uint16_t printCtr      = 0;
static uint16_t telemetry_cnt = 0;
static uint16_t debug_ctr     = 0;
static float    yawAngleDeg   = 0.0f;

// Flags de micro-ROS
static bool   s_ros_pub_ready     = false;
static bool   s_ros_string_ready  = false;
static size_t s_ros_string_cap    = 0;

// ======= Diagnóstico de ROS2 / micro-ROS =======
typedef struct {
  bool      agent_ok;
  bool      pub_ready;
  rcl_ret_t last_pub_rc;
  uint32_t  pub_ok;
  uint32_t  pub_fail;
  uint64_t  last_pub_ok_us;  // timestamp esp_timer del último publish OK
  uint64_t  last_diag_log_us;
} ros_diag_t;

static ros_diag_t s_ros{};   // C++ value-init: todo en cero/false

// Reserva única para el String de ROS para evitar reallocs en el loop.
// cap_sugerida = 240 (ajústala si cambias el formato).
static void trix_ros2_prepare_string_buffer(size_t cap_sugerida) {
  if (s_ros_string_ready) return;

  if (!telem_msg.data.data) {
    std_msgs__msg__String__init(&telem_msg);
  }

  static char dummy[256];
  size_t cap = (cap_sugerida < sizeof(dummy)-1) ? cap_sugerida : sizeof(dummy)-1;
  for (size_t i = 0; i < cap; ++i) dummy[i] = ' ';
  dummy[cap] = '\0';

  rosidl_runtime_c__String__assignn(&telem_msg.data, dummy, cap);

  s_ros_string_ready = true;
  s_ros_string_cap   = cap;
}

// Chequeo ligero de publisher listo (impl no nulo)
static inline bool trix_ros2_publisher_is_ready(const rcl_publisher_t *pub) {
  return (pub && pub->impl != NULL);
}

static inline bool trix_ros2_agent_is_reachable(int timeout_ms) {
  return (rmw_uros_ping_agent(timeout_ms, 1) == RMW_RET_OK);
}

// Publica String reutilizando la capacidad ya reservada y actualiza diagnóstico
static void trix_ros2_publish_string(const char *buf, size_t len) {
  if (!s_ros_string_ready) trix_ros2_prepare_string_buffer(240);

  if (len > s_ros_string_cap) {
    len = s_ros_string_cap;
    ESP_LOGW(TAG, "ROS msg recortado a %u bytes", (unsigned)len);
  }

  rosidl_runtime_c__String__assignn(&telem_msg.data, buf, len);

  rcl_ret_t rc = rcl_publish(&telem_pub, &telem_msg, NULL);
  s_ros.last_pub_rc = rc;
  if (rc == RCL_RET_OK) {
    s_ros.pub_ok++;
    s_ros.last_pub_ok_us = esp_timer_get_time();
  } else {
    s_ros.pub_fail++;
    // Evita -Werror=address: no cheques la dirección del buffer, usa el flag de RCL
    if (rcl_error_is_set()) {
      const rcl_error_string_t err = rcl_get_error_string();
      ESP_LOGE(TAG, "rcl_publish fallo: %s (%s)", rcl_ret_to_str(rc), err.str);
      rcl_reset_error();
    } else {
      ESP_LOGE(TAG, "rcl_publish fallo: %s", rcl_ret_to_str(rc));
    }
  }
}

// Log periódico de diagnóstico (≈1 Hz)
static void trix_ros2_diag_tick(void) {
  const uint64_t now = esp_timer_get_time();
  if (now - s_ros.last_diag_log_us < 1000000ULL) return;  // 1 s

  const bool prev_agent = s_ros.agent_ok;
  s_ros.agent_ok  = trix_ros2_agent_is_reachable(50);
  s_ros.pub_ready = trix_ros2_publisher_is_ready(&telem_pub);

  if (prev_agent != s_ros.agent_ok) {
    ESP_LOGW(TAG, "micro-ROS agent %s", s_ros.agent_ok ? "ALCANZABLE (UP)" : "NO ALCANZABLE (DOWN)");
  }

  float secs_since_ok = -1.0f;
  if (s_ros.last_pub_ok_us != 0) {
    secs_since_ok = (now - s_ros.last_pub_ok_us) / 1e6f;
  }

  ESP_LOGI(TAG,
    "[ROS2] agent=%s | pub_ready=%s | last_rc=%s | ok=%u fail=%u | last_ok=%.1fs | msg_cap=%u",
    s_ros.agent_ok ? "UP" : "DOWN",
    s_ros.pub_ready ? "READY" : "NULL",
    rcl_ret_to_str(s_ros.last_pub_rc),
    (unsigned)s_ros.pub_ok, (unsigned)s_ros.pub_fail,
    secs_since_ok,
    (unsigned)s_ros_string_cap
  );

  s_ros.last_diag_log_us = now;
}

// =============================== LOOP ===============================
void trix_main_loop(void) {
  lastMicros = esp_timer_get_time();

  // Intenta marcar publisher como listo desde el arranque
  s_ros_pub_ready = trix_ros2_publisher_is_ready(&telem_pub);
  s_ros.pub_ready = s_ros_pub_ready;
  s_ros.last_pub_rc = RCL_RET_OK;

  while (1) {
    // Δt
    uint64_t now = esp_timer_get_time();
    float dt = (now - lastMicros) / 1e6f;
    lastMicros = now;

    uint64_t t_start = esp_timer_get_time();

    float ax, ay, az, gx, gy, gz;
    if (trix_imu_read(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {

      uint64_t t_imu = esp_timer_get_time();

      // ---- Estimación de actitud ----
      float accelRoll  = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / (float)M_PI;
      float accelPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

      trix_kalman_update(&kPitch, accelPitch, gy, dt);
      trix_kalman_update(&kRoll,  accelRoll,  gx, dt);

      float anglePitch = kPitch.angle;
      float angleRoll  = kRoll.angle;

      // yaw integrado simple
      yawAngleDeg += gz * dt;
      if (yawAngleDeg >  180.f) yawAngleDeg -= 360.f;
      if (yawAngleDeg < -180.f) yawAngleDeg += 360.f;

      // ---- Control en cascada ----
      float sp_pitch_rate = trix_pid_update(&pidPitchAng, setpoint_pitch_deg, anglePitch);
      float sp_roll_rate  = trix_pid_update(&pidRollAng,  setpoint_roll_deg,  angleRoll);
      float sp_yaw_rate   = trix_pid_update(&pidYawAng,   setpoint_yaw_deg,   yawAngleDeg);

      pid_outputs_t pid_out = {
        .pitch = trix_pid_update(&pidPitchRate, sp_pitch_rate, gy),
        .roll  = trix_pid_update(&pidRollRate,  sp_roll_rate,  gx),
        .yaw   = trix_pid_update(&pidYawRate,   sp_yaw_rate,   gz)
      };

      // ---- Mixer (X-Quad) ----
      int16_t m1 = (int16_t)constrain_value(THR_BASE + pid_out.roll + pid_out.pitch - pid_out.yaw, THR_MIN, THR_MAX);
      int16_t m2 = (int16_t)constrain_value(THR_BASE + pid_out.roll - pid_out.pitch + pid_out.yaw, THR_MIN, THR_MAX);
      int16_t m3 = (int16_t)constrain_value(THR_BASE - pid_out.roll + pid_out.pitch + pid_out.yaw, THR_MIN, THR_MAX);
      int16_t m4 = (int16_t)constrain_value(THR_BASE - pid_out.roll - pid_out.pitch - pid_out.yaw, THR_MIN, THR_MAX);

      trix_esc_set(1, m1);
      trix_esc_set(2, m2);
      trix_esc_set(3, m3);
      trix_esc_set(4, m4);

      uint64_t t_control = esp_timer_get_time();

      // ---- Logs ligeros (~20 Hz) ----
      if (++printCtr >= 50) {
        printCtr = 0;
        ESP_LOGI(TAG,
          "Ang %.1f/%.1f/%.1f | RateSP %.1f/%.1f/%.1f | PIDout %.1f/%.1f/%.1f | M %d %d %d %d",
          anglePitch, angleRoll, yawAngleDeg,
          sp_pitch_rate, sp_roll_rate, sp_yaw_rate,
          pid_out.pitch, pid_out.roll, pid_out.yaw,
          m1, m2, m3, m4
        );
      }

      // ---- Telemetría (~100/10 = 10 Hz) ----
      if (++telemetry_cnt >= 10) {
        telemetry_cnt = 0;

        // UDP (como ya tenías)
        drone_telemetry_t pkt = { anglePitch, angleRoll, yawAngleDeg, m1, m2, m3, m4 };
        trix_udp_telemetria_send(&pkt);

        // micro-ROS: refresca estado del publisher si aún no listo
        if (!s_ros_pub_ready) {
          s_ros_pub_ready = trix_ros2_publisher_is_ready(&telem_pub);
          s_ros.pub_ready = s_ros_pub_ready;
          if (!s_ros_pub_ready) {
            ESP_LOGW(TAG, "Publisher ROS aún no listo (impl=NULL).");
          }
        }

        if (s_ros_pub_ready) {
          // Formato compacto <= 240 bytes con resumen de estado ROS
          char buffer[240];
          int n = snprintf(buffer, sizeof(buffer),
            "ROS:%s|Pub:%s|ok:%u|fail:%u | Ang %.1f/%.1f/%.1f | RateSP %.1f/%.1f/%.1f | PID %.1f/%.1f/%.1f | M %d %d %d %d",
            s_ros.agent_ok ? "UP" : "DOWN",
            s_ros.pub_ready ? "READY" : "NULL",
            (unsigned)s_ros.pub_ok, (unsigned)s_ros.pub_fail,
            anglePitch, angleRoll, yawAngleDeg,
            sp_pitch_rate, sp_roll_rate, sp_yaw_rate,
            pid_out.pitch, pid_out.roll, pid_out.yaw,
            m1, m2, m3, m4
          );
          size_t len = (n < 0) ? 0 : (size_t)n;
          trix_ros2_publish_string(buffer, len);
        }
      }

      // ---- Perfilado de tiempos (cada ~50 loops) ----
      uint64_t t_end = esp_timer_get_time();
      if (++debug_ctr >= 50) {
        debug_ctr = 0;
        ESP_LOGI(TAG, "Tiempos: IMU=%llu us | Control=%llu us | Total=%llu us",
                 (unsigned long long)(t_imu - t_start),
                 (unsigned long long)(t_control - t_imu),
                 (unsigned long long)(t_end - t_start));
      }
    }

    // Diagnóstico ROS/Agent (≈1 Hz)
    trix_ros2_diag_tick();

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

