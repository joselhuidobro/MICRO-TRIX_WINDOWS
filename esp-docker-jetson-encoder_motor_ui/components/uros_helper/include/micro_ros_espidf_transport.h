#pragma once
#include <stdint.h>  // ← añade esto

#ifdef __cplusplus
extern "C" {
#endif

void set_microros_wifi_transports(const char* ssid,
                                  const char* passwd,
                                  const char* agent_ip,
                                  uint16_t agent_port);

#ifdef __cplusplus
}
#endif

