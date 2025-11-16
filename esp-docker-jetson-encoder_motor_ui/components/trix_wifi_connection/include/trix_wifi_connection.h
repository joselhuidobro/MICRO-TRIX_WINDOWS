#pragma once

#include <stdbool.h>  // ← NECESARIO para usar `bool`

#ifdef __cplusplus
extern "C" {
#endif

void trix_wifi_init(void);
bool trix_wifi_is_connected(void);

#ifdef __cplusplus
}
#endif

