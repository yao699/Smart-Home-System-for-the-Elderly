#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void esp8266_at_start(void);
bool esp8266_at_wifi_test(const char *ssid, const char *password);
void esp8266_at_wifi_test_async(const char *ssid, const char *password);

#ifdef __cplusplus
}
#endif
