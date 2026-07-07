#pragma once
#include <stdint.h>
typedef struct {
    int presence;
    int motion;
    int body_motion;
    int distance_cm;
    int heart_rate;
    int breath_rate;
    uint32_t frame_count;
    uint32_t checksum_error;
} radar_state_t;
void radar_uart_start(void);
radar_state_t radar_get_state(void);
