#include "radar_uart.h"
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "RADAR"
#define RADAR_UART_PORT   UART_NUM_2
#define RADAR_UART_TX_PIN GPIO_NUM_11
#define RADAR_UART_RX_PIN GPIO_NUM_12
#define RADAR_UART_BAUD   115200
#define RX_BUF_SIZE       2048
#define DATA_MAX          128

static radar_state_t g = {
    .presence = -1, .motion = -1, .body_motion = -1,
    .distance_cm = -1, .heart_rate = -1, .breath_rate = -1,
};

static const char *presence_str(int v) {
    if (v == 0) return "nobody";
    if (v == 1) return "somebody";
    return "unknown";
}

static const char *motion_str(int v) {
    if (v == 0) return "none";
    if (v == 1) return "still";
    if (v == 2) return "active";
    return "unknown";
}

static uint16_t be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

radar_state_t radar_get_state(void) {
    return g;
}

static void print_summary(void) {
    ESP_LOGI(TAG,
             "SUMMARY frames=%" PRIu32 " err=%" PRIu32
             " presence=%s motion=%s body=%d distance=%dcm heart=%d breath=%d",
             g.frame_count, g.checksum_error, presence_str(g.presence),
             motion_str(g.motion), g.body_motion, g.distance_cm,
             g.heart_rate, g.breath_rate);
}

static void parse_frame(uint8_t ctrl, uint8_t cmd, const uint8_t *data, uint16_t len) {
    g.frame_count++;

    if (ctrl == 0x80) {
        if (cmd == 0x01 && len >= 1) {
            g.presence = data[0];
            ESP_LOGI(TAG, "PRESENCE: %s (%u)", presence_str(g.presence), data[0]);
        } else if (cmd == 0x02 && len >= 1) {
            g.motion = data[0];
            ESP_LOGI(TAG, "MOTION: %s (%u)", motion_str(g.motion), data[0]);
        } else if (cmd == 0x03 && len >= 1) {
            g.body_motion = data[0];
            ESP_LOGI(TAG, "BODY_MOTION: %d / 100", g.body_motion);
        } else if (cmd == 0x04 && len >= 2) {
            g.distance_cm = be16(data);
            ESP_LOGI(TAG, "DISTANCE: %d cm", g.distance_cm);
        }
    } else if (ctrl == 0x81) {
        if (cmd == 0x02 && len >= 1) {
            g.breath_rate = data[0];
            ESP_LOGI(TAG, "BREATH_RATE: %d / min", g.breath_rate);
        }
    } else if (ctrl == 0x85) {
        if (cmd == 0x02 && len >= 1) {
            g.heart_rate = data[0];
            ESP_LOGI(TAG, "HEART_RATE: %d / min", g.heart_rate);
        }
    } else if (ctrl == 0x84) {
        ESP_LOGI(TAG, "SLEEP frame cmd=0x%02X len=%u", cmd, len);
    }

    if ((g.frame_count % 10) == 0) {
        print_summary();
    }
}

static void send_cmd(const uint8_t *cmd, int len, const char *name) {
    ESP_LOGI(TAG, "SEND %s", name);
    uart_write_bytes(RADAR_UART_PORT, (const char *)cmd, len);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void send_enable_cmds(void) {
    static const uint8_t en_presence[] = {0x53,0x59,0x80,0x00,0x00,0x01,0x01,0x2E,0x54,0x43};
    static const uint8_t en_breath[]   = {0x53,0x59,0x81,0x00,0x00,0x01,0x01,0x2F,0x54,0x43};
    static const uint8_t en_heart[]    = {0x53,0x59,0x85,0x00,0x00,0x01,0x01,0x33,0x54,0x43};
    static const uint8_t en_sleep[]    = {0x53,0x59,0x84,0x00,0x00,0x01,0x01,0x32,0x54,0x43};
    send_cmd(en_presence, sizeof(en_presence), "enable presence");
    send_cmd(en_breath, sizeof(en_breath), "enable breath");
    send_cmd(en_heart, sizeof(en_heart), "enable heart");
    send_cmd(en_sleep, sizeof(en_sleep), "enable sleep");
}

static void parser_task(void *arg) {
    uint8_t rx[128], data[DATA_MAX];
    enum { W53, W59, CTRL, CMD, LENH, LENL, DATA, SUM, T54, T43 } st = W53;
    uint8_t ctrl = 0, cmd = 0, len_h = 0, len_l = 0, sum = 0;
    uint16_t data_len = 0;
    int data_pos = 0;
    int64_t last_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Radar parser started. Waiting for R60ABD1 frames...");

    while (1) {
        int n = uart_read_bytes(RADAR_UART_PORT, rx, sizeof(rx), pdMS_TO_TICKS(200));
        for (int i = 0; i < n; i++) {
            uint8_t b = rx[i];
            switch (st) {
            case W53:
                if (b == 0x53) { sum = b; st = W59; }
                break;
            case W59:
                if (b == 0x59) { sum += b; st = CTRL; } else st = W53;
                break;
            case CTRL:
                ctrl = b; sum += b; st = CMD; break;
            case CMD:
                cmd = b; sum += b; st = LENH; break;
            case LENH:
                len_h = b; sum += b; st = LENL; break;
            case LENL:
                len_l = b; sum += b; data_len = ((uint16_t)len_h << 8) | len_l; data_pos = 0;
                if (data_len > DATA_MAX) st = W53;
                else st = data_len ? DATA : SUM;
                break;
            case DATA:
                data[data_pos++] = b; sum += b;
                if (data_pos >= data_len) st = SUM;
                break;
            case SUM:
                if (b != sum) { g.checksum_error++; st = W53; }
                else st = T54;
                break;
            case T54:
                st = (b == 0x54) ? T43 : W53;
                break;
            case T43:
                if (b == 0x43) parse_frame(ctrl, cmd, data, data_len);
                st = W53;
                break;
            default:
                st = W53;
                break;
            }
        }

        int64_t now = esp_timer_get_time();
        if (now - last_us > 5000000LL) {
            print_summary();
            last_us = now;
        }
    }
}

void radar_uart_start(void) {
    static bool started = false;
    if (started) return;
    started = true;

    ESP_LOGI(TAG, "R60ABD1 UART start");
    ESP_LOGI(TAG, "UART2 TX=GPIO11 RX=GPIO12 baud=115200");
    ESP_LOGI(TAG, "Wiring: radar TX -> GPIO12, radar RX -> GPIO11, 5V/GND common");

    uart_config_t cfg = {
        .baud_rate = RADAR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_PORT, RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RADAR_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_PORT, RADAR_UART_TX_PIN, RADAR_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vTaskDelay(pdMS_TO_TICKS(500));
    send_enable_cmds();
    xTaskCreate(parser_task, "radar_parser", 4096, NULL, 9, NULL);
}
