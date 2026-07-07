#include "esp8266_at.h"
#include "radar_uart.h"
#include "mipi_cam.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#define TAG "ESP8266"

#define AISTUDIO_HOST          "aistudio.baidu.com"
#define AISTUDIO_PORT          443
#define AISTUDIO_PATH          "/llm/lmapi/v3/chat/completions"
#define AISTUDIO_MODEL         "ernie-4.0-turbo-8k-latest"
#define AISTUDIO_VL_MODEL      "ernie-4.5-turbo-vl"

// Replace the following placeholder with your own AI Studio Access Token.
// Do not send the real token to others or show it in screenshots.
#define AISTUDIO_ACCESS_TOKEN  ""  // 你的API_TOKEN

static bool esp8266_at_ai_analysis_test(void);
static bool esp8266_at_ai_realtime_snapshot_analysis(void);
static void ai_demo_key_start_once(void);
static void ai_wait_next_cycle_or_key(void);
static void list_sdcard_root_for_debug(void);
static bool get_file_size_bytes(const char *path, size_t *out_size);


// ESP32-P4 UART1 <-> ATK-MW8266D / ESP8266 AT
#define ESP_AT_UART_PORT       UART_NUM_1
#define ESP_AT_TX_PIN          GPIO_NUM_26
#define ESP_AT_RX_PIN          GPIO_NUM_27
#define ESP_AT_BAUD            115200
#define ESP_AT_RX_BUF_SIZE     4096

#define AI_ANALYSIS_INTERVAL_MS 10000
#define AI_DEMO_FALL_FILE       "/sdcard/DEMO_FALL.JPG"
#define AI_DEMO_FALL_FILE_83    "/sdcard/DEMO_F~1.JPG"
#define AI_DEMO_FALL_FILE_ALT   "/sdcard/FALL.JPG"
#define AI_DEMO_KEY_GPIO        GPIO_NUM_35
#define AI_DEMO_KEY_ACTIVE      0
#define AI_SEND_CHUNK_SIZE      1024
#define AI_DEMO_IMAGE_MAX_BYTES (220 * 1024)

#define AI_REAL_SNAPSHOT_1      "/sdcard/AI_1.JPG"
#define AI_REAL_SNAPSHOT_2      "/sdcard/AI_2.JPG"
#define AI_REAL_SNAPSHOT_3      "/sdcard/AI_3.JPG"
#define AI_REAL_SNAPSHOT_WAIT_MS 10000

static bool s_started = false;
static volatile bool s_ai_demo_fall_mode = false;
static volatile bool s_ai_force_analysis_now = false;
static TaskHandle_t s_ai_demo_key_task_handle = NULL;

void esp8266_at_start(void)
{
    if (s_started) {
        ESP_LOGW(TAG, "esp8266_at_start already called");
        return;
    }
    s_started = true;

    ESP_LOGI(TAG, "ESP8266 AT UART start");
    ESP_LOGI(TAG, "UART1 TX=GPIO26 RX=GPIO27 baud=115200");
    ESP_LOGI(TAG, "If flying wires: ESP8266 TXD -> GPIO27, ESP8266 RXD -> GPIO26, 5V/GND common");
    ESP_LOGI(TAG, "If using the ATK module socket that was tested before, keep the same successful wiring.");

    uart_config_t cfg = {
        .baud_rate = ESP_AT_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(ESP_AT_UART_PORT, ESP_AT_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ESP_AT_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(ESP_AT_UART_PORT, ESP_AT_TX_PIN, ESP_AT_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vTaskDelay(pdMS_TO_TICKS(300));
}

static void at_flush(void)
{
    uart_flush_input(ESP_AT_UART_PORT);
    vTaskDelay(pdMS_TO_TICKS(30));
}

static bool at_wait_for(const char *expect1, const char *expect2, int timeout_ms)
{
    char total[2048];
    int total_len = 0;
    int64_t end_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;

    memset(total, 0, sizeof(total));

    while (esp_timer_get_time() < end_us) {
        uint8_t buf[256];
        int n = uart_read_bytes(ESP_AT_UART_PORT, buf, sizeof(buf) - 1, pdMS_TO_TICKS(200));
        if (n > 0) {
            buf[n] = 0;
            ESP_LOGI(TAG, "RX: %s", (char *)buf);

            int copy = n;
            if (total_len + copy >= (int)sizeof(total) - 1) {
                copy = (int)sizeof(total) - 1 - total_len;
            }
            if (copy > 0) {
                memcpy(total + total_len, buf, copy);
                total_len += copy;
                total[total_len] = 0;
            }

            if (expect1 && strstr(total, expect1)) {
                return true;
            }
            if (expect2 && strstr(total, expect2)) {
                return true;
            }
            if (strstr(total, "ERROR") || strstr(total, "FAIL")) {
                return false;
            }
        }
    }

    ESP_LOGW(TAG, "Wait timeout, expect='%s' or '%s'", expect1 ? expect1 : "", expect2 ? expect2 : "");
    return false;
}

static bool at_cmd_expect(const char *cmd, const char *expect1, const char *expect2, int timeout_ms)
{
    ESP_LOGI(TAG, "TX: %s", cmd);
    at_flush();
    uart_write_bytes(ESP_AT_UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(ESP_AT_UART_PORT, "\r\n", 2);
    return at_wait_for(expect1, expect2, timeout_ms);
}

bool esp8266_at_wifi_test(const char *ssid, const char *password)
{
    char cmd[160];

    ESP_LOGI(TAG, "ESP8266 WiFi AT test start");

    // Some ESP8266 modules print boot messages after power-on; give it a moment.
    vTaskDelay(pdMS_TO_TICKS(1200));

    if (!at_cmd_expect("AT", "OK", NULL, 2000)) {
        ESP_LOGE(TAG, "AT no response. Check ESP8266 power and UART wiring.");
        return false;
    }

    at_cmd_expect("ATE0", "OK", NULL, 1000);
    at_cmd_expect("AT+CWMODE=1", "OK", NULL, 2000);
    at_cmd_expect("AT+CIPMUX=0", "OK", NULL, 2000);

    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    if (!at_cmd_expect(cmd, "WIFI GOT IP", "OK", 20000)) {
        ESP_LOGE(TAG, "WiFi connect failed. Check SSID/password or ESP8266 signal.");
        return false;
    }

    at_cmd_expect("AT+CIFSR", "OK", NULL, 3000);

    ESP_LOGI(TAG, "ESP8266 WIFI READY");
    return true;
}


typedef struct {
    char ssid[64];
    char password[64];
} esp8266_wifi_task_args_t;

static void esp8266_wifi_task(void *arg)
{
    esp8266_wifi_task_args_t *cfg = (esp8266_wifi_task_args_t *)arg;

    ESP_LOGI(TAG, "ESP8266 WiFi task started with larger stack");

    bool ok = esp8266_at_wifi_test(cfg->ssid, cfg->password);
    if (ok) {
        ESP_LOGI(TAG, "ESP8266 WIFI TASK OK");
        ESP_LOGI(TAG, "AI periodic analysis enabled: every %d ms", AI_ANALYSIS_INTERVAL_MS);
        ESP_LOGI(TAG, "Normal mode: capture 3 real camera snapshots every 10s + radar data for AI vision analysis");
        ESP_LOGI(TAG, "Demo mode: press BOOT key to upload /sdcard/DEMO_FALL.JPG for quick fall demonstration");
        ESP_LOGI(TAG, "TF demo image path: %s", AI_DEMO_FALL_FILE);
        ai_demo_key_start_once();
        list_sdcard_root_for_debug();

        int cycle = 1;
        while (1) {
            ESP_LOGI(TAG, "AI periodic analysis cycle %d start", cycle);
            bool ai_ok = esp8266_at_ai_analysis_test();
            ESP_LOGI(TAG, "AI periodic analysis cycle %d result: %s", cycle, ai_ok ? "OK" : "FAILED");
            cycle++;

            ai_wait_next_cycle_or_key();
        }
    } else {
        ESP_LOGE(TAG, "ESP8266 WIFI TASK FAILED");
    }

    vPortFree(cfg);
    vTaskDelete(NULL);
}

void esp8266_at_wifi_test_async(const char *ssid, const char *password)
{
    esp8266_wifi_task_args_t *cfg = (esp8266_wifi_task_args_t *)pvPortMalloc(sizeof(esp8266_wifi_task_args_t));
    if (!cfg) {
        ESP_LOGE(TAG, "No memory for ESP8266 WiFi task args");
        return;
    }

    snprintf(cfg->ssid, sizeof(cfg->ssid), "%s", ssid ? ssid : "");
    snprintf(cfg->password, sizeof(cfg->password), "%s", password ? password : "");

    BaseType_t ret = xTaskCreate(esp8266_wifi_task, "esp8266_wifi", 12288, cfg, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Create ESP8266 WiFi task failed");
        vPortFree(cfg);
        return;
    }

    ESP_LOGI(TAG, "ESP8266 WiFi task created");
}



static void *ai_heap_malloc(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }
    return p;
}

static void ai_heap_free(void *p)
{
    if (p) {
        heap_caps_free(p);
    }
}

static void list_sdcard_root_for_debug(void)
{
    DIR *dir = opendir("/sdcard");
    if (!dir) {
        ESP_LOGW(TAG, "Open /sdcard failed when listing root directory");
        return;
    }

    ESP_LOGI(TAG, "========== /sdcard file list ==========");
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "/sdcard/%s", entry->d_name);
    }
    ESP_LOGI(TAG, "=======================================");
    closedir(dir);
}

static bool file_exists_nonempty(const char *path)
{
    size_t sz = 0;
    return get_file_size_bytes(path, &sz);
}

static const char *find_demo_fall_image_path(void)
{
    /*
     * Some ESP-IDF FatFS configurations disable long file name support.
     * In that case Windows shows DEMO_FALL.JPG, but ESP32 sees the 8.3 alias
     * DEMO_F~1.JPG in readdir(), and fopen("/sdcard/DEMO_FALL.JPG") fails.
     */
    if (file_exists_nonempty(AI_DEMO_FALL_FILE)) {
        return AI_DEMO_FALL_FILE;
    }
    if (file_exists_nonempty(AI_DEMO_FALL_FILE_83)) {
        return AI_DEMO_FALL_FILE_83;
    }
    if (file_exists_nonempty(AI_DEMO_FALL_FILE_ALT)) {
        return AI_DEMO_FALL_FILE_ALT;
    }

    DIR *dir = opendir("/sdcard");
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (strstr(entry->d_name, "DEMO_F") && strstr(entry->d_name, ".JPG")) {
                static char found_path[320];
                int n = snprintf(found_path, sizeof(found_path), "/sdcard/%s", entry->d_name);
                if (n < 0 || n >= (int)sizeof(found_path)) {
                    ESP_LOGW(TAG, "Skip too long sdcard filename: %s", entry->d_name);
                    continue;
                }
                closedir(dir);
                if (file_exists_nonempty(found_path)) {
                    return found_path;
                }
                return found_path;
            }
        }
        closedir(dir);
    }

    return NULL;
}


static bool get_file_size_bytes(const char *path, size_t *out_size)
{
    if (!path || !out_size) {
        return false;
    }

    struct stat st;
    if (stat(path, &st) == 0 && st.st_size > 0) {
        *out_size = (size_t)st.st_size;
        return true;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        return false;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return false;
    }

    long sz = ftell(f);
    fclose(f);

    if (sz <= 0) {
        return false;
    }

    *out_size = (size_t)sz;
    return true;
}

static uint8_t *read_file_to_heap(const char *path, size_t *out_size)
{
    size_t file_size = 0;
    if (!get_file_size_bytes(path, &file_size)) {
        ESP_LOGE(TAG, "Image file not found or empty: %s", path);
        list_sdcard_root_for_debug();
        return NULL;
    }

    if (file_size > AI_DEMO_IMAGE_MAX_BYTES) {
        ESP_LOGE(TAG, "Image file too large: %u bytes, max=%u bytes. Please compress it smaller.",
                 (unsigned)file_size, (unsigned)AI_DEMO_IMAGE_MAX_BYTES);
        return NULL;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Open image failed: %s", path);
        list_sdcard_root_for_debug();
        return NULL;
    }

    uint8_t *buf = (uint8_t *)ai_heap_malloc(file_size);
    if (!buf) {
        ESP_LOGE(TAG, "No memory for image file: %u bytes", (unsigned)file_size);
        fclose(f);
        return NULL;
    }

    size_t n = fread(buf, 1, file_size, f);
    fclose(f);

    if (n != file_size) {
        ESP_LOGE(TAG, "Read image failed: got=%u expected=%u", (unsigned)n, (unsigned)file_size);
        ai_heap_free(buf);
        return NULL;
    }

    *out_size = file_size;
    ESP_LOGI(TAG, "Image loaded: %s, %u bytes", path, (unsigned)file_size);
    return buf;
}

static char *base64_encode_alloc(const uint8_t *src, size_t len, size_t *out_len)
{
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    if (!src || len == 0 || !out_len) {
        return NULL;
    }

    size_t enc_len = ((len + 2) / 3) * 4;
    char *out = (char *)ai_heap_malloc(enc_len + 1);
    if (!out) {
        ESP_LOGE(TAG, "No memory for base64: %u bytes", (unsigned)(enc_len + 1));
        return NULL;
    }

    size_t i = 0;
    size_t j = 0;
    while (i < len) {
        uint32_t octet_a = i < len ? src[i++] : 0;
        uint32_t octet_b = i < len ? src[i++] : 0;
        uint32_t octet_c = i < len ? src[i++] : 0;
        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        out[j++] = table[(triple >> 18) & 0x3F];
        out[j++] = table[(triple >> 12) & 0x3F];
        out[j++] = table[(triple >> 6) & 0x3F];
        out[j++] = table[triple & 0x3F];
    }

    size_t mod = len % 3;
    if (mod) {
        out[enc_len - 1] = '=';
        if (mod == 1) {
            out[enc_len - 2] = '=';
        }
    }

    out[enc_len] = 0;
    *out_len = enc_len;
    ESP_LOGI(TAG, "Image base64 encoded: raw=%u base64=%u", (unsigned)len, (unsigned)enc_len);
    return out;
}

static void ai_demo_key_task(void *arg)
{
    (void)arg;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << AI_DEMO_KEY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "AI demo key enabled: BOOT GPIO%d, press to toggle normal/demo image mode", AI_DEMO_KEY_GPIO);

    int last_level = 1;

    while (1) {
        int level = gpio_get_level(AI_DEMO_KEY_GPIO);

        if (last_level == 1 && level == AI_DEMO_KEY_ACTIVE) {
            vTaskDelay(pdMS_TO_TICKS(30));

            if (gpio_get_level(AI_DEMO_KEY_GPIO) == AI_DEMO_KEY_ACTIVE) {
                s_ai_demo_fall_mode = !s_ai_demo_fall_mode;
                s_ai_force_analysis_now = true;

                ESP_LOGW(TAG, "========== AI MODE TOGGLE ==========");
                ESP_LOGW(TAG, "Current mode: %s",
                         s_ai_demo_fall_mode ? "FALL DEMO IMAGE / TF卡摔倒图片分析模式" : "NORMAL / 正常雷达摄像头状态模式");
                ESP_LOGW(TAG, "Next AI analysis will run immediately if WiFi is ready");
                ESP_LOGW(TAG, "====================================");

                while (gpio_get_level(AI_DEMO_KEY_GPIO) == AI_DEMO_KEY_ACTIVE) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }

                last_level = 1;
                continue;
            }
        }

        last_level = level;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void ai_demo_key_start_once(void)
{
    if (s_ai_demo_key_task_handle) {
        return;
    }

    BaseType_t ret = xTaskCreate(ai_demo_key_task, "ai_demo_key", 4096, NULL, 6, &s_ai_demo_key_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Create AI demo key task failed");
        s_ai_demo_key_task_handle = NULL;
    }
}

static void ai_wait_next_cycle_or_key(void)
{
    int waited = 0;
    while (waited < AI_ANALYSIS_INTERVAL_MS) {
        if (s_ai_force_analysis_now) {
            s_ai_force_analysis_now = false;
            ESP_LOGI(TAG, "AI analysis interval interrupted by key toggle");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }
}

static bool token_configured(void)
{
    return strcmp(AISTUDIO_ACCESS_TOKEN, "PASTE_YOUR_AI_STUDIO_ACCESS_TOKEN_HERE") != 0 &&
           strlen(AISTUDIO_ACCESS_TOKEN) > 20;
}


static void log_ai_content_from_response(const char *resp)
{
    if (!resp) {
        return;
    }

    const char *p = strstr(resp, "\"content\":\"");
    if (!p) {
        return;
    }

    p += strlen("\"content\":\"");

    char out[512];
    int j = 0;

    while (*p && j < (int)sizeof(out) - 1) {
        if (*p == '"' && (p == resp || *(p - 1) != '\\')) {
            break;
        }

        if (*p == '\\') {
            p++;
            if (*p == 'n' || *p == 'r' || *p == 't') {
                out[j++] = ' ';
            } else if (*p == '"' || *p == '\\' || *p == '/') {
                out[j++] = *p;
            } else {
                if (j < (int)sizeof(out) - 2) {
                    out[j++] = '\\';
                    out[j++] = *p;
                }
            }
            if (*p) {
                p++;
            }
            continue;
        }

        out[j++] = *p++;
    }

    out[j] = 0;

    if (j > 0) {
        ESP_LOGI(TAG, "========== AI ANALYSIS RESULT ==========");
        ESP_LOGI(TAG, "%s", out);
        ESP_LOGI(TAG, "=======================================");
    }
}

static bool ai_response_ok(const char *resp)
{
    if (!resp) {
        return false;
    }

    bool ok = (strstr(resp, "HTTP/1.1 200") != NULL) &&
              (strstr(resp, "\"choices\"") != NULL) &&
              (strstr(resp, "\"content\"") != NULL);

    if (ok) {
        log_ai_content_from_response(resp);
        ESP_LOGI(TAG, "AI Studio response looks OK");
        return true;
    }

    ESP_LOGW(TAG, "AI response received, but did not find choices/content. Check token/model/path.");
    return false;
}

static bool wait_for_http_response(int timeout_ms)
{
    int64_t end_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;
    bool got_any = false;

    char collect[4096];
    int collect_len = 0;
    memset(collect, 0, sizeof(collect));

    while (esp_timer_get_time() < end_us) {
        uint8_t buf[512];
        int n = uart_read_bytes(ESP_AT_UART_PORT, buf, sizeof(buf) - 1, pdMS_TO_TICKS(300));
        if (n > 0) {
            got_any = true;
            buf[n] = 0;
            ESP_LOGI(TAG, "AI RX: %s", (char *)buf);

            int copy = n;
            if (collect_len + copy >= (int)sizeof(collect) - 1) {
                copy = (int)sizeof(collect) - 1 - collect_len;
            }
            if (copy > 0) {
                memcpy(collect + collect_len, buf, copy);
                collect_len += copy;
                collect[collect_len] = 0;
            }

            if (strstr(collect, "CLOSED")) {
                break;
            }
        }
    }

    if (!got_any) {
        ESP_LOGE(TAG, "AI request no response");
        return false;
    }

    return ai_response_ok(collect);
}

static bool at_send_bytes_chunked(const char *data, int len)
{
    if (!data || len <= 0) {
        return false;
    }

    int sent = 0;
    int chunk_index = 1;
    char cmd[64];

    while (sent < len) {
        int chunk = len - sent;
        if (chunk > AI_SEND_CHUNK_SIZE) {
            chunk = AI_SEND_CHUNK_SIZE;
        }

        snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d", chunk);
        if (!at_cmd_expect(cmd, ">", NULL, 8000)) {
            ESP_LOGE(TAG, "AI CIPSEND failed at chunk %d", chunk_index);
            return false;
        }

        ESP_LOGI(TAG, "Sending AI HTTP chunk %d: %d bytes, progress=%d/%d",
                 chunk_index, chunk, sent + chunk, len);
        uart_write_bytes(ESP_AT_UART_PORT, data + sent, chunk);

        if (!at_wait_for("SEND OK", NULL, 12000)) {
            ESP_LOGE(TAG, "AI SEND OK timeout at chunk %d", chunk_index);
            return false;
        }

        sent += chunk;
        chunk_index++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return true;
}

static bool ai_http_post_raw(const char *req, int req_len, int response_timeout_ms)
{
    char cmd[128];

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"SSL\",\"%s\",%d", AISTUDIO_HOST, AISTUDIO_PORT);
    if (!at_cmd_expect(cmd, "CONNECT", "OK", 15000)) {
        ESP_LOGE(TAG, "AI SSL connect failed");
        at_cmd_expect("AT+CIPCLOSE", "OK", NULL, 2000);
        return false;
    }

    ESP_LOGI(TAG, "Sending AI HTTP request in chunks, total=%d bytes", req_len);
    if (!at_send_bytes_chunked(req, req_len)) {
        ESP_LOGE(TAG, "AI HTTP request send failed");
        at_cmd_expect("AT+CIPCLOSE", "OK", NULL, 2000);
        return false;
    }

    bool ok = wait_for_http_response(response_timeout_ms);

    at_cmd_expect("AT+CIPCLOSE", "OK", NULL, 2000);

    if (ok) {
        ESP_LOGI(TAG, "AI Studio analysis request OK");
    } else {
        ESP_LOGE(TAG, "AI Studio analysis request failed");
    }

    return ok;
}

static bool build_http_request_from_body(const char *body, int body_len, char **out_req, int *out_req_len)
{
    if (!body || body_len <= 0 || !out_req || !out_req_len) {
        return false;
    }

    char header[768];
    int header_len = snprintf(header, sizeof(header),
        "POST %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Authorization: Bearer %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n",
        AISTUDIO_PATH,
        AISTUDIO_HOST,
        AISTUDIO_ACCESS_TOKEN,
        body_len
    );

    if (header_len <= 0 || header_len >= (int)sizeof(header)) {
        ESP_LOGE(TAG, "HTTP header build failed or truncated");
        return false;
    }

    int req_len = header_len + body_len;
    char *req = (char *)ai_heap_malloc(req_len + 1);
    if (!req) {
        ESP_LOGE(TAG, "No memory for AI HTTP request: %d bytes", req_len + 1);
        return false;
    }

    memcpy(req, header, header_len);
    memcpy(req + header_len, body, body_len);
    req[req_len] = 0;

    *out_req = req;
    *out_req_len = req_len;
    return true;
}


static bool ai_wait_ms_interruptible(int total_ms)
{
    int waited = 0;
    while (waited < total_ms) {
        if (s_ai_force_analysis_now) {
            ESP_LOGI(TAG, "AI snapshot minute interrupted by BOOT key");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }
    return true;
}

static void cleanup_realtime_snapshot_files(void)
{
    unlink(AI_REAL_SNAPSHOT_1);
    unlink(AI_REAL_SNAPSHOT_2);
    unlink(AI_REAL_SNAPSHOT_3);
}

static bool capture_realtime_snapshot(const char *path, int index, int total)
{
    ESP_LOGI(TAG, "Capture realtime AI snapshot %d/%d: %s", index, total, path);
    esp_err_t ret = mipi_cam_save_snapshot_jpg(path, 5000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Capture realtime AI snapshot failed: %s, ret=%s", path, esp_err_to_name(ret));
        return false;
    }

    size_t sz = 0;
    if (get_file_size_bytes(path, &sz)) {
        ESP_LOGI(TAG, "Realtime AI snapshot ready: %s, %u bytes", path, (unsigned)sz);
    }
    return true;
}

static bool build_and_post_three_snapshot_images(const char *path1, const char *path2, const char *path3)
{
    radar_state_t ai_rs = radar_get_state();
    const char *paths[3] = { path1, path2, path3 };
    char *b64[3] = { NULL, NULL, NULL };
    size_t b64_len[3] = { 0, 0, 0 };

    for (int i = 0; i < 3; i++) {
        size_t img_size = 0;
        uint8_t *img = read_file_to_heap(paths[i], &img_size);
        if (!img) {
            for (int j = 0; j < 3; j++) {
                ai_heap_free(b64[j]);
            }
            return false;
        }

        b64[i] = base64_encode_alloc(img, img_size, &b64_len[i]);
        ai_heap_free(img);
        if (!b64[i]) {
            for (int j = 0; j < 3; j++) {
                ai_heap_free(b64[j]);
            }
            return false;
        }
    }

    char *prefix = (char *)ai_heap_malloc(3600);
    if (!prefix) {
        ESP_LOGE(TAG, "No memory for 3-snapshot AI prefix");
        for (int i = 0; i < 3; i++) ai_heap_free(b64[i]);
        return false;
    }

    int prefix_len = snprintf(prefix, 3600,
        "{\"model\":\"%s\"," 
        "\"messages\":["
            "{"
                "\"role\":\"system\"," 
                "\"content\":\"你是健康监测原型机的视觉风险提示助手，只做非医疗诊断的风险提醒。你会看到约20秒内每10秒抽取的三帧实时摄像头画面，并结合雷达心率呼吸数据判断。若三帧均未检测到人，必须输出：未检测到人，风险等级写无。若看到疑似倒地、趴卧、躺倒、长时间低位静止等，提示高风险并建议人工确认。必须保留具体心率和呼吸数字。\""
            "},"
            "{"
                "\"role\":\"user\"," 
                "\"content\":["
                    "{"
                        "\"type\":\"text\"," 
                        "\"text\":\"以下是正常实时监测模式下按10秒间隔抽取的三帧摄像头画面，约为第0秒、第10秒、第20秒。雷达同步数据：距离%d厘米，体动%d/100，运动状态%d，心率%d次/分，呼吸%d次/分。请用中文输出不超过190字，格式固定为：分析结果：心率%d次/分，呼吸%d次/分，三帧画面中...；风险等级：无/低/中/高；建议：...。要求：如果三帧均未检测到人，请明确写未检测到人，风险等级写无；如果检测到人，请判断姿态是否正常或疑似摔倒；不要说确定诊断。\""
                    "}",
        AISTUDIO_VL_MODEL,
        ai_rs.distance_cm,
        ai_rs.body_motion,
        ai_rs.motion,
        ai_rs.heart_rate,
        ai_rs.breath_rate,
        ai_rs.heart_rate,
        ai_rs.breath_rate
    );

    if (prefix_len <= 0 || prefix_len >= 3600) {
        ESP_LOGE(TAG, "3-snapshot AI prefix build failed or truncated: %d", prefix_len);
        ai_heap_free(prefix);
        for (int i = 0; i < 3; i++) ai_heap_free(b64[i]);
        return false;
    }

    const char *img_prefix = ",{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,";
    const char *img_suffix = "\",\"detail\":\"low\"}}";
    const char *suffix = "]}],\"max_tokens\":210,\"temperature\":0.1}";

    int img_prefix_len = strlen(img_prefix);
    int img_suffix_len = strlen(img_suffix);
    int suffix_len = strlen(suffix);
    int body_len = prefix_len + suffix_len;
    for (int i = 0; i < 3; i++) {
        body_len += img_prefix_len + (int)b64_len[i] + img_suffix_len;
    }

    char *body = (char *)ai_heap_malloc(body_len + 1);
    if (!body) {
        ESP_LOGE(TAG, "No memory for 3-snapshot AI body: %d bytes", body_len + 1);
        ai_heap_free(prefix);
        for (int i = 0; i < 3; i++) ai_heap_free(b64[i]);
        return false;
    }

    char *w = body;
    memcpy(w, prefix, prefix_len); w += prefix_len;
    for (int i = 0; i < 3; i++) {
        memcpy(w, img_prefix, img_prefix_len); w += img_prefix_len;
        memcpy(w, b64[i], b64_len[i]); w += b64_len[i];
        memcpy(w, img_suffix, img_suffix_len); w += img_suffix_len;
    }
    memcpy(w, suffix, suffix_len); w += suffix_len;
    *w = 0;

    ai_heap_free(prefix);
    for (int i = 0; i < 3; i++) ai_heap_free(b64[i]);

    char *req = NULL;
    int req_len = 0;
    bool built = build_http_request_from_body(body, body_len, &req, &req_len);
    ai_heap_free(body);

    if (!built) {
        return false;
    }

    ESP_LOGI(TAG, "AI 3-snapshot HTTP body length=%d request length=%d", body_len, req_len);
    ESP_LOGI(TAG, "3-snapshot upload uses VL model: %s", AISTUDIO_VL_MODEL);

    bool ok = ai_http_post_raw(req, req_len, 60000);
    ai_heap_free(req);
    return ok;
}

static bool esp8266_at_ai_realtime_snapshot_analysis(void)
{
    ESP_LOGI(TAG, "AI request mode: realtime_3_snapshot_image_analysis_10s_interval");
    ESP_LOGI(TAG, "Realtime cycle: clear previous AI_*.JPG, capture 3 frames at 10s interval, upload to AI");

    cleanup_realtime_snapshot_files();

    if (!capture_realtime_snapshot(AI_REAL_SNAPSHOT_1, 1, 3)) {
        return false;
    }

    if (!ai_wait_ms_interruptible(AI_REAL_SNAPSHOT_WAIT_MS)) {
        return false;
    }

    if (!capture_realtime_snapshot(AI_REAL_SNAPSHOT_2, 2, 3)) {
        return false;
    }

    if (!ai_wait_ms_interruptible(AI_REAL_SNAPSHOT_WAIT_MS)) {
        return false;
    }

    if (!capture_realtime_snapshot(AI_REAL_SNAPSHOT_3, 3, 3)) {
        return false;
    }

    ESP_LOGI(TAG, "Upload 3 realtime camera snapshots + radar data to AI");
    return build_and_post_three_snapshot_images(AI_REAL_SNAPSHOT_1,
                                                AI_REAL_SNAPSHOT_2,
                                                AI_REAL_SNAPSHOT_3);
}

static bool esp8266_at_ai_normal_analysis(void)
{
    radar_state_t ai_rs = radar_get_state();

    ESP_LOGI(TAG, "AI request mode: normal_radar_camera_status_60s");

    char *body = (char *)ai_heap_malloc(2800);
    if (!body) {
        ESP_LOGE(TAG, "No memory for normal AI body");
        return false;
    }

    const char *camera_scene =
        "正常模式：摄像头正在本地采集并保存图像作为本地证据，本次AI只接收雷达数据和摄像头状态摘要。请不要提示图片未上传。";

    int body_len = snprintf(body, 2800,
        "{\"model\":\"%s\"," 
        "\"messages\":["
            "{"
                "\"role\":\"system\"," 
                "\"content\":\"你是健康监测原型机的风险提示助手，只能做非医疗诊断的风险提醒。不要说确定诊断，不要说图片未上传。输出时必须保留输入中的具体心率和呼吸数值，不能只写正常范围。\""
            "},"
            "{"
                "\"role\":\"user\"," 
                "\"content\":\"雷达数据：距离%d厘米，体动%d/100，运动状态%d，心率%d次/分，呼吸%d次/分。摄像头状态：%s。请用中文输出不超过140字，格式固定为：分析结果：心率%d次/分，呼吸%d次/分，...；风险等级：低/中/高；建议：...。要求：必须逐字保留心率和呼吸的具体数字，不能只写心率、呼吸正常。若数值为-1，请写暂未获取。\""
            "}"
        "],"
        "\"max_tokens\":140,"
        "\"temperature\":0.2"
        "}",
        AISTUDIO_MODEL,
        ai_rs.distance_cm,
        ai_rs.body_motion,
        ai_rs.motion,
        ai_rs.heart_rate,
        ai_rs.breath_rate,
        camera_scene,
        ai_rs.heart_rate,
        ai_rs.breath_rate
    );

    if (body_len <= 0 || body_len >= 2800) {
        ESP_LOGE(TAG, "Normal AI body build failed or truncated: %d", body_len);
        ai_heap_free(body);
        return false;
    }

    char *req = NULL;
    int req_len = 0;
    bool built = build_http_request_from_body(body, body_len, &req, &req_len);
    ai_heap_free(body);

    if (!built) {
        return false;
    }

    ESP_LOGI(TAG, "AI HTTP body length=%d request length=%d", body_len, req_len);
    bool ok = ai_http_post_raw(req, req_len, 25000);
    ai_heap_free(req);
    return ok;
}

static bool esp8266_at_ai_demo_fall_image_analysis(void)
{
    radar_state_t ai_rs = radar_get_state();

    ESP_LOGW(TAG, "AI request mode: fall_demo_image_analysis");

    const char *demo_path = find_demo_fall_image_path();
    if (!demo_path) {
        ESP_LOGE(TAG, "Demo image not found. Try one of: %s, %s, %s",
                 AI_DEMO_FALL_FILE, AI_DEMO_FALL_FILE_83, AI_DEMO_FALL_FILE_ALT);
        list_sdcard_root_for_debug();
        return false;
    }
    ESP_LOGW(TAG, "Demo image file resolved: %s", demo_path);

    size_t img_size = 0;
    uint8_t *img = read_file_to_heap(demo_path, &img_size);
    if (!img) {
        return false;
    }

    size_t b64_len = 0;
    char *b64 = base64_encode_alloc(img, img_size, &b64_len);
    ai_heap_free(img);
    if (!b64) {
        return false;
    }

    char *prefix = (char *)ai_heap_malloc(2600);
    if (!prefix) {
        ESP_LOGE(TAG, "No memory for demo image body prefix");
        ai_heap_free(b64);
        return false;
    }

    int prefix_len = snprintf(prefix, 2600,
        "{\"model\":\"%s\"," 
        "\"messages\":["
            "{"
                "\"role\":\"system\"," 
                "\"content\":\"你是健康监测原型机的风险提示助手，只做非医疗诊断的异常风险提示。看到疑似倒地、躺倒、长时间低位静止等场景时，应提示高风险并建议人工确认。输出时必须保留输入中的具体心率和呼吸数值。\""
            "},"
            "{"
                "\"role\":\"user\"," 
                "\"content\":["
                    "{"
                        "\"type\":\"text\"," 
                        "\"text\":\"请分析这张TF卡中的演示图片是否存在老人跌倒、躺倒、异常姿态或需要人工查看的风险。雷达同步数据：距离%d厘米，体动%d/100，运动状态%d，心率%d次/分，呼吸%d次/分。请用中文输出不超过170字，格式固定为：分析结果：心率%d次/分，呼吸%d次/分，图片中...；风险等级：低/中/高；建议：...。要求：必须保留心率和呼吸具体数字，不能只写正常范围；不要说确定诊断。\""
                    "},"
                    "{"
                        "\"type\":\"image_url\"," 
                        "\"image_url\":{\"url\":\"data:image/jpeg;base64,",
        AISTUDIO_VL_MODEL,
        ai_rs.distance_cm,
        ai_rs.body_motion,
        ai_rs.motion,
        ai_rs.heart_rate,
        ai_rs.breath_rate,
        ai_rs.heart_rate,
        ai_rs.breath_rate
    );

    if (prefix_len <= 0 || prefix_len >= 2600) {
        ESP_LOGE(TAG, "Demo image body prefix build failed or truncated: %d", prefix_len);
        ai_heap_free(prefix);
        ai_heap_free(b64);
        return false;
    }

    const char *suffix = "\",\"detail\":\"high\"}}]}],\"max_tokens\":180,\"temperature\":0.1}";
    int suffix_len = strlen(suffix);
    int body_len = prefix_len + (int)b64_len + suffix_len;

    char *body = (char *)ai_heap_malloc(body_len + 1);
    if (!body) {
        ESP_LOGE(TAG, "No memory for demo image AI body: %d bytes", body_len + 1);
        ai_heap_free(prefix);
        ai_heap_free(b64);
        return false;
    }

    memcpy(body, prefix, prefix_len);
    memcpy(body + prefix_len, b64, b64_len);
    memcpy(body + prefix_len + b64_len, suffix, suffix_len);
    body[body_len] = 0;

    ai_heap_free(prefix);
    ai_heap_free(b64);

    char *req = NULL;
    int req_len = 0;
    bool built = build_http_request_from_body(body, body_len, &req, &req_len);
    ai_heap_free(body);

    if (!built) {
        return false;
    }

    ESP_LOGI(TAG, "AI image HTTP body length=%d request length=%d", body_len, req_len);
    ESP_LOGI(TAG, "Image upload uses VL model: %s", AISTUDIO_VL_MODEL);

    bool ok = ai_http_post_raw(req, req_len, 45000);
    ai_heap_free(req);
    return ok;
}

static bool esp8266_at_ai_analysis_test(void)
{
    if (!token_configured()) {
        ESP_LOGE(TAG, "AI Studio token not configured. Edit esp8266_at.c and replace PASTE_YOUR_AI_STUDIO_ACCESS_TOKEN_HERE.");
        return false;
    }

    radar_state_t rs = radar_get_state();

    ESP_LOGI(TAG, "AI Studio HTTP POST start");
    ESP_LOGI(TAG, "Radar state for AI: distance=%d body=%d motion=%d heart=%d breath=%d",
             rs.distance_cm, rs.body_motion, rs.motion, rs.heart_rate, rs.breath_rate);
    ESP_LOGI(TAG, "Current AI mode: %s", s_ai_demo_fall_mode ? "FALL DEMO IMAGE" : "NORMAL");

    if (s_ai_demo_fall_mode) {
        return esp8266_at_ai_demo_fall_image_analysis();
    }

    return esp8266_at_ai_realtime_snapshot_analysis();
}
