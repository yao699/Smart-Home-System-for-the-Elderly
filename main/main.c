/**
 ******************************************************************************
 * @file        main.c
 * @brief       ESP32-P4 OV5645 MIPI camera + onboard TF card frame save test
 *
 * 功能：
 * 1. 不初始化 MIPI LCD，避免未接屏幕时卡死。
 * 2. 挂载正点原子 ESP32-P4 板载 TF 卡槽。
 * 3. 初始化 OV5645 MIPI 摄像头。
 * 4. 摄像头采集任务会保存前几帧 YUV422 原始图像到 TF 卡。
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

#include "led.h"
#include "myiic.h"
#include "mipi_cam.h"
#include "radar_uart.h"
#include "esp8266_at.h"

static const char *TAG = "MAIN";

#define MOUNT_POINT "/sdcard"

/* 正点原子 ESP32-P4 板载 TF 卡槽 SDMMC 引脚 */
#define SD_PIN_CLK GPIO_NUM_43
#define SD_PIN_CMD GPIO_NUM_44
#define SD_PIN_D0  GPIO_NUM_39
#define SD_PIN_D1  GPIO_NUM_40
#define SD_PIN_D2  GPIO_NUM_41
#define SD_PIN_D3  GPIO_NUM_42

static sdmmc_card_t *s_sdcard = NULL;

static esp_err_t write_boot_file(void)
{
    const char *path = MOUNT_POINT "/BOOT.TXT";
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "open %s failed", path);
        return ESP_FAIL;
    }

    fprintf(f, "ESP32-P4 camera + SD card test OK\n");
    fprintf(f, "Files F0001.YUV/F0002.YUV... will be saved by camera task.\n");
    fclose(f);
    ESP_LOGI(TAG, "Boot file written: %s", path);
    return ESP_OK;
}

static esp_err_t sdcard_mount_onboard(void)
{
    ESP_LOGI(TAG, "Mount onboard TF card at %s", MOUNT_POINT);
    ESP_LOGI(TAG, "SDMMC pins: CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d",
             SD_PIN_CLK, SD_PIN_CMD, SD_PIN_D0, SD_PIN_D1, SD_PIN_D2, SD_PIN_D3);

    gpio_set_pull_mode(SD_PIN_CMD, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_PIN_D0, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_PIN_D1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_PIN_D2, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_PIN_D3, GPIO_PULLUP_ONLY);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = SD_PIN_CLK;
    slot_config.cmd = SD_PIN_CMD;
    slot_config.d0 = SD_PIN_D0;
    slot_config.d1 = SD_PIN_D1;
    slot_config.d2 = SD_PIN_D2;
    slot_config.d3 = SD_PIN_D3;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &s_sdcard);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. Please format TF card as FAT32.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize TF card: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    ESP_LOGI(TAG, "SD card mounted successfully.");
    sdmmc_card_print_info(stdout, s_sdcard);

    return write_boot_file();
}

void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    led_init();
    myiic_init();

    ESP_LOGI(TAG, "LCD init skipped. Start SD card + MIPI camera frame save test.");
    ESP_LOGI(TAG, "Make sure TF card and camera are inserted before power on.");

    ret = sdcard_mount_onboard();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed. Camera will NOT start. ret=%s", esp_err_to_name(ret));
        while (1) {
            LED0_TOGGLE();
            ESP_LOGI(TAG, "waiting: fix TF card / FAT32 / insert before power on");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    ret = mipi_cam_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mipi_cam_init failed: %s", esp_err_to_name(ret));
        while (1) {
            LED0_TOGGLE();
            ESP_LOGI(TAG, "waiting: check camera FPC direction / camera connection");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    ESP_LOGI(TAG, "Camera init returned OK. Wait for F0001.YUV/F0002.YUV on TF card.");

    while (1) {
        LED0_TOGGLE();
static bool radar_started_once = false;
if (!radar_started_once) {
    radar_started_once = true;
    ESP_LOGI(TAG, "Start R60ABD1 radar UART parser");
    radar_uart_start();
}

static bool esp8266_wifi_started_once = false;
if (!esp8266_wifi_started_once) {
    esp8266_wifi_started_once = true;
    ESP_LOGI(TAG, "Start ESP8266 AT WiFi test");
    esp8266_at_start();
    esp8266_at_wifi_test_async("testwifi", "12345678");
}

ESP_LOGI(TAG, "main alive: camera task is running, TF card stays mounted");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
