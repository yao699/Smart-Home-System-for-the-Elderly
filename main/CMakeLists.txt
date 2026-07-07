# Fixed CMakeLists for ESP32-P4 OV5645 MIPI camera + onboard TF card frame save test
# Use explicit SRCS so backup .c files in main/ or APP/MIPI_CAM/ will not be compiled.

idf_component_register(
    SRCS
        "main.c"
        "APP/MIPI_CAM/app_video.c"
        "APP/MIPI_CAM/mipi_cam.c"
        "APP/RADAR/radar_uart.c"
        "APP/ESP8266_AT/esp8266_at.c"
    INCLUDE_DIRS
        "."
        "APP/MIPI_CAM"
        "APP/RADAR"
        "APP/ESP8266_AT"
    PRIV_REQUIRES
        BSP
        nvs_flash
        fatfs
        sdmmc
        esp_timer
        esp_driver_gpio
        esp_driver_uart
        esp_driver_sdmmc
        esp_driver_ppa
        esp_driver_cam
        esp_driver_isp
        esp_driver_jpeg
        esp_video
        esp_cam_sensor
)
