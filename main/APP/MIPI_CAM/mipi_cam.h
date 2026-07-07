/**
 ****************************************************************************************************
 * @file        mipi_cam.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       mipicamera驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-P4 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __MIPI_CAM_H
#define __MIPI_CAM_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_video.h"
#include "myiic.h"
#include "driver/ppa.h"
#include "esp_timer.h"
#include "lcd.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"


#define OV5645_CAM_WIDTH        1280    /* OV5645摄像头图像输出宽度 */
#define OV5645_CAM_HEIGHT       960     /* OV5645摄像头图像输出高度 */  

/* 函数声明 */
esp_err_t mipi_cam_init(void);          /* mipi_cam初始化 */
esp_err_t mipi_cam_save_snapshot_jpg(const char *path, uint32_t timeout_ms); /* 保存一帧实时摄像头灰度JPG快照 */

#endif
