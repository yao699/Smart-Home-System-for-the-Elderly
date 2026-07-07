/**
 ****************************************************************************************************
 * @file        mipi_cam.c
 * @brief       OV5645 MIPI camera USERPTR capture + save YUV frames to onboard TF card
 *
 * 说明：
 * 1. 保留 OV5645 / MIPI-CSI 初始化，不初始化/不使用 MIPI LCD 显示。
 * 2. 使用 USERPTR 方式，由应用层在 PSRAM 中分配摄像头帧缓冲。
 * 3. 保存前几帧 YUV422 原始图像到 TF 卡：/sdcard/F0001.YUV、/sdcard/F0002.YUV、/sdcard/F0003.YUV。
 * 4. 同时写入 /sdcard/CAM.CSV，记录帧号、宽高、字节数、checksum、亮度统计。
 ****************************************************************************************************
 */

#include "mipi_cam.h"
#include "mipi_lcd.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <stdlib.h>

#include "esp_heap_caps.h"
#include "driver/jpeg_encode.h"
#include "driver/jpeg_types.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "linux/videodev2.h"

static const char *mipi_cam_tag = "mipi_cam";

#define CAPTURE_BUF_NUM            2
#define MEMORY_ALIGN               64
#define FRAME_LOG_INTERVAL         30

/* 交付版默认不保存调试用原始 YUV，避免 TF 卡留下隐私画面。 */
#define SAVE_FRAME_MAX_COUNT       0
#define SAVE_FRAME_INTERVAL        30
#define SD_FRAME_META_PATH         "/sdcard/CAM.CSV"

/*
 * AI 实时分析用快照：不保存 1 分钟原始视频，只在连续采集流中按需抽帧。
 * 为了减小网络上传量，这里把 640x480 的 Y 亮度平面下采样为 320x240 灰度 JPG。
 */
#define AI_SNAPSHOT_WIDTH          320
#define AI_SNAPSHOT_HEIGHT         240
#define AI_SNAPSHOT_JPEG_QUALITY   70
#define AI_SNAPSHOT_PATH_MAX       128
#define AI_SNAPSHOT_TIMEOUT_MS     3000

static int s_video_cam_fd = -1;
static uint8_t *s_camera_outbuf[CAPTURE_BUF_NUM] = {0};
static uint32_t s_camera_buf_size[CAPTURE_BUF_NUM] = {0};
static uint32_t s_saved_frame_count = 0;

typedef struct {
    SemaphoreHandle_t mutex;
    bool pending;
    bool done;
    char path[AI_SNAPSHOT_PATH_MAX];
    esp_err_t result;
    uint32_t source_frame;
} ai_snapshot_request_t;

static ai_snapshot_request_t s_ai_snapshot_req = {0};
static jpeg_encoder_handle_t s_jpeg_encoder = NULL;

typedef struct {
    uint32_t checksum;
    uint32_t avg;
    uint8_t min;
    uint8_t max;
    size_t samples;
} frame_stat_t;

static frame_stat_t frame_analyze_sampled(const uint8_t *data, size_t len)
{
    frame_stat_t st = {0};

    if (data == NULL || len == 0) {
        return st;
    }

    size_t target_samples = 4096;
    size_t step = len / target_samples;
    if (step == 0) {
        step = 1;
    }

    uint32_t checksum = 0x811c9dc5;
    uint64_t sum = 0;
    uint8_t min_v = 255;
    uint8_t max_v = 0;
    size_t samples = 0;

    for (size_t i = 0; i < len; i += step) {
        uint8_t v = data[i];
        checksum ^= v;
        checksum *= 16777619U;
        sum += v;
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
        samples++;
    }

    st.checksum = checksum;
    st.avg = samples ? (uint32_t)(sum / samples) : 0;
    st.min = min_v;
    st.max = max_v;
    st.samples = samples;
    return st;
}

static void free_user_buffers(void)
{
    for (int i = 0; i < CAPTURE_BUF_NUM; i++) {
        if (s_camera_outbuf[i]) {
            heap_caps_free(s_camera_outbuf[i]);
            s_camera_outbuf[i] = NULL;
            s_camera_buf_size[i] = 0;
        }
    }
}

static esp_err_t save_frame_to_sd(const uint8_t *data,
                                  size_t len,
                                  uint32_t frame_count,
                                  uint32_t width,
                                  uint32_t height,
                                  frame_stat_t st)
{
    if (data == NULL || len == 0) {
        return ESP_FAIL;
    }

    if (s_saved_frame_count >= SAVE_FRAME_MAX_COUNT) {
        return ESP_OK;
    }

    /* 第 1 帧保存一次，后面每隔 SAVE_FRAME_INTERVAL 帧再保存，避免连续写卡。 */
    if (!(frame_count == 1 || (frame_count % SAVE_FRAME_INTERVAL) == 0)) {
        return ESP_OK;
    }

    s_saved_frame_count++;

    char frame_path[32];
    snprintf(frame_path, sizeof(frame_path), "/sdcard/F%04" PRIu32 ".YUV", s_saved_frame_count);

    ESP_LOGI(mipi_cam_tag, "Saving frame to TF card: %s, bytes=%u", frame_path, (unsigned int)len);

    FILE *f = fopen(frame_path, "wb");
    if (f == NULL) {
        ESP_LOGE(mipi_cam_tag, "open frame file failed: %s", frame_path);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, len, f);
    fclose(f);

    if (written != len) {
        ESP_LOGE(mipi_cam_tag, "write frame file short: written=%u expected=%u",
                 (unsigned int)written, (unsigned int)len);
        return ESP_FAIL;
    }

    ESP_LOGI(mipi_cam_tag, "Frame saved OK: %s", frame_path);

    bool need_header = (s_saved_frame_count == 1);
    FILE *meta = fopen(SD_FRAME_META_PATH, need_header ? "w" : "a");
    if (meta != NULL) {
        if (need_header) {
            fprintf(meta, "save_index,source_frame,width,height,bytes,checksum,avg,min,max,file\n");
        }
        fprintf(meta, "%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%u,0x%08" PRIx32 ",%" PRIu32 ",%u,%u,%s\n",
                s_saved_frame_count,
                frame_count,
                width,
                height,
                (unsigned int)len,
                st.checksum,
                st.avg,
                (unsigned int)st.min,
                (unsigned int)st.max,
                frame_path);
        fclose(meta);
        ESP_LOGI(mipi_cam_tag, "Meta updated: %s", SD_FRAME_META_PATH);
    } else {
        ESP_LOGW(mipi_cam_tag, "open meta csv failed: %s", SD_FRAME_META_PATH);
    }

    return ESP_OK;
}


static void ai_snapshot_request_init_once(void)
{
    if (s_ai_snapshot_req.mutex == NULL) {
        s_ai_snapshot_req.mutex = xSemaphoreCreateMutex();
        if (s_ai_snapshot_req.mutex == NULL) {
            ESP_LOGE(mipi_cam_tag, "create AI snapshot mutex failed");
        }
    }
}

static esp_err_t ensure_jpeg_encoder(void)
{
    if (s_jpeg_encoder != NULL) {
        return ESP_OK;
    }

    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .intr_priority = 0,
        .timeout_ms = 200,
    };

    esp_err_t ret = jpeg_new_encoder_engine(&encode_eng_cfg, &s_jpeg_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(mipi_cam_tag, "jpeg_new_encoder_engine failed: %s", esp_err_to_name(ret));
        s_jpeg_encoder = NULL;
        return ret;
    }

    ESP_LOGI(mipi_cam_tag, "JPEG encoder engine ready for AI snapshots");
    return ESP_OK;
}

static void downsample_yuv422_packed_to_gray(const uint8_t *src,
                                              size_t src_len,
                                              uint32_t src_w,
                                              uint32_t src_h,
                                              uint8_t *dst_gray,
                                              uint32_t dst_w,
                                              uint32_t dst_h)
{
    if (!src || !dst_gray || src_w == 0 || src_h == 0 || dst_w == 0 || dst_h == 0) {
        return;
    }

    /*
     * 当前 OV5645/ESP32-P4 采集到的 640x480 YUV422 数据实际按 packed 形式排列，
     * 每个像素约 2 字节，亮度 Y 位于偶数字节。上一版按连续 Y 平面读取，
     * 会把色度字节也当成亮度，生成竖条纹灰图。
     */
    for (uint32_t y = 0; y < dst_h; y++) {
        uint32_t sy = (uint32_t)(((uint64_t)y * src_h) / dst_h);
        if (sy >= src_h) {
            sy = src_h - 1;
        }
        uint8_t *dst_line = dst_gray + y * dst_w;

        for (uint32_t x = 0; x < dst_w; x++) {
            uint32_t sx = (uint32_t)(((uint64_t)x * src_w) / dst_w);
            if (sx >= src_w) {
                sx = src_w - 1;
            }

            size_t pixel_index = (size_t)sy * (size_t)src_w + (size_t)sx;
            size_t y_index = pixel_index * 2;
            if (y_index < src_len) {
                dst_line[x] = src[y_index];
            } else {
                dst_line[x] = 128;
            }
        }
    }
}

static esp_err_t save_current_frame_as_ai_jpg(const uint8_t *frame,
                                              size_t frame_len,
                                              uint32_t src_w,
                                              uint32_t src_h,
                                              const char *path,
                                              uint32_t source_frame)
{
    if (!frame || frame_len == 0 || !path || src_w == 0 || src_h == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t packed_need = (size_t)src_w * (size_t)src_h * 2;
    if (frame_len < packed_need) {
        ESP_LOGE(mipi_cam_tag, "frame too small for packed YUV422: len=%u need=%u",
                 (unsigned)frame_len, (unsigned)packed_need);
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret = ensure_jpeg_encoder();
    if (ret != ESP_OK) {
        return ret;
    }

    const uint32_t jpg_w = AI_SNAPSHOT_WIDTH;
    const uint32_t jpg_h = AI_SNAPSHOT_HEIGHT;
    const size_t gray_size = (size_t)jpg_w * (size_t)jpg_h;
    const size_t jpg_buf_size = gray_size;  /* 灰度图通常远小于 raw，预留 raw 大小足够。 */

    jpeg_encode_memory_alloc_cfg_t in_mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER,
    };
    jpeg_encode_memory_alloc_cfg_t out_mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };

    size_t actual_in_size = 0;
    size_t actual_out_size = 0;
    uint8_t *gray = (uint8_t *)jpeg_alloc_encoder_mem(gray_size, &in_mem_cfg, &actual_in_size);
    uint8_t *jpg = (uint8_t *)jpeg_alloc_encoder_mem(jpg_buf_size, &out_mem_cfg, &actual_out_size);

    if (!gray || !jpg) {
        ESP_LOGE(mipi_cam_tag, "alloc JPEG buffers failed: gray=%p jpg=%p", gray, jpg);
        if (gray) free(gray);
        if (jpg) free(jpg);
        return ESP_ERR_NO_MEM;
    }

    downsample_yuv422_packed_to_gray(frame, frame_len, src_w, src_h, gray, jpg_w, jpg_h);

    jpeg_encode_cfg_t enc_cfg = {
        .src_type = JPEG_ENCODE_IN_FORMAT_GRAY,
        .sub_sample = JPEG_DOWN_SAMPLING_GRAY,
        .image_quality = AI_SNAPSHOT_JPEG_QUALITY,
        .width = jpg_w,
        .height = jpg_h,
    };

    uint32_t jpg_size = 0;
    ret = jpeg_encoder_process(s_jpeg_encoder,
                               &enc_cfg,
                               gray,
                               (uint32_t)gray_size,
                               jpg,
                               (uint32_t)jpg_buf_size,
                               &jpg_size);

    if (ret != ESP_OK || jpg_size == 0 || jpg_size > jpg_buf_size) {
        ESP_LOGE(mipi_cam_tag, "jpeg_encoder_process failed: %s, out=%u/%u",
                 esp_err_to_name(ret), (unsigned)jpg_size, (unsigned)jpg_buf_size);
        free(gray);
        free(jpg);
        return (ret != ESP_OK) ? ret : ESP_FAIL;
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(mipi_cam_tag, "open AI snapshot failed: %s", path);
        free(gray);
        free(jpg);
        return ESP_FAIL;
    }

    size_t written = fwrite(jpg, 1, jpg_size, f);
    fclose(f);

    free(gray);
    free(jpg);

    if (written != jpg_size) {
        ESP_LOGE(mipi_cam_tag, "write AI snapshot short: %u/%u path=%s",
                 (unsigned)written, (unsigned)jpg_size, path);
        return ESP_FAIL;
    }

    ESP_LOGI(mipi_cam_tag,
             "AI snapshot saved: %s, source_frame=%" PRIu32 ", %ux%u packed-yuv422 -> %ux%u gray jpg, size=%u bytes",
             path,
             source_frame,
             (unsigned)src_w,
             (unsigned)src_h,
             (unsigned)jpg_w,
             (unsigned)jpg_h,
             (unsigned)jpg_size);

    return ESP_OK;
}

static void handle_ai_snapshot_request_if_needed(const uint8_t *frame,
                                                 size_t frame_len,
                                                 uint32_t src_w,
                                                 uint32_t src_h,
                                                 uint32_t frame_count)
{
    ai_snapshot_request_init_once();
    if (s_ai_snapshot_req.mutex == NULL) {
        return;
    }

    char path[AI_SNAPSHOT_PATH_MAX] = {0};
    bool should_capture = false;

    if (xSemaphoreTake(s_ai_snapshot_req.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (s_ai_snapshot_req.pending && !s_ai_snapshot_req.done) {
            snprintf(path, sizeof(path), "%s", s_ai_snapshot_req.path);
            should_capture = true;
        }
        xSemaphoreGive(s_ai_snapshot_req.mutex);
    }

    if (!should_capture) {
        return;
    }

    esp_err_t ret = save_current_frame_as_ai_jpg(frame, frame_len, src_w, src_h, path, frame_count);

    if (xSemaphoreTake(s_ai_snapshot_req.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (s_ai_snapshot_req.pending && strcmp(s_ai_snapshot_req.path, path) == 0) {
            s_ai_snapshot_req.result = ret;
            s_ai_snapshot_req.source_frame = frame_count;
            s_ai_snapshot_req.done = true;
            s_ai_snapshot_req.pending = false;
        }
        xSemaphoreGive(s_ai_snapshot_req.mutex);
    }
}

esp_err_t mipi_cam_save_snapshot_jpg(const char *path, uint32_t timeout_ms)
{
    if (!path || path[0] == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_video_cam_fd < 0) {
        ESP_LOGE(mipi_cam_tag, "camera not ready, cannot save AI snapshot");
        return ESP_ERR_INVALID_STATE;
    }

    ai_snapshot_request_init_once();
    if (s_ai_snapshot_req.mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (timeout_ms == 0) {
        timeout_ms = AI_SNAPSHOT_TIMEOUT_MS;
    }

    if (xSemaphoreTake(s_ai_snapshot_req.mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGE(mipi_cam_tag, "take snapshot mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (s_ai_snapshot_req.pending && !s_ai_snapshot_req.done) {
        xSemaphoreGive(s_ai_snapshot_req.mutex);
        ESP_LOGW(mipi_cam_tag, "AI snapshot request busy");
        return ESP_ERR_INVALID_STATE;
    }

    snprintf(s_ai_snapshot_req.path, sizeof(s_ai_snapshot_req.path), "%s", path);
    s_ai_snapshot_req.pending = true;
    s_ai_snapshot_req.done = false;
    s_ai_snapshot_req.result = ESP_ERR_TIMEOUT;
    s_ai_snapshot_req.source_frame = 0;
    xSemaphoreGive(s_ai_snapshot_req.mutex);

    ESP_LOGI(mipi_cam_tag, "AI snapshot requested: %s", path);

    int waited_ms = 0;
    while (waited_ms < (int)timeout_ms) {
        bool done = false;
        esp_err_t result = ESP_ERR_TIMEOUT;
        uint32_t src_frame = 0;

        if (xSemaphoreTake(s_ai_snapshot_req.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            done = s_ai_snapshot_req.done;
            result = s_ai_snapshot_req.result;
            src_frame = s_ai_snapshot_req.source_frame;
            xSemaphoreGive(s_ai_snapshot_req.mutex);
        }

        if (done) {
            ESP_LOGI(mipi_cam_tag, "AI snapshot request done: %s, source_frame=%" PRIu32 ", result=%s",
                     path, src_frame, esp_err_to_name(result));
            return result;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        waited_ms += 50;
    }

    if (xSemaphoreTake(s_ai_snapshot_req.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (s_ai_snapshot_req.pending && strcmp(s_ai_snapshot_req.path, path) == 0) {
            s_ai_snapshot_req.pending = false;
            s_ai_snapshot_req.done = true;
            s_ai_snapshot_req.result = ESP_ERR_TIMEOUT;
        }
        xSemaphoreGive(s_ai_snapshot_req.mutex);
    }

    ESP_LOGE(mipi_cam_tag, "AI snapshot request timeout: %s", path);
    return ESP_ERR_TIMEOUT;
}

static void mipi_cam_capture_task(void *arg)
{
    int video_fd = *(int *)arg;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    uint32_t frame_count = 0;
    int64_t start_time_us = esp_timer_get_time();
    int64_t last_wait_log_us = start_time_us;

    memset(&format, 0, sizeof(format));
    format.type = type;
    if (ioctl(video_fd, VIDIOC_G_FMT, &format) != 0) {
        ESP_LOGE(mipi_cam_tag, "get fmt failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(mipi_cam_tag,
             "capture format: width=%" PRIu32 ", height=%" PRIu32 ", pixfmt=0x%08" PRIx32,
             format.fmt.pix.width,
             format.fmt.pix.height,
             format.fmt.pix.pixelformat);

    memset(&req, 0, sizeof(req));
    req.count = CAPTURE_BUF_NUM;
    req.type = type;
    req.memory = V4L2_MEMORY_USERPTR;

    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(mipi_cam_tag, "VIDIOC_REQBUFS USERPTR failed, errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(mipi_cam_tag, "VIDIOC_REQBUFS ok, requested=%d, actual=%" PRIu32, CAPTURE_BUF_NUM, req.count);

    for (int i = 0; i < CAPTURE_BUF_NUM; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;

        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(mipi_cam_tag, "VIDIOC_QUERYBUF index=%d failed, errno=%d", i, errno);
            free_user_buffers();
            vTaskDelete(NULL);
            return;
        }

        s_camera_buf_size[i] = buf.length;
        s_camera_outbuf[i] = (uint8_t *)heap_caps_aligned_alloc(
            MEMORY_ALIGN,
            buf.length,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED | MALLOC_CAP_8BIT
        );

        if (!s_camera_outbuf[i]) {
            ESP_LOGE(mipi_cam_tag, "alloc PSRAM user buffer failed, index=%d, len=%" PRIu32, i, buf.length);
            free_user_buffers();
            vTaskDelete(NULL);
            return;
        }

        ESP_LOGI(mipi_cam_tag, "user buffer[%d]: addr=%p, len=%" PRIu32,
                 i, s_camera_outbuf[i], s_camera_buf_size[i]);

        buf.m.userptr = (unsigned long)s_camera_outbuf[i];
        buf.length = s_camera_buf_size[i];

        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(mipi_cam_tag, "VIDIOC_QBUF index=%d failed, errno=%d", i, errno);
            free_user_buffers();
            vTaskDelete(NULL);
            return;
        }
    }

    if (camera_stream_start(video_fd) != ESP_OK) {
        ESP_LOGE(mipi_cam_tag, "camera_stream_start failed");
        free_user_buffers();
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(mipi_cam_tag, "camera stream started, begin USERPTR frame capture + SD save test");

    int flags = fcntl(video_fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(video_fd, F_SETFL, flags | O_NONBLOCK);
    }

    while (1) {
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = V4L2_MEMORY_USERPTR;

        int res = ioctl(video_fd, VIDIOC_DQBUF, &buf);
        if (res != 0) {
            int err = errno;
            int64_t now = esp_timer_get_time();
            if (now - last_wait_log_us > 2000000) {
                ESP_LOGW(mipi_cam_tag,
                         "waiting for frame... VIDIOC_DQBUF ret=%d errno=%d, elapsed=%lld ms",
                         res,
                         err,
                         (long long)((now - start_time_us) / 1000));
                last_wait_log_us = now;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        frame_count++;
        size_t frame_len = buf.bytesused;
        if (frame_len == 0 && buf.index < CAPTURE_BUF_NUM) {
            frame_len = s_camera_buf_size[buf.index];
        }

        frame_stat_t st = {0};
        if (buf.index < CAPTURE_BUF_NUM) {
            st = frame_analyze_sampled(s_camera_outbuf[buf.index], frame_len);
        }

        if (frame_count <= 10 || (frame_count % FRAME_LOG_INTERVAL) == 0) {
            int64_t now = esp_timer_get_time();
            float fps = 0.0f;
            if (now > start_time_us) {
                fps = frame_count * 1000000.0f / (float)(now - start_time_us);
            }

            ESP_LOGI(mipi_cam_tag,
                     "frame captured: count=%" PRIu32 ", index=%" PRIu32 ", bytes=%u, addr=%p, checksum=0x%08" PRIx32 ", avg=%" PRIu32 ", min=%u, max=%u, samples=%u, fps=%.2f",
                     frame_count,
                     buf.index,
                     (unsigned int)frame_len,
                     (buf.index < CAPTURE_BUF_NUM) ? s_camera_outbuf[buf.index] : NULL,
                     st.checksum,
                     st.avg,
                     (unsigned int)st.min,
                     (unsigned int)st.max,
                     (unsigned int)st.samples,
                     fps);
        }

        if (buf.index < CAPTURE_BUF_NUM) {
            (void)save_frame_to_sd(s_camera_outbuf[buf.index],
                                   frame_len,
                                   frame_count,
                                   format.fmt.pix.width,
                                   format.fmt.pix.height,
                                   st);

            handle_ai_snapshot_request_if_needed(s_camera_outbuf[buf.index],
                                                 frame_len,
                                                 format.fmt.pix.width,
                                                 format.fmt.pix.height,
                                                 frame_count);
        }

        if (buf.index < CAPTURE_BUF_NUM) {
            buf.m.userptr = (unsigned long)s_camera_outbuf[buf.index];
            buf.length = s_camera_buf_size[buf.index];
        }

        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(mipi_cam_tag, "VIDIOC_QBUF return buffer failed, errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

esp_err_t mipi_cam_init(void)
{
    esp_err_t ret = ESP_OK;

    /* 保留 MIPI 设备/PHY 电源初始化；不初始化 LCD 显示。 */
    mipi_dev_bsp_enable_dsi_phy_power();

    ret = app_video_main(bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(mipi_cam_tag, "video main init failed with error 0x%x", ret);
        return ESP_FAIL;
    }

    s_video_cam_fd = app_video_open(0);
    if (s_video_cam_fd < 0) {
        ESP_LOGE(mipi_cam_tag, "video cam open failed");
        return ESP_FAIL;
    }

    ret = app_video_init(s_video_cam_fd, APP_VIDEO_FMT_YUV422);
    if (ret != ESP_OK) {
        ESP_LOGE(mipi_cam_tag, "Video cam init failed with error 0x%x", ret);
        close(s_video_cam_fd);
        s_video_cam_fd = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(mipi_cam_tag, "OV5645 MIPI camera init success, start USERPTR capture task");

    BaseType_t task_ret = xTaskCreatePinnedToCore(
        mipi_cam_capture_task,
        "mipi_cam_capture",
        10240,
        &s_video_cam_fd,
        5,
        NULL,
        1
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(mipi_cam_tag, "create mipi_cam_capture task failed");
        close(s_video_cam_fd);
        s_video_cam_fd = -1;
        return ESP_FAIL;
    }

    return ESP_OK;
}
