#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "eye_test.h"

#define EYE_W 160
#define EYE_H 160

static const char *TAG = "eye_test";

// Render one eye frame into buf (big-endian RGB565 bytes, as the panel expects)
static void draw_eye(uint8_t *buf, int cx, int cy, int eyelid) {
    for (int y = 0; y < EYE_H; y++) {
        for (int x = 0; x < EYE_W; x++) {
            uint16_t color = 0x0000;  // black background
            int dxs = x - 80, dys = y - 84;
            if (dxs * dxs + dys * dys <= 68 * 68) {
                color = 0xFFFF;  // white sclera
                int dxi = x - cx, dyi = y - cy;
                int r2 = dxi * dxi + dyi * dyi;
                if (r2 <= 31 * 31) {
                    // blue iris with radial gradient
                    int d = (int)sqrt((double)r2);
                    uint8_t b = (uint8_t)(0x1F - d / 2);
                    uint8_t g = (uint8_t)(0x28 - d);
                    uint8_t r = (uint8_t)(0x08 - d / 8);
                    color = (uint16_t)((r << 11) | (g << 5) | b);
                    if (r2 <= 15 * 15) {
                        color = 0x0000;  // pupil
                        int hx = x - (cx - 6), hy = y - (cy - 8);
                        if (hx * hx + hy * hy <= 3 * 3) {
                            color = 0xFFFF;  // highlight
                        }
                    }
                }
            }
            if (y < eyelid) {
                color = 0x0000;  // eyelid (blink)
            }
            buf[(y * EYE_W + x) * 2] = (uint8_t)(color >> 8);
            buf[(y * EYE_W + x) * 2 + 1] = (uint8_t)(color & 0xFF);
        }
    }
}

static void eye_test_task(void *arg) {
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)arg;
    uint8_t *buf = (uint8_t *)heap_caps_malloc(EYE_W * EYE_H * 2, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "no DMA memory for eye buffer");
        vTaskDelete(NULL);
        return;
    }
    uint32_t frame = 0;
    while (1) {
        // pupil scans left-right with slight vertical sway
        int dx = (int)(16.0 * sin(frame * 0.12));
        int dy = (int)(5.0 * sin(frame * 0.06));
        // blink: close + open every 200 frames (~14s)
        int eyelid = 0;
        int phase = (int)(frame % 200);
        if (phase < 14) {
            eyelid = phase * 5;
        } else if (phase < 28) {
            eyelid = (28 - phase) * 5;
        }
        draw_eye(buf, 80 + dx, 84 + dy, eyelid);
        esp_lcd_panel_draw_bitmap(panel, 0, 0, EYE_W, EYE_H, buf);
        frame++;
        vTaskDelay(pdMS_TO_TICKS(70));
    }
}

void eye_test_start(esp_lcd_panel_handle_t panel) {
    ESP_LOGI(TAG, "starting eye animation test");
    xTaskCreate(eye_test_task, "eye_test", 4096, panel, 2, NULL);
}
