#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_common.h"
#include "esp_lcd_gc9d01.h"
#include "eye_test.h"

#define EYE_W 160
#define EYE_H 160

static const char *TAG = "eye_test";

// HSV -> RGB565
static uint16_t hsv_to_rgb565(int h, int s, int v) {
    int region = h / 60;
    int f = h % 60;
    int p = v * (255 - s) / 255;
    int q = v * (255 - s * f / 60) / 255;
    int t = v * (255 - s * (60 - f) / 60) / 255;
    int r, g, b;
    switch (region) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
    }
    uint16_t r5 = r >> 3, g6 = g >> 2, b5 = b >> 3;
    return (uint16_t)((r5 << 11) | (g6 << 5) | b5);
}

static void draw_solid(esp_lcd_panel_handle_t panel, uint16_t color) {
    uint8_t *buf = (uint8_t *)heap_caps_malloc(EYE_W * EYE_H * 2, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "no DMA mem");
        return;
    }
    for (int p = 0; p < EYE_W * EYE_H; p++) {
        buf[p * 2] = (uint8_t)(color >> 8);
        buf[p * 2 + 1] = (uint8_t)(color & 0xFF);
    }
    esp_lcd_panel_draw_bitmap(panel, 0, 0, EYE_W, EYE_H, buf);
    heap_caps_free(buf);
}

// next lexicographic permutation of a[0..3]; returns 0 when wrapped
static int next_perm(int *a) {
    int i = 3;
    while (i > 0 && a[i - 1] >= a[i]) i--;
    if (i == 0) return 0;
    int j = 3;
    while (a[j] <= a[i - 1]) j--;
    int t = a[i - 1]; a[i - 1] = a[j]; a[j] = t;
    for (int l = i, r = 3; l < r; l++, r--) {
        t = a[l]; a[l] = a[r]; a[r] = t;
    }
    return 1;
}

static void permutation_task(void *arg) {
    while (1) {
        int p[4] = {0, 1, 2, 3};  // index into pins[] = {SCK, RST, DC, CS}
        int pins[4] = {21, 45, 47, 48};
        int combo = 0;
        do {
            int sck = pins[p[0]], rst = pins[p[1]], dc = pins[p[2]], cs = pins[p[3]];
            combo++;
            uint16_t color = hsv_to_rgb565(combo * 15, 255, 255);

            // re-init SPI bus with this combo's SCK
            spi_bus_free(SPI3_HOST);
            spi_bus_config_t buscfg = {};
            buscfg.mosi_io_num = 13;
            buscfg.miso_io_num = -1;
            buscfg.sclk_io_num = sck;
            buscfg.quadwp_io_num = -1;
            buscfg.quadhd_io_num = -1;
            buscfg.max_transfer_sz = EYE_W * EYE_H * 2;
            if (spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
                ESP_LOGE(TAG, "combo %d: bus fail", combo);
                continue;
            }
            esp_lcd_panel_io_handle_t io = NULL;
            esp_lcd_panel_handle_t panel = NULL;
            esp_lcd_panel_io_spi_config_t io_cfg = {};
            io_cfg.cs_gpio_num = cs;
            io_cfg.dc_gpio_num = dc;
            io_cfg.spi_mode = 0;
            io_cfg.pclk_hz = 40 * 1000 * 1000;
            io_cfg.trans_queue_depth = 10;
            io_cfg.lcd_cmd_bits = 8;
            io_cfg.lcd_param_bits = 8;
            if (esp_lcd_new_panel_io_spi(SPI3_HOST, &io_cfg, &io) != ESP_OK) {
                ESP_LOGE(TAG, "combo %d: io fail", combo);
                continue;
            }
            esp_lcd_panel_dev_config_t pcfg = {};
            pcfg.reset_gpio_num = rst;
            pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
            pcfg.bits_per_pixel = 16;
            if (esp_lcd_new_panel_gc9d01(io, &pcfg, &panel) != ESP_OK) {
                ESP_LOGE(TAG, "combo %d: panel fail", combo);
                esp_lcd_panel_io_del(io);
                continue;
            }
            esp_lcd_panel_reset(panel);
            esp_lcd_panel_init(panel);
            ESP_LOGI(TAG, "combo %d/24: SCK=%d RST=%d DC=%d CS=%d",
                     combo, sck, rst, dc, cs);
            draw_solid(panel, color);
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_lcd_panel_del(panel);
            esp_lcd_panel_io_del(io);
        } while (next_perm(p));
    }
}

void eye_test_permutation_start(void) {
    ESP_LOGI(TAG, "starting 24-combo permutation test (MOSI=13 fixed)");
    xTaskCreate(permutation_task, "eye_perm", 4096, NULL, 2, NULL);
}
