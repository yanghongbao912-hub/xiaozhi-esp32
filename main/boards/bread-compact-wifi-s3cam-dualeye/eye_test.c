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

static void permutation_task(void *arg) {
    // {RST, DC, CS}
    static const int combos[6][3] = {
        {45, 48, 47},
        {45, 47, 48},
        {48, 45, 47},
        {48, 47, 45},
        {47, 45, 48},
        {47, 48, 45},
    };
    static const uint16_t colors[6] = {0xF800, 0x07E0, 0x001F, 0xFFFF, 0xFFE0, 0x07FF};
    static const char *names[6] = {"RED", "GREEN", "BLUE", "WHITE", "YELLOW", "CYAN"};

    while (1) {
        for (int i = 0; i < 6; i++) {
            esp_lcd_panel_io_handle_t io = NULL;
            esp_lcd_panel_handle_t panel = NULL;

            esp_lcd_panel_io_spi_config_t io_cfg = {};
            io_cfg.cs_gpio_num = combos[i][2];
            io_cfg.dc_gpio_num = combos[i][1];
            io_cfg.spi_mode = 0;
            io_cfg.pclk_hz = 40 * 1000 * 1000;
            io_cfg.trans_queue_depth = 10;
            io_cfg.lcd_cmd_bits = 8;
            io_cfg.lcd_param_bits = 8;
            if (esp_lcd_new_panel_io_spi(SPI3_HOST, &io_cfg, &io) != ESP_OK) {
                ESP_LOGE(TAG, "combo %d: io fail", i + 1);
                continue;
            }
            esp_lcd_panel_dev_config_t pcfg = {};
            pcfg.reset_gpio_num = combos[i][0];
            pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
            pcfg.bits_per_pixel = 16;
            if (esp_lcd_new_panel_gc9d01(io, &pcfg, &panel) != ESP_OK) {
                ESP_LOGE(TAG, "combo %d: panel fail", i + 1);
                esp_lcd_panel_io_del(io);
                continue;
            }
            esp_lcd_panel_reset(panel);
            esp_lcd_panel_init(panel);
            // no invert: solid colors should appear as-is
            ESP_LOGI(TAG, "combo %d/%d: RST=%d DC=%d CS=%d -> %s",
                     i + 1, 6, combos[i][0], combos[i][1], combos[i][2], names[i]);
            draw_solid(panel, colors[i]);
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_lcd_panel_del(panel);
            esp_lcd_panel_io_del(io);
        }
    }
}

void eye_test_permutation_start(void) {
    ESP_LOGI(TAG, "starting RST/DC/CS permutation test");
    xTaskCreate(permutation_task, "eye_perm", 4096, NULL, 2, NULL);
}
