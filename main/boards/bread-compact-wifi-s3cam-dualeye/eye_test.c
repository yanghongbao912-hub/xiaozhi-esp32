#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_common.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_gc9d01.h"
#include "eye_test.h"

#define EYE_W 160
#define EYE_H 160

static const char *TAG = "eye_test";

static void draw_solid(esp_lcd_panel_handle_t panel, uint16_t color, int w, int h) {
    uint8_t *buf = (uint8_t *)heap_caps_malloc(w * h * 2, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "no DMA mem");
        return;
    }
    for (int p = 0; p < w * h; p++) {
        buf[p * 2] = (uint8_t)(color >> 8);
        buf[p * 2 + 1] = (uint8_t)(color & 0xFF);
    }
    esp_lcd_panel_draw_bitmap(panel, 0, 0, w, h, buf);
    heap_caps_free(buf);
}

static void dualinit_task(void *arg) {
    // {SCK, RST, DC, CS, init_type}  init_type: 0=GC9A01(240x240), 1=GC9D01(160x160)
    static const int combos[4][5] = {
        {21, 45, 48, 47, 0},  // v3 mapping + GC9A01 init
        {21, 45, 48, 47, 1},  // v3 mapping + GC9D01 init
        {48, 21, 47, 45, 0},  // seller mapping + GC9A01 init
        {48, 21, 47, 45, 1},  // seller mapping + GC9D01 init
    };
    static const uint16_t colors[4] = {0xF800, 0x07E0, 0x001F, 0xFFFF};
    static const char *names[4] = {"RED", "GREEN", "BLUE", "WHITE"};
    static const char *inits[2] = {"GC9A01-240", "GC9D01-160"};

    while (1) {
        for (int i = 0; i < 4; i++) {
            int sck = combos[i][0], rst = combos[i][1], dc = combos[i][2], cs = combos[i][3], itype = combos[i][4];
            int w = (itype == 0) ? 240 : 160;
            int h = (itype == 0) ? 240 : 160;

            spi_bus_free(SPI3_HOST);
            spi_bus_config_t buscfg = {};
            buscfg.mosi_io_num = 13;
            buscfg.miso_io_num = -1;
            buscfg.sclk_io_num = sck;
            buscfg.quadwp_io_num = -1;
            buscfg.quadhd_io_num = -1;
            buscfg.max_transfer_sz = w * h * 2;
            if (spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
                ESP_LOGE(TAG, "combo %d: bus fail", i + 1);
                continue;
            }
            esp_lcd_panel_io_handle_t io = NULL;
            esp_lcd_panel_handle_t panel = NULL;
            esp_lcd_panel_io_spi_config_t io_cfg = {};
            io_cfg.cs_gpio_num = cs;
            io_cfg.dc_gpio_num = dc;
            io_cfg.spi_mode = 0;
            io_cfg.pclk_hz = 20 * 1000 * 1000;  // 20MHz: 杜邦线信号完整性
            io_cfg.trans_queue_depth = 10;
            io_cfg.lcd_cmd_bits = 8;
            io_cfg.lcd_param_bits = 8;
            if (esp_lcd_new_panel_io_spi(SPI3_HOST, &io_cfg, &io) != ESP_OK) {
                ESP_LOGE(TAG, "combo %d: io fail", i + 1);
                continue;
            }
            esp_lcd_panel_dev_config_t pcfg = {};
            pcfg.reset_gpio_num = rst;
            pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
            pcfg.bits_per_pixel = 16;
            if (itype == 0) {
                if (esp_lcd_new_panel_gc9a01(io, &pcfg, &panel) != ESP_OK) {
                    ESP_LOGE(TAG, "combo %d: gc9a01 panel fail", i + 1);
                    esp_lcd_panel_io_del(io);
                    continue;
                }
            } else {
                if (esp_lcd_new_panel_gc9d01(io, &pcfg, &panel) != ESP_OK) {
                    ESP_LOGE(TAG, "combo %d: gc9d01 panel fail", i + 1);
                    esp_lcd_panel_io_del(io);
                    continue;
                }
            }
            esp_lcd_panel_reset(panel);
            esp_lcd_panel_init(panel);
            ESP_LOGI(TAG, "combo %d/4: SCK=%d RST=%d DC=%d CS=%d init=%s -> %s",
                     i + 1, sck, rst, dc, cs, inits[itype], names[i]);
            draw_solid(panel, colors[i], w, h);
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_lcd_panel_del(panel);
            esp_lcd_panel_io_del(io);
        }
    }
}

void eye_test_permutation_start(void) {
    ESP_LOGI(TAG, "starting dual-init test @20MHz (GC9A01-240 vs GC9D01-160)");
    xTaskCreate(dualinit_task, "eye_perm", 4096, NULL, 2, NULL);
}
