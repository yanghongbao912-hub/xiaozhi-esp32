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

static void test_task(void *arg) {
    // mappings: {SCK, MOSI, CS, DC, RST}
    static const int maps[4][5] = {
        {21, 13, 47, 48, 45},  // ① v3 (user's current)
        {48, 13, 21, 47, 45},  // ② user
        {13, 47, 21, 48, 45},  // ③ user (MOSI=47!)
        {48, 13, 45, 47, 21},  // ④ seller
    };
    static const char *mnames[4] = {"①v3", "②user", "③user", "④seller"};
    // init: 0=GC9A01(240x240), 1=GC9D01(160x160)
    static const char *inits[2] = {"GC9A01-240", "GC9D01-160"};

    while (1) {
        int combo = 0;
        for (int m = 0; m < 4; m++) {
            for (int it = 0; it < 2; it++) {
                combo++;
                int sck = maps[m][0], mosi = maps[m][1], cs = maps[m][2], dc = maps[m][3], rst = maps[m][4];
                int w = (it == 0) ? 240 : 160;
                int h = (it == 0) ? 240 : 160;
                uint16_t color;
                switch (combo) {
                case 1: color = 0xF800; break;  // RED
                case 2: color = 0x07E0; break;  // GREEN
                case 3: color = 0x001F; break;  // BLUE
                case 4: color = 0xFFFF; break;  // WHITE
                case 5: color = 0xFFE0; break;  // YELLOW
                case 6: color = 0x07FF; break;  // CYAN
                case 7: color = 0xF81F; break;  // MAGENTA
                default: color = 0xFD20; break; // ORANGE
                }
                const char *cname;
                switch (combo) {
                case 1: cname = "RED"; break;
                case 2: cname = "GREEN"; break;
                case 3: cname = "BLUE"; break;
                case 4: cname = "WHITE"; break;
                case 5: cname = "YELLOW"; break;
                case 6: cname = "CYAN"; break;
                case 7: cname = "MAGENTA"; break;
                default: cname = "ORANGE"; break;
                }

                spi_bus_free(SPI3_HOST);
                spi_bus_config_t buscfg = {};
                buscfg.mosi_io_num = mosi;
                buscfg.miso_io_num = -1;
                buscfg.sclk_io_num = sck;
                buscfg.quadwp_io_num = -1;
                buscfg.quadhd_io_num = -1;
                buscfg.max_transfer_sz = w * h * 2;
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
                io_cfg.pclk_hz = 20 * 1000 * 1000;  // 20MHz: 杜邦线信号完整性
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
                if (it == 0) {
                    if (esp_lcd_new_panel_gc9a01(io, &pcfg, &panel) != ESP_OK) {
                        ESP_LOGE(TAG, "combo %d: gc9a01 panel fail", combo);
                        esp_lcd_panel_io_del(io);
                        continue;
                    }
                } else {
                    if (esp_lcd_new_panel_gc9d01(io, &pcfg, &panel) != ESP_OK) {
                        ESP_LOGE(TAG, "combo %d: gc9d01 panel fail", combo);
                        esp_lcd_panel_io_del(io);
                        continue;
                    }
                }
                esp_lcd_panel_reset(panel);
                esp_lcd_panel_init(panel);
                ESP_LOGI(TAG, "combo %d/8: %s map: SCK=%d MOSI=%d CS=%d DC=%d RST=%d init=%s -> %s",
                         combo, mnames[m], sck, mosi, cs, dc, rst, inits[it], cname);
                draw_solid(panel, color, w, h);
                vTaskDelay(pdMS_TO_TICKS(5000));
                esp_lcd_panel_del(panel);
                esp_lcd_panel_io_del(io);
            }
        }
    }
}

void eye_test_permutation_start(void) {
    ESP_LOGI(TAG, "starting 8-combo test @20MHz (4 mappings x GC9A01-240/GC9D01-160)");
    xTaskCreate(test_task, "eye_test", 4096, NULL, 2, NULL);
}
