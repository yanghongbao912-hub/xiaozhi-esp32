#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
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
    // Clean-pin mapping: MOSI=38, SCK=21, CS=47, DC=3, RST=45
    // GPIO13 & GPIO48 pulled LOW: onboard RGB LED disabled
    const int mosi = 38, sck = 21, cs = 47, dc = 3, rst = 45;

    gpio_config_t off_cfg = {};
    off_cfg.pin_bit_mask = (1ULL << 13) | (1ULL << 48);
    off_cfg.mode = GPIO_MODE_OUTPUT;
    off_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&off_cfg);
    gpio_set_level(GPIO_NUM_13, 0);
    gpio_set_level(GPIO_NUM_48, 0);
    ESP_LOGI(TAG, "onboard RGB LED pins 13/48 forced LOW (LED off)");

    while (1) {
        for (int it = 0; it < 2; it++) {  // 0=GC9D01-160, 1=GC9A01-240
            int w = (it == 0) ? 160 : 240;
            int h = (it == 0) ? 160 : 240;
            uint16_t color = (it == 0) ? 0xF800 : 0x07E0;  // RED, GREEN
            const char *cname = (it == 0) ? "RED" : "GREEN";
            const char *iname = (it == 0) ? "GC9D01-160" : "GC9A01-240";

            spi_bus_free(SPI3_HOST);
            spi_bus_config_t buscfg = {};
            buscfg.mosi_io_num = mosi;
            buscfg.miso_io_num = -1;
            buscfg.sclk_io_num = sck;
            buscfg.quadwp_io_num = -1;
            buscfg.quadhd_io_num = -1;
            buscfg.max_transfer_sz = w * h * 2;
            if (spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
                ESP_LOGE(TAG, "bus fail");
                continue;
            }
            esp_lcd_panel_io_handle_t io = NULL;
            esp_lcd_panel_handle_t panel = NULL;
            esp_lcd_panel_io_spi_config_t io_cfg = {};
            io_cfg.cs_gpio_num = cs;
            io_cfg.dc_gpio_num = dc;
            io_cfg.spi_mode = 0;
            io_cfg.pclk_hz = 20 * 1000 * 1000;
            io_cfg.trans_queue_depth = 10;
            io_cfg.lcd_cmd_bits = 8;
            io_cfg.lcd_param_bits = 8;
            if (esp_lcd_new_panel_io_spi(SPI3_HOST, &io_cfg, &io) != ESP_OK) {
                ESP_LOGE(TAG, "io fail");
                continue;
            }
            esp_lcd_panel_dev_config_t pcfg = {};
            pcfg.reset_gpio_num = rst;
            pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
            pcfg.bits_per_pixel = 16;
            if (it == 0) {
                if (esp_lcd_new_panel_gc9d01(io, &pcfg, &panel) != ESP_OK) {
                    ESP_LOGE(TAG, "gc9d01 panel fail");
                    esp_lcd_panel_io_del(io);
                    continue;
                }
            } else {
                if (esp_lcd_new_panel_gc9a01(io, &pcfg, &panel) != ESP_OK) {
                    ESP_LOGE(TAG, "gc9a01 panel fail");
                    esp_lcd_panel_io_del(io);
                    continue;
                }
            }
            esp_lcd_panel_reset(panel);
            esp_lcd_panel_init(panel);
            ESP_LOGI(TAG, "clean-pin test: MOSI=%d SCK=%d CS=%d DC=%d RST=%d init=%s -> %s",
                     mosi, sck, cs, dc, rst, iname, cname);
            draw_solid(panel, color, w, h);
            vTaskDelay(pdMS_TO_TICKS(6000));
            esp_lcd_panel_del(panel);
            esp_lcd_panel_io_del(io);
        }
    }
}

void eye_test_permutation_start(void) {
    ESP_LOGI(TAG, "starting clean-pin test (LED off, MOSI=38, DC=3)");
    xTaskCreate(test_task, "eye_test", 4096, NULL, 2, NULL);
}
