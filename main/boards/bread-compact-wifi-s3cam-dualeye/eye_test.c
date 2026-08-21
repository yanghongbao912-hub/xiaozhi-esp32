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
#include "eye_test.h"

static const char *TAG = "eye_test";

// Waveshare 0.71inch DualEye (GC9D01) init sequence - same as bread-compact-wifi-lcd
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xFE, (uint8_t[]){}, 0, 0},
    {0xEF, (uint8_t[]){}, 0, 0},
    {0x80, (uint8_t[]){0xFF}, 1, 0},
    {0x81, (uint8_t[]){0xFF}, 1, 0},
    {0x82, (uint8_t[]){0xFF}, 1, 0},
    {0x83, (uint8_t[]){0xFF}, 1, 0},
    {0x84, (uint8_t[]){0xFF}, 1, 0},
    {0x85, (uint8_t[]){0xFF}, 1, 0},
    {0x86, (uint8_t[]){0xFF}, 1, 0},
    {0x87, (uint8_t[]){0xFF}, 1, 0},
    {0x88, (uint8_t[]){0xFF}, 1, 0},
    {0x89, (uint8_t[]){0xFF}, 1, 0},
    {0x8A, (uint8_t[]){0xFF}, 1, 0},
    {0x8B, (uint8_t[]){0xFF}, 1, 0},
    {0x8C, (uint8_t[]){0xFF}, 1, 0},
    {0x8D, (uint8_t[]){0xFF}, 1, 0},
    {0x8E, (uint8_t[]){0xFF}, 1, 0},
    {0x8F, (uint8_t[]){0xFF}, 1, 0},
    {0x3A, (uint8_t[]){0x05}, 1, 0},
    {0xEC, (uint8_t[]){0x01}, 1, 0},
    {0x74, (uint8_t[]){0x02, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00}, 7, 0},
    {0x98, (uint8_t[]){0x3E}, 1, 0},
    {0x99, (uint8_t[]){0x3E}, 1, 0},
    {0xB5, (uint8_t[]){0x0D, 0x0D}, 2, 0},
    {0x60, (uint8_t[]){0x38, 0x0F, 0x79, 0x67}, 4, 0},
    {0x61, (uint8_t[]){0x38, 0x11, 0x79, 0x67}, 4, 0},
    {0x64, (uint8_t[]){0x38, 0x17, 0x71, 0x5F, 0x79, 0x67}, 6, 0},
    {0x65, (uint8_t[]){0x38, 0x13, 0x71, 0x5B, 0x79, 0x67}, 6, 0},
    {0x6A, (uint8_t[]){0x00, 0x00}, 2, 0},
    {0x6C, (uint8_t[]){0x22, 0x02, 0x22, 0x02, 0x22, 0x22, 0x50}, 7, 0},
    {0x6E, (uint8_t[]){0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x0F, 0x0F, 0x0D, 0x0D, 0x0B, 0x0B, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x0A, 0x0C, 0x0C, 0x0E, 0x0E, 0x10, 0x10, 0x00, 0x00, 0x02, 0x02, 0x04, 0x04}, 32, 0},
    {0xBF, (uint8_t[]){0x01}, 1, 0},
    {0xF9, (uint8_t[]){0x40}, 1, 0},
    {0x9B, (uint8_t[]){0x3B}, 1, 0},
    {0x93, (uint8_t[]){0x33, 0x7F, 0x00}, 3, 0},
    {0x7E, (uint8_t[]){0x30}, 1, 0},
    {0x70, (uint8_t[]){0x0D, 0x02, 0x08, 0x0D, 0x02, 0x08}, 6, 0},
    {0x71, (uint8_t[]){0x0D, 0x02, 0x08}, 3, 0},
    {0x91, (uint8_t[]){0x0E, 0x09}, 2, 0},
    {0xC3, (uint8_t[]){0x19}, 1, 0},
    {0xC4, (uint8_t[]){0x19}, 1, 0},
    {0xC9, (uint8_t[]){0x3C}, 1, 0},
    {0xF0, (uint8_t[]){0x53, 0x15, 0x0A, 0x04, 0x00, 0x3E}, 6, 0},
    {0xF2, (uint8_t[]){0x53, 0x15, 0x0A, 0x04, 0x00, 0x3A}, 6, 0},
    {0xF1, (uint8_t[]){0x56, 0xA8, 0x7F, 0x33, 0x34, 0x5F}, 6, 0},
    {0xF3, (uint8_t[]){0x52, 0xA4, 0x7F, 0x33, 0x34, 0xDF}, 6, 0},
    {0x36, (uint8_t[]){0x00}, 1, 0},
    {0x11, (uint8_t[]){}, 0, 200},
    {0x29, (uint8_t[]){}, 0, 0},
    {0x2C, (uint8_t[]){}, 0, 0},
};

static void draw_solid(esp_lcd_panel_handle_t panel, uint16_t color, int x0, int y0, int w, int h) {
    uint8_t *buf = (uint8_t *)heap_caps_malloc(w * h * 2, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "no DMA mem for %dx%d", w, h);
        return;
    }
    for (int p = 0; p < w * h; p++) {
        buf[p * 2] = (uint8_t)(color >> 8);
        buf[p * 2 + 1] = (uint8_t)(color & 0xFF);
    }
    esp_lcd_panel_draw_bitmap(panel, x0, y0, x0 + w, y0 + h, buf);
    heap_caps_free(buf);
    ESP_LOGI(TAG, "drew %dx%d at (%d,%d) color=0x%04X", w, h, x0, y0, color);
}

static void test_task(void *arg) {
    // clean pins: MOSI=38, SCK=21, CS=47, DC=3, RST=45
    const int mosi = 38, sck = 21, cs = 47, dc = 3, rst = 45;

    // disable onboard RGB LED (GPIO13/48 low)
    gpio_config_t off_cfg = {};
    off_cfg.pin_bit_mask = (1ULL << 13) | (1ULL << 48);
    off_cfg.mode = GPIO_MODE_OUTPUT;
    off_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&off_cfg);
    gpio_set_level(GPIO_NUM_13, 0);
    gpio_set_level(GPIO_NUM_48, 0);
    ESP_LOGI(TAG, "LED pins 13/48 forced LOW");

    while (1) {
        spi_bus_free(SPI3_HOST);
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = mosi;
        buscfg.miso_io_num = -1;
        buscfg.sclk_io_num = sck;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 240 * 240 * 2;
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
        io_cfg.pclk_hz = 10 * 1000 * 1000;
        io_cfg.trans_queue_depth = 10;
        io_cfg.lcd_cmd_bits = 8;
        io_cfg.lcd_param_bits = 8;
        if (esp_lcd_new_panel_io_spi(SPI3_HOST, &io_cfg, &io) != ESP_OK) {
            ESP_LOGE(TAG, "io fail");
            continue;
        }
        esp_lcd_panel_dev_config_t pcfg = {};
        pcfg.reset_gpio_num = rst;
        pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;  // 0.71 dualeye uses BGR
        pcfg.bits_per_pixel = 16;
        // vendor init sequence MUST be set BEFORE esp_lcd_new_panel_gc9a01
        gc9a01_vendor_config_t vendor_cfg = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
        pcfg.vendor_config = &vendor_cfg;
        if (esp_lcd_new_panel_gc9a01(io, &pcfg, &panel) != ESP_OK) {
            ESP_LOGE(TAG, "gc9a01 panel fail");
            esp_lcd_panel_io_del(io);
            continue;
        }
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        ESP_LOGI(TAG, "GC9A01+DualEye seq: MOSI=%d SCK=%d CS=%d DC=%d RST=%d BGR", mosi, sck, cs, dc, rst);

        // panel glass is 240x240, active area 160x160 at offset (40,40)
        // draw RED at (40,40) then (0,0) to cover both offset hypotheses
        draw_solid(panel, 0xF800, 40, 40, 160, 160);
        vTaskDelay(pdMS_TO_TICKS(5000));
        draw_solid(panel, 0x07E0, 0, 0, 160, 160);
        vTaskDelay(pdMS_TO_TICKS(5000));
        draw_solid(panel, 0x001F, 40, 40, 160, 160);
        vTaskDelay(pdMS_TO_TICKS(5000));

        esp_lcd_panel_del(panel);
        esp_lcd_panel_io_del(io);
    }
}

void eye_test_permutation_start(void) {
    ESP_LOGI(TAG, "starting GC9A01+DualEye-seq test, 160x160 draws");
    xTaskCreate(test_task, "eye_test", 4096, NULL, 2, NULL);
}
