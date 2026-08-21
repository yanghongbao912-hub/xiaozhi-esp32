#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "esp_lcd_gc9d01.h"
#include "eye_test.h"

static const char *TAG = "eye_test";

static void draw_solid(esp_lcd_panel_handle_t panel, uint16_t color, int x0, int y0, int w, int h) {
    uint8_t *buf = (uint8_t *)heap_caps_malloc(w * h * 2, MALLOC_CAP_DMA);
    if (!buf) return;
    for (int p = 0; p < w * h; p++) {
        buf[p * 2] = (uint8_t)(color >> 8);
        buf[p * 2 + 1] = (uint8_t)(color & 0xFF);
    }
    esp_lcd_panel_draw_bitmap(panel, x0, y0, x0 + w, y0 + h, buf);
    heap_caps_free(buf);
}

static void test_task(void *arg) {
    // MOSI=38, SCK=21 LOCKED. Permute only {CS,DC,RST} over {47,3,45}, distinct colors.
    const int mosi = 38, sck = 21;
    const int perms[6][3] = {
        {47, 3, 45},   // CS=47 DC=3  RST=45  -> RED   (config.h guess)
        {47, 45, 3},   // CS=47 DC=45 RST=3   -> GREEN
        {3, 47, 45},   // CS=3  DC=47 RST=45  -> BLUE
        {3, 45, 47},   // CS=3  DC=45 RST=47  -> YELLOW
        {45, 3, 47},   // CS=45 DC=3  RST=47  -> MAGENTA
        {45, 47, 3},   // CS=45 DC=47 RST=3   -> CYAN
    };
    const uint16_t colors[6] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF};
    const char *names[6] = {"RED", "GREEN", "BLUE", "YELLOW", "MAGENTA", "CYAN"};

    gpio_config_t off_cfg = {};
    off_cfg.pin_bit_mask = (1ULL << 13) | (1ULL << 48);
    off_cfg.mode = GPIO_MODE_OUTPUT;
    off_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&off_cfg);
    gpio_set_level(GPIO_NUM_13, 0);
    gpio_set_level(GPIO_NUM_48, 0);

    while (1) {
        for (int i = 0; i < 6; i++) {
            int cs = perms[i][0], dc = perms[i][1], rst = perms[i][2];
            ESP_LOGW(TAG, "=== CS=%d DC=%d RST=%d -> %s ===", cs, dc, rst, names[i]);

            spi_bus_free(SPI3_HOST);
            spi_bus_config_t buscfg = {};
            buscfg.mosi_io_num = mosi;
            buscfg.miso_io_num = -1;
            buscfg.sclk_io_num = sck;
            buscfg.quadwp_io_num = -1;
            buscfg.quadhd_io_num = -1;
            buscfg.max_transfer_sz = 160 * 160 * 2;
            if (spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) continue;

            esp_lcd_panel_io_handle_t io = NULL;
            esp_lcd_panel_handle_t panel = NULL;
            esp_lcd_panel_io_spi_config_t io_cfg = {};
            io_cfg.cs_gpio_num = cs;
            io_cfg.dc_gpio_num = dc;
            io_cfg.spi_mode = 0;
            io_cfg.pclk_hz = 2 * 1000 * 1000;
            io_cfg.trans_queue_depth = 10;
            io_cfg.lcd_cmd_bits = 8;
            io_cfg.lcd_param_bits = 8;
            if (esp_lcd_new_panel_io_spi(SPI3_HOST, &io_cfg, &io) != ESP_OK) continue;

            esp_lcd_panel_dev_config_t pcfg = {};
            pcfg.reset_gpio_num = rst;
            pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
            pcfg.bits_per_pixel = 16;
            if (esp_lcd_new_panel_gc9d01(io, &pcfg, &panel) != ESP_OK) {
                esp_lcd_panel_io_del(io);
                continue;
            }
            esp_lcd_panel_reset(panel);
            esp_lcd_panel_init(panel);

            draw_solid(panel, colors[i], 0, 0, 160, 160);
            vTaskDelay(pdMS_TO_TICKS(4000));

            esp_lcd_panel_del(panel);
            esp_lcd_panel_io_del(io);
        }
    }
}

void eye_test_permutation_start(void) {
    ESP_LOGI(TAG, "CS/DC/RST color test (MOSI=38 SCK=21 locked)");
    xTaskCreate(test_task, "eye_test", 4096, NULL, 2, NULL);
}
