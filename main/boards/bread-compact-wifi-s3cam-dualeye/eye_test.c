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

static bool next_perm(int *a, int n) {
    int i = n - 2;
    while (i >= 0 && a[i] >= a[i + 1]) i--;
    if (i < 0) return false;
    int j = n - 1;
    while (a[j] <= a[i]) j--;
    int t = a[i]; a[i] = a[j]; a[j] = t;
    for (int l = i + 1, r = n - 1; l < r; l++, r--) { t = a[l]; a[l] = a[r]; a[r] = t; }
    return true;
}

static void test_task(void *arg) {
    // 5 physical GPIOs (user wiring, FIXED):
    //   GPIO38->FPC11, GPIO21->FPC10, GPIO47->FPC9, GPIO3->FPC8, GPIO45->FPC7
    // permute all 5 signals {MOSI,SCK,CS,DC,RST} over these 5 GPIOs (120 combos)
    // Panel driver = GC9D01 (strict, with 3-phase reset 50/50/120ms)
    int gpio[5] = {3, 21, 38, 45, 47};

    gpio_config_t off_cfg = {};
    off_cfg.pin_bit_mask = (1ULL << 13) | (1ULL << 48);
    off_cfg.mode = GPIO_MODE_OUTPUT;
    off_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&off_cfg);
    gpio_set_level(GPIO_NUM_13, 0);
    gpio_set_level(GPIO_NUM_48, 0);

    int n = 0;
    do {
        n++;
        int mosi = gpio[0], sck = gpio[1], cs = gpio[2], dc = gpio[3], rst = gpio[4];
        ESP_LOGW(TAG, "=== PERM %d/120: MOSI=%d SCK=%d CS=%d DC=%d RST=%d (WHITE) ===", n, mosi, sck, cs, dc, rst);

        spi_bus_free(SPI3_HOST);
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = mosi;
        buscfg.miso_io_num = -1;
        buscfg.sclk_io_num = sck;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = 240 * 240 * 2;
        if (spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
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
            continue;
        }
        esp_lcd_panel_dev_config_t pcfg = {};
        pcfg.reset_gpio_num = rst;
        pcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        pcfg.bits_per_pixel = 16;
        // GC9D01 driver: built-in GC9D01 init seq + 3-phase reset (50/50/120ms)
        if (esp_lcd_new_panel_gc9d01(io, &pcfg, &panel) != ESP_OK) {
            esp_lcd_panel_io_del(io);
            continue;
        }
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);

        draw_solid(panel, 0xFFFF, 40, 40, 160, 160);
        draw_solid(panel, 0xFFFF, 0, 0, 160, 160);
        vTaskDelay(pdMS_TO_TICKS(2000));

        esp_lcd_panel_del(panel);
        esp_lcd_panel_io_del(io);
    } while (next_perm(gpio, 5));

    ESP_LOGI(TAG, "all 120 permutations done, restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

void eye_test_permutation_start(void) {
    ESP_LOGI(TAG, "starting FULL permutation scan with GC9D01 driver (120 combos)");
    xTaskCreate(test_task, "eye_test", 4096, NULL, 2, NULL);
}
