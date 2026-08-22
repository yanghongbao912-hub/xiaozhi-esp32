#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "eye_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"

#include <esp_log.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include "esp_lcd_gc9d01.h"

#ifdef DUALEYE_TEST_EYE
#include "eye_test.h"
#endif

#define TAG "CompactWifiBoardS3CamDualEye"

class CompactWifiBoardS3CamDualEye : public WifiBoard {
private:
    Button boot_button_;
    Button touch_button_;
    Display* display_;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
#ifdef DUALEYE_TEST_EYE
        // Test mode: constant backlight via plain GPIO high (no PWM,
        // avoids LEDC interacting with the onboard WS2812 on GPIO48).
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            gpio_config_t bl_cfg = {};
            bl_cfg.pin_bit_mask = 1ULL << DISPLAY_BACKLIGHT_PIN;
            bl_cfg.mode = GPIO_MODE_OUTPUT;
            gpio_config(&bl_cfg);
            gpio_set_level(DISPLAY_BACKLIGHT_PIN, 1);
            ESP_LOGI(TAG, "Backlight: constant ON (GPIO %d)", DISPLAY_BACKLIGHT_PIN);
        }
        // RST/DC/CS permutation test: the test task creates its own panel
        // for each combination and draws a distinct color.
        display_ = new NoDisplay();
        eye_test_permutation_start();
        return;
#endif
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 2 * 1000 * 1000;  // GC9D01 needs low SPI clock (2MHz verified)
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // Strict GC9D01 driver (authoritative DualGate 160x160 sequence,
        // COLMOD 0x55, inversion 0xEC 0x70, ends 0x3C CONTINUE).
        ESP_LOGI(TAG, "Install GC9D01 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9d01(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        // inversion handled by vendor seq 0xEC 0x70; do NOT call invert_color here
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new EyeDisplay(panel_io, panel,
                                  DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        // TTP223 触摸: 点一下, 小智说固定台词
        touch_button_.OnClick([this]() {
            ESP_LOGI(TAG, "Touch button click -> speak");
            std::string wake_word = "死鬼摸人家干嘛";
            Application::GetInstance().WakeWordInvoke(wake_word);
        });
    }

public:
    CompactWifiBoardS3CamDualEye() :
        boot_button_(BOOT_BUTTON_GPIO), touch_button_(TOUCH_BUTTON_GPIO, true) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
#ifdef DUALEYE_TEST_EYE
        // test mode: backlight already driven constantly in InitializeLcdDisplay
#else
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->SetBrightness(100);  // 保持最高亮度
        }
#endif
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(CompactWifiBoardS3CamDualEye);
