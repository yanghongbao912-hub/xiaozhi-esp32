#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "eye_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "pca9685.h"
#include "quadruped.h"

#include <esp_log.h>
#include <driver/spi_common.h>
#include <driver/i2c_master.h>
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
    i2c_master_bus_handle_t servo_i2c_bus_ = nullptr;
    Pca9685* pca9685_ = nullptr;
    QuadrupedController* quadruped_ = nullptr;

    void InitializeServo() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = SERVO_SDA_PIN,
            .scl_io_num = SERVO_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        if (i2c_new_master_bus(&bus_config, &servo_i2c_bus_) != ESP_OK) {
            ESP_LOGE(TAG, "I2C bus init failed (SDA=%d SCL=%d)", (int)SERVO_SDA_PIN, (int)SERVO_SCL_PIN);
            return;
        }
        // 诊断: 扫描 I2C 总线, 看 PCA9685 是否在总线上
        ESP_LOGI(TAG, "I2C scan on SDA=%d SCL=%d:", (int)SERVO_SDA_PIN, (int)SERVO_SCL_PIN);
        bool found = false;
        for (int addr = 0x08; addr <= 0x77; addr++) {
            if (i2c_master_probe(servo_i2c_bus_, addr, 50) == ESP_OK) {
                ESP_LOGI(TAG, "  I2C device found @ 0x%02X", addr);
                found = true;
            }
        }
        if (!found) {
            ESP_LOGE(TAG, "  NO I2C device found! Check: SDA/SCL wiring, VCC 3V3, V+ 5V, GND, pullup");
        }
        pca9685_ = new Pca9685(servo_i2c_bus_, PCA9685_I2C_ADDR);
        if (pca9685_->Init() != ESP_OK) {
            ESP_LOGE(TAG, "PCA9685 init failed, check wiring (SDA=%d SCL=%d, V+ 5V?)", (int)SERVO_SDA_PIN, (int)SERVO_SCL_PIN);
        }
        quadruped_ = new QuadrupedController(pca9685_);
        quadruped_->Init();
        ESP_LOGI(TAG, "Servo I2C ready: SDA=%d SCL=%d, PCA9685 addr=0x%02X", (int)SERVO_SDA_PIN, (int)SERVO_SCL_PIN, PCA9685_I2C_ADDR);
    }

    static void QuadrupedTickTask(void* arg) {
        auto* board = static_cast<CompactWifiBoardS3CamDualEye*>(arg);
        while (1) {
            if (board->quadruped_ != nullptr) board->quadruped_->Tick();
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    static void QuadrupedDemoTask(void* arg) {
        auto* board = static_cast<CompactWifiBoardS3CamDualEye*>(arg);
        vTaskDelay(pdMS_TO_TICKS(1500));
        if (board->quadruped_ == nullptr) {
            ESP_LOGE(TAG, "Quadruped not ready, demo skipped");
            vTaskDelete(nullptr);
            return;
        }
        // 启动 Tick 任务 (20ms 周期)
        xTaskCreate(QuadrupedTickTask, "quad_tick", 4096, board, 1, nullptr);
        board->quadruped_->SetSpeed(0.6f);

        // 姿势演示
        ESP_LOGW(TAG, "=== Quadruped demo: 9 poses ===");
        board->quadruped_->SetPose(QuadPose::kNeutral);   vTaskDelay(pdMS_TO_TICKS(1800));
        board->quadruped_->SetPose(QuadPose::kLieDown);   vTaskDelay(pdMS_TO_TICKS(1800));
        board->quadruped_->SetPose(QuadPose::kPuppy);     vTaskDelay(pdMS_TO_TICKS(1800));
        board->quadruped_->SetPose(QuadPose::kSit);       vTaskDelay(pdMS_TO_TICKS(1800));
        board->quadruped_->SetPose(QuadPose::kRelax);     vTaskDelay(pdMS_TO_TICKS(1800));
        board->quadruped_->SetPose(QuadPose::kBattle);    vTaskDelay(pdMS_TO_TICKS(1800));
        board->quadruped_->SetPose(QuadPose::kHandshakeL); vTaskDelay(pdMS_TO_TICKS(1800));
        board->quadruped_->SetPose(QuadPose::kHandshakeR); vTaskDelay(pdMS_TO_TICKS(1800));
        board->quadruped_->SetPose(QuadPose::kWiggle);    vTaskDelay(pdMS_TO_TICKS(2500));
        board->quadruped_->SetPose(QuadPose::kNeutral);   vTaskDelay(pdMS_TO_TICKS(1500));

        // 步态演示 (TROT + WAVE)
        ESP_LOGW(TAG, "=== Quadruped demo: gait TROT forward ===");
        board->quadruped_->SetGait(QuadGait::kTrot);
        board->quadruped_->SetDirection(QuadDir::kForward);  vTaskDelay(pdMS_TO_TICKS(3000));
        board->quadruped_->SetDirection(QuadDir::kBackward); vTaskDelay(pdMS_TO_TICKS(3000));
        board->quadruped_->SetDirection(QuadDir::kTurnLeft); vTaskDelay(pdMS_TO_TICKS(2500));
        board->quadruped_->SetDirection(QuadDir::kTurnRight); vTaskDelay(pdMS_TO_TICKS(2500));
        board->quadruped_->SetDirection(QuadDir::kShiftLeft); vTaskDelay(pdMS_TO_TICKS(2500));
        board->quadruped_->SetDirection(QuadDir::kShiftRight); vTaskDelay(pdMS_TO_TICKS(2500));
        ESP_LOGW(TAG, "=== Quadruped demo: gait WAVE forward ===");
        board->quadruped_->SetGait(QuadGait::kWave);
        board->quadruped_->SetDirection(QuadDir::kForward);  vTaskDelay(pdMS_TO_TICKS(3000));

        // 结束: 回到立正
        board->quadruped_->SetDirection(QuadDir::kStop);
        board->quadruped_->SetPose(QuadPose::kNeutral);
        ESP_LOGW(TAG, "Quadruped demo done, idle");
        vTaskDelete(nullptr);
    }

    // MCP 工具: 四足运动控制 (AI/网页可直接调用)
    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        if (quadruped_ == nullptr) return;

        // 1) 运动控制: 步态 + 方向 + 速度
        mcp_server.AddTool("self.robot.gait",
            "四足机器人运动控制。gait: 步态, 0=波浪WAVE(每次抬一腿,最稳) 1=对角TROT(对角两腿,快); "
            "direction: 方向, 0=前进 1=后退 2=左转 3=右转 4=左平移 5=右平移 6=停止; "
            "speed: 速度 0-100, 平滑加速。方向切换自动先减速归零再反向, 不会抽筋。",
            PropertyList({
                Property("gait", kPropertyTypeInteger, 1, 0, 1),
                Property("direction", kPropertyTypeInteger, 6, 0, 6),
                Property("speed", kPropertyTypeInteger, 60, 0, 100)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int gait = properties["gait"].value<int>();
                int direction = properties["direction"].value<int>();
                int speed = properties["speed"].value<int>();
                quadruped_->SetPose(QuadPose::kNeutral);   // 退出姿势模式, 回到步态
                quadruped_->SetGait(gait == 0 ? QuadGait::kWave : QuadGait::kTrot);
                quadruped_->SetSpeed(speed / 100.0f);
                quadruped_->SetDirection((QuadDir)direction);
                return true;
            });

        // 2) 预设姿势 (9 种, 平滑过渡, 可随时打断)
        mcp_server.AddTool("self.robot.pose",
            "四足机器人预设姿势(一键触发, 平滑过渡, 可随时打断)。pose: 0=立正(停止并回中立, 说停就停) "
            "1=趴下 2=小狗趴 3=蹲下/坐下 4=稍息 5=战斗姿态 6=握手(抬左前腿) 7=握手(抬右前腿) 8=扭屁股(持续动画)",
            PropertyList({
                Property("pose", kPropertyTypeInteger, 0, 0, 8)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int pose = properties["pose"].value<int>();
                if (pose == 0) quadruped_->SetDirection(QuadDir::kStop);
                quadruped_->SetPose((QuadPose)pose);
                return true;
            });

        // 3) 参数配置 (网页/AI 调步幅/抬腿/步频/限幅/舵机反转等, 存 NVS 断电不丢)
        mcp_server.AddTool("self.robot.set_param",
            "四足机器人参数配置(存NVS断电不丢)。id: 参数编号; value: 数值。"
            "id表: 0=gait(0波浪/1对角) 1=direction(0前进..6停止) 2=speed(0-100) "
            "3=hip_amp步幅(度) 4=knee_amp抬腿高度(度) 5=phase_step步频TROT 6=phase_step_wave步频WAVE "
            "7=smooth_step每tick最大角度变化(防暴冲) 8=accel_limit加速度限幅 9=angle_limit角度限幅 "
            "10-13=髋舵机反转(0/1) 14-17=膝舵机反转(0/1) 18-21=髋中立微调(度) 22-25=膝中立微调(度)",
            PropertyList({
                Property("id", kPropertyTypeInteger, 3, 0, 25),
                Property("value", kPropertyTypeInteger, 45, -90, 180)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int id = properties["id"].value<int>();
                int value = properties["value"].value<int>();
                float v = (float)value;
                if (id == 2) v = value / 100.0f;   // speed: 0-100 -> 0-1
                if (quadruped_->SetParam(id, v)) {
                    ESP_LOGI(TAG, "quad param[%d]=%d", id, value);
                    return true;
                }
                return false;
            });
    }

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
        InitializeServo();
        InitializeTools();
        xTaskCreate(QuadrupedDemoTask, "quad_demo", 4096, this, 1, nullptr);
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
