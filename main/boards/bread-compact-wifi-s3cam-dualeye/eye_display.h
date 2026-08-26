#pragma once

#include "display/lcd_display.h"

#include <lvgl.h>

/**
 * @brief 魔法灵动眼睛 (0.71寸双目屏, 单 CS 同步显示)
 *
 * 彩虹渐变虹膜 + 辐射纹理 + 发光光晕 + 瞳孔高光 + 闪烁星点,
 * 支持眨眼 / 眼球转动 / 呼吸闪烁 / 打盹.
 */
class EyeDisplay : public SpiLcdDisplay {
public:
    EyeDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
               int width, int height, int offset_x, int offset_y,
               bool mirror_x, bool mirror_y, bool swap_xy);
    ~EyeDisplay() override;

    void SetupUI() override;
    void SetEmotion(const char* emotion) override;
    void SetChatMessage(const char* role, const char* content) override;

    void SetAudioEnergy(float energy);

private:
    // 虹膜 (预生成位图)
    lv_obj_t* iris_img_ = nullptr;
    uint8_t* iris_buf_ = nullptr;

    // 动态对象
    lv_obj_t* pupil_ = nullptr;
    lv_obj_t* pupil_glint_ = nullptr;
    lv_obj_t* star_points_[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* eyelid_top_ = nullptr;
    lv_obj_t* eyelid_bottom_ = nullptr;
    lv_timer_t* anim_timer_ = nullptr;

    // 尺寸参数
    int pupil_radius_ = 22;
    int iris_radius_ = 52;

    // 眨眼
    enum class BlinkPhase { kOpen, kClosing, kClosed, kOpening };
    BlinkPhase blink_phase_ = BlinkPhase::kOpen;
    uint32_t phase_elapsed_ = 0;
    int blink_closing_ms_ = 150;
    int blink_closed_ms_ = 90;
    int blink_opening_ms_ = 150;
    uint32_t next_blink_at_ = 0;

    // 瞳孔转动
    int pupil_cx_ = 0, pupil_cy_ = 0;
    int pupil_tx_ = 0, pupil_ty_ = 0;
    uint32_t next_move_at_ = 0;
    uint32_t move_back_at_ = 0;

    // 瞳孔大小(声音)
    int pupil_radius_target_ = 22;
    float audio_energy_ = 0.0f;

    // 呼吸/星点闪烁相位
    uint32_t anim_start_ = 0;

    // 打盹
    int drowsy_ = 0;
    uint32_t drowsy_until_ = 0;
    uint32_t next_drowsy_at_ = 0;

    void GenerateIris();
    static void TimerCb(lv_timer_t* timer);
    void Tick();
    void UpdateBlink();
    void UpdatePupil();
    void UpdateGlow();
    void RandomizeBlink();
    void RandomizeMove();
    void RandomizeDrowsy();
};
