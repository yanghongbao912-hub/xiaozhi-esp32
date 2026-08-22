#pragma once

#include "display/lcd_display.h"

#include <lvgl.h>

/**
 * @brief 灵动眼睛显示 (0.71寸双目屏, 单 CS 同步显示)
 *
 * 继承 SpiLcdDisplay 复用面板 + LVGL 初始化, 但把 SetupUI 替换为
 * 程序化绘制的眼睛动画: 白色眼球 + 黑色瞳孔 + 上眼皮, 支持
 * 眨眼 / 眼球转动 / 随机打盹 / 瞳孔跟随声音(预留).
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

    // 由外部(音频)更新音量能量 0.0~1.0, 驱动瞳孔大小
    void SetAudioEnergy(float energy);

private:
    lv_obj_t* pupil_ = nullptr;      // 黑色瞳孔(圆)
    lv_obj_t* pupil_glint_ = nullptr; // 瞳孔高光(小白圆)
    lv_obj_t* eyelid_top_ = nullptr;  // 上眼皮(白矩形)
    lv_obj_t* eyelid_bottom_ = nullptr; // 下眼皮(白矩形)
    lv_timer_t* anim_timer_ = nullptr;

    // 眨眼状态机
    enum class BlinkPhase { kOpen, kClosing, kClosed, kOpening };
    BlinkPhase blink_phase_ = BlinkPhase::kOpen;
    uint32_t phase_elapsed_ = 0;
    int blink_closing_ms_ = 150;
    int blink_closed_ms_ = 90;
    int blink_opening_ms_ = 150;
    uint32_t next_blink_at_ = 0;

    // 瞳孔转动
    int pupil_cx_ = 0, pupil_cy_ = 0;   // 瞳孔当前中心
    int pupil_tx_ = 0, pupil_ty_ = 0;   // 瞳孔目标中心
    uint32_t next_move_at_ = 0;
    uint32_t move_back_at_ = 0;

    // 瞳孔大小(声音能量)
    int pupil_radius_ = 34;
    int pupil_radius_target_ = 34;
    float audio_energy_ = 0.0f;

    // 打盹
    int drowsy_ = 0;          // 0=清醒 1=半眯 2=快眨
    uint32_t drowsy_until_ = 0;

    static void TimerCb(lv_timer_t* timer);
    void Tick();
    void UpdateBlink();
    void UpdatePupil();
    void UpdateEyelids(int open_px);  // open_px: 眼皮打开程度(0=全闭, height=全开)
    void RandomizeBlink();
    void RandomizeMove();
    void RandomizeDrowsy();
};
