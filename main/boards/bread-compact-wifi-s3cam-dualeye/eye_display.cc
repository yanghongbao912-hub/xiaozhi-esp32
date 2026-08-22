#include "eye_display.h"

#include <esp_log.h>
#include <esp_random.h>
#include <string>

#define TAG "EyeDisplay"

EyeDisplay::EyeDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                       int width, int height, int offset_x, int offset_y,
                       bool mirror_x, bool mirror_y, bool swap_xy)
    : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
    pupil_radius_ = width_ * 21 / 100;  // ~34px for 160
    if (pupil_radius_ < 20) pupil_radius_ = 20;
    pupil_radius_target_ = pupil_radius_;
}

EyeDisplay::~EyeDisplay() {
    if (anim_timer_ != nullptr) {
        lv_timer_del(anim_timer_);
        anim_timer_ = nullptr;
    }
}

void EyeDisplay::SetupUI() {
    if (setup_ui_called_) {
        return;
    }
    Display::SetupUI();
    DisplayLockGuard lock(this);

    auto screen = lv_screen_active();
    // 眼球 = 白色圆屏背景
    lv_obj_set_style_bg_color(screen, lv_color_hex(0xFFFFFF), 0);

    pupil_cx_ = width_ / 2;
    pupil_cy_ = height_ / 2;
    pupil_tx_ = pupil_cx_;
    pupil_ty_ = pupil_cy_;

    // 瞳孔 (黑色圆)
    pupil_ = lv_obj_create(screen);
    lv_obj_set_size(pupil_, pupil_radius_ * 2, pupil_radius_ * 2);
    lv_obj_set_style_radius(pupil_, 1000, 0);
    lv_obj_set_style_bg_color(pupil_, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(pupil_, 0, 0);
    lv_obj_set_pos(pupil_, pupil_cx_ - pupil_radius_, pupil_cy_ - pupil_radius_);

    // 瞳孔高光 (小白圆)
    pupil_glint_ = lv_obj_create(screen);
    lv_obj_set_size(pupil_glint_, 12, 12);
    lv_obj_set_style_radius(pupil_glint_, 1000, 0);
    lv_obj_set_style_bg_color(pupil_glint_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(pupil_glint_, 0, 0);

    // 上眼皮 (白矩形, 从上往下盖)
    eyelid_top_ = lv_obj_create(screen);
    lv_obj_set_size(eyelid_top_, width_, height_);
    lv_obj_set_style_radius(eyelid_top_, 0, 0);
    lv_obj_set_style_bg_color(eyelid_top_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(eyelid_top_, 0, 0);
    lv_obj_set_pos(eyelid_top_, 0, -height_);

    anim_timer_ = lv_timer_create(TimerCb, 40, this);
    RandomizeBlink();
    RandomizeMove();
    ESP_LOGI(TAG, "Eye UI ready: pupil=%dpx on %dx%d", pupil_radius_, width_, height_);
}

void EyeDisplay::SetEmotion(const char* emotion) {
    if (emotion == nullptr) return;
    // 情绪映射到瞳孔/眼皮的简单处理
    std::string e(emotion);
    if (e == "happy" || e == "smile" || e == "laughing") {
        // 高兴: 眯眼(瞳孔微缩) + 快眨
        pupil_radius_target_ = pupil_radius_ - 6;
        drowsy_ = 1;
        drowsy_until_ = lv_tick_get() + 2000;
    } else if (e == "sad" || e == "crying") {
        // 难过: 瞳孔下垂
        pupil_ty_ = pupil_cy_ + height_ / 6;
    } else if (e == "neutral" || e == "happy") {
        pupil_radius_target_ = pupil_radius_;
        drowsy_ = 0;
    }
}

void EyeDisplay::SetChatMessage(const char* role, const char* content) {
    // 眼睛模式不显示聊天文字
    (void)role;
    (void)content;
}

void EyeDisplay::SetAudioEnergy(float energy) {
    audio_energy_ = energy;
}

void EyeDisplay::TimerCb(lv_timer_t* timer) {
    auto* self = static_cast<EyeDisplay*>(lv_timer_get_user_data(timer));
    if (self != nullptr) {
        self->Tick();
    }
}

void EyeDisplay::Tick() {
    uint32_t now = lv_tick_get();

    // 声音能量 -> 瞳孔大小目标 (能量越高瞳孔越大)
    int energy_radius = pupil_radius_ + (int)(audio_energy_ * 8.0f);
    if (energy_radius > pupil_radius_ + 8) energy_radius = pupil_radius_ + 8;
    if (drowsy_ == 0) {
        pupil_radius_target_ = energy_radius;
    }

    UpdateBlink();
    UpdatePupil();

    // 打盹调度
    if (drowsy_ == 0 && now >= drowsy_until_ && (esp_random() % 100) < 3) {
        RandomizeDrowsy();
    }

    // 随机眨眼调度
    if (blink_phase_ == BlinkPhase::kOpen && now >= next_blink_at_) {
        blink_phase_ = BlinkPhase::kClosing;
        phase_elapsed_ = 0;
    }

    // 随机转动调度
    if (now >= next_move_at_) {
        RandomizeMove();
    }
}

void EyeDisplay::UpdateBlink() {
    uint32_t now = lv_tick_get();
    int cover = 0;  // 0=睁眼 height_=闭眼

    switch (blink_phase_) {
    case BlinkPhase::kOpen:
        cover = 0;
        break;
    case BlinkPhase::kClosing: {
        uint32_t el = now - phase_elapsed_;
        cover = height_ * el / blink_closing_ms_;
        if (cover >= height_) {
            cover = height_;
            blink_phase_ = BlinkPhase::kClosed;
            phase_elapsed_ = now;
        }
        break;
    }
    case BlinkPhase::kClosed: {
        uint32_t el = now - phase_elapsed_;
        cover = height_;
        if (el >= (uint32_t)blink_closed_ms_) {
            blink_phase_ = BlinkPhase::kOpening;
            phase_elapsed_ = now;
        }
        break;
    }
    case BlinkPhase::kOpening: {
        uint32_t el = now - phase_elapsed_;
        cover = height_ - height_ * el / blink_opening_ms_;
        if (cover <= 0) {
            cover = 0;
            blink_phase_ = BlinkPhase::kOpen;
            RandomizeBlink();
        }
        break;
    }
    }

    lv_obj_set_pos(eyelid_top_, 0, cover - height_);
}

void EyeDisplay::UpdatePupil() {
    // 瞳孔位置平滑逼近目标
    if (pupil_cx_ < pupil_tx_) pupil_cx_ += 3;
    else if (pupil_cx_ > pupil_tx_) pupil_cx_ -= 3;
    if (pupil_cy_ < pupil_ty_) pupil_cy_ += 3;
    else if (pupil_cy_ > pupil_ty_) pupil_cy_ -= 3;

    // 瞳孔大小平滑逼近目标
    if (pupil_radius_ < pupil_radius_target_) pupil_radius_++;
    else if (pupil_radius_ > pupil_radius_target_) pupil_radius_--;

    lv_obj_set_size(pupil_, pupil_radius_ * 2, pupil_radius_ * 2);
    lv_obj_set_pos(pupil_, pupil_cx_ - pupil_radius_, pupil_cy_ - pupil_radius_);

    // 高光跟随瞳孔 (左上角)
    lv_obj_set_pos(pupil_glint_, pupil_cx_ - pupil_radius_ * 2 / 3, pupil_cy_ - pupil_radius_ * 2 / 3);

    // 瞳孔到达目标后, 过一会儿回到中心
    uint32_t now = lv_tick_get();
    if (pupil_cx_ == pupil_tx_ && pupil_cy_ == pupil_ty_ && pupil_tx_ != width_ / 2 && now >= move_back_at_) {
        pupil_tx_ = width_ / 2;
        pupil_ty_ = height_ / 2;
    }
}

void EyeDisplay::RandomizeBlink() {
    uint32_t now = lv_tick_get();
    // 正常眨眼间隔 2~5 秒; 打盹时快眨
    if (drowsy_ == 2) {
        next_blink_at_ = now + 200 + (esp_random() % 400);
        blink_closing_ms_ = 60;
        blink_closed_ms_ = 40;
        blink_opening_ms_ = 60;
    } else if (drowsy_ == 1) {
        next_blink_at_ = now + 1500 + (esp_random() % 1500);
        blink_closing_ms_ = 200;
        blink_closed_ms_ = 150;
        blink_opening_ms_ = 200;
    } else {
        next_blink_at_ = now + 2000 + (esp_random() % 3000);
        blink_closing_ms_ = 130;
        blink_closed_ms_ = 80;
        blink_opening_ms_ = 130;
    }
}

void EyeDisplay::RandomizeMove() {
    uint32_t now = lv_tick_get();
    int range = width_ / 6;  // ~26px
    pupil_tx_ = width_ / 2 + (int)(esp_random() % (range * 2)) - range;
    pupil_ty_ = height_ / 2 + (int)(esp_random() % (range * 2)) - range;
    // 瞳孔不能超出眼球
    if (pupil_tx_ < width_ / 4) pupil_tx_ = width_ / 4;
    if (pupil_tx_ > width_ * 3 / 4) pupil_tx_ = width_ * 3 / 4;
    if (pupil_ty_ < height_ / 4) pupil_ty_ = height_ / 4;
    if (pupil_ty_ > height_ * 3 / 4) pupil_ty_ = height_ * 3 / 4;

    move_back_at_ = now + 600 + (esp_random() % 800);
    next_move_at_ = now + 2000 + (esp_random() % 2000);
}

void EyeDisplay::RandomizeDrowsy() {
    // 随机进入打盹: 1=半眯 2=快眨
    drowsy_ = (esp_random() % 2) ? 1 : 2;
    drowsy_until_ = lv_tick_get() + 1500 + (esp_random() % 2500);
    if (drowsy_ == 1) {
        // 半眯: 眼皮半合 (模拟眯眼) - 通过缩短眨眼间隔 + 缩小瞳孔
        pupil_radius_target_ = pupil_radius_ - 4;
    }
    ESP_LOGI(TAG, "drowsy=%d", drowsy_);
    RandomizeBlink();
}
