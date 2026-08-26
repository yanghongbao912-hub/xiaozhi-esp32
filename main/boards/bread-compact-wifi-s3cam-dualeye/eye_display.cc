#include "eye_display.h"

#include <esp_log.h>
#include <esp_random.h>
#include <math.h>
#include <string>

#define TAG "EyeDisplay"

static uint16_t hsv_to_rgb565(float h, float s, float v) {
    // h in [0,1), s,v in [0,1]
    if (s <= 0.0f) {
        uint8_t g = (uint8_t)(v * 63.0f + 0.5f);
        uint8_t r = (uint8_t)(v * 31.0f + 0.5f);
        return (uint16_t)(r << 11) | (uint16_t)(g << 5) | r;
    }
    h *= 6.0f;
    int i = (int)h;
    float f = h - (float)i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));
    float r, g, b;
    switch (i % 6) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
    }
    return (uint16_t)((uint16_t)(r * 31.0f + 0.5f) << 11) |
           ((uint16_t)(g * 63.0f + 0.5f) << 5) |
           (uint16_t)(b * 31.0f + 0.5f);
}

static uint16_t blend565(uint16_t a, uint16_t b, float t) {
    // t in [0,1]: 0 => a, 1 => b
    int ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
    int br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
    int r = (int)(ar + (br - ar) * t);
    int g = (int)(ag + (bg - ag) * t);
    int bl = (int)(ab + (bb - ab) * t);
    return (uint16_t)((r << 11) | (g << 5) | bl);
}

EyeDisplay::EyeDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                       int width, int height, int offset_x, int offset_y,
                       bool mirror_x, bool mirror_y, bool swap_xy)
    : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
    pupil_radius_ = width_ * 14 / 100;  // ~22 for 160
    iris_radius_ = width_ * 33 / 100;   // ~52 for 160
    if (pupil_radius_ < 12) pupil_radius_ = 12;
    if (iris_radius_ < pupil_radius_ + 20) iris_radius_ = pupil_radius_ + 20;
    pupil_radius_target_ = pupil_radius_;
}

EyeDisplay::~EyeDisplay() {
    if (anim_timer_ != nullptr) {
        lv_timer_del(anim_timer_);
        anim_timer_ = nullptr;
    }
    if (iris_buf_ != nullptr) {
        heap_caps_free(iris_buf_);
        iris_buf_ = nullptr;
    }
}

void EyeDisplay::GenerateIris() {
    int w = width_, h = height_;
    int cx = w / 2, cy = h / 2;
    const int glow = 14;
    iris_buf_ = (uint8_t*)heap_caps_malloc(w * h * 2, MALLOC_CAP_SPIRAM);
    if (iris_buf_ == nullptr) {
        ESP_LOGE(TAG, "no mem for iris buffer");
        return;
    }
    uint16_t* buf = (uint16_t*)iris_buf_;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int dx = x - cx, dy = y - cy;
            float dist = sqrtf((float)(dx * dx + dy * dy));
            float angle = atan2f((float)dy, (float)dx);
            if (angle < 0) angle += 2.0f * 3.14159265f;
            float hue = angle / (2.0f * 3.14159265f);

            uint16_t color;
            if (dist <= (float)pupil_radius_) {
                color = 0x0000;  // 瞳孔黑
            } else if (dist <= (float)iris_radius_) {
                // 彩虹渐变虹膜 + 辐射纹理
                float t = dist / (float)iris_radius_;
                float sat = 0.55f + 0.30f * t;
                float tex = 0.72f + 0.28f * sinf(angle * 13.0f + dist * 0.32f);
                float val = tex * (1.05f - 0.15f * t);  // 中心亮边缘暗
                if (val > 1.0f) val = 1.0f;
                color = hsv_to_rgb565(hue, sat, val);
            } else if (dist <= (float)(iris_radius_ + glow)) {
                // 发光光晕: 虹膜边缘渐隐到白
                float t = (dist - (float)iris_radius_) / (float)glow;
                uint16_t iris = hsv_to_rgb565(hue, 0.55f, 0.85f);
                color = blend565(iris, 0xFFFF, t * t);
            } else {
                color = 0xFFFF;  // 眼球白
            }
            buf[y * w + x] = color;
        }
    }
}

void EyeDisplay::SetupUI() {
    if (setup_ui_called_) {
        return;
    }
    // 必须调用父类 SetupUI, 缺失会导致 AfE 初始化崩溃
    SpiLcdDisplay::SetupUI();

    DisplayLockGuard lock(this);
    auto screen = lv_screen_active();
    // 隐藏父类文字/图标 UI
    if (emoji_label_ != nullptr) lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
    if (emoji_image_ != nullptr) lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
    if (status_bar_ != nullptr) lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
    if (top_bar_ != nullptr) lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_bg_color(screen, lv_color_hex(0xFFFFFF), 0);

    // 虹膜
    GenerateIris();
    iris_img_ = lv_canvas_create(screen);
    lv_canvas_set_buffer(iris_img_, iris_buf_, width_, height_, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(iris_img_, 0, 0);

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

    // 瞳孔高光
    pupil_glint_ = lv_obj_create(screen);
    lv_obj_set_size(pupil_glint_, 10, 10);
    lv_obj_set_style_radius(pupil_glint_, 1000, 0);
    lv_obj_set_style_bg_color(pupil_glint_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(pupil_glint_, 0, 0);

    // 闪烁星点 (虹膜周围的 4 个白点)
    const float star_ang[4] = {0.6f, 2.2f, 3.8f, 5.4f};
    for (int i = 0; i < 4; i++) {
        star_points_[i] = lv_obj_create(screen);
        lv_obj_set_size(star_points_[i], 6, 6);
        lv_obj_set_style_radius(star_points_[i], 1000, 0);
        lv_obj_set_style_bg_color(star_points_[i], lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_border_width(star_points_[i], 0, 0);
        int sx = width_ / 2 + (int)(cosf(star_ang[i]) * (iris_radius_ - 8));
        int sy = height_ / 2 + (int)(sinf(star_ang[i]) * (iris_radius_ - 8));
        lv_obj_set_pos(star_points_[i], sx - 3, sy - 3);
    }

    // 上眼皮 (白色矩形, 从上方往下盖)
    eyelid_top_ = lv_obj_create(screen);
    lv_obj_set_size(eyelid_top_, width_, height_ / 2);
    lv_obj_set_style_radius(eyelid_top_, 0, 0);
    lv_obj_set_style_bg_color(eyelid_top_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(eyelid_top_, 0, 0);
    lv_obj_set_pos(eyelid_top_, 0, -height_ / 2);

    // 下眼皮 (白色矩形, 从下方往上抬, 与上眼皮在瞳孔中心会合)
    eyelid_bottom_ = lv_obj_create(screen);
    lv_obj_set_size(eyelid_bottom_, width_, height_ / 2);
    lv_obj_set_style_radius(eyelid_bottom_, 0, 0);
    lv_obj_set_style_bg_color(eyelid_bottom_, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(eyelid_bottom_, 0, 0);
    lv_obj_set_pos(eyelid_bottom_, 0, height_);

    anim_timer_ = lv_timer_create(TimerCb, 40, this);
    anim_start_ = lv_tick_get();
    RandomizeBlink();
    RandomizeMove();
    next_drowsy_at_ = lv_tick_get() + 8000 + (esp_random() % 7000);
    ESP_LOGI(TAG, "Magic eye ready: iris=%dpx pupil=%dpx on %dx%d", iris_radius_, pupil_radius_, width_, height_);
}

void EyeDisplay::SetEmotion(const char* emotion) {
    if (emotion == nullptr) return;
    std::string e(emotion);
    if (e == "happy" || e == "smile" || e == "laughing") {
        pupil_radius_target_ = pupil_radius_ - 4;
        drowsy_ = 1;
        drowsy_until_ = lv_tick_get() + 2000;
    } else if (e == "sad" || e == "crying") {
        pupil_ty_ = pupil_cy_ + height_ / 6;
    } else if (e == "neutral") {
        pupil_radius_target_ = pupil_radius_;
        drowsy_ = 0;
    }
}

void EyeDisplay::SetChatMessage(const char* role, const char* content) {
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

    int energy_radius = pupil_radius_ + (int)(audio_energy_ * 6.0f);
    if (energy_radius > pupil_radius_ + 6) energy_radius = pupil_radius_ + 6;

    if (drowsy_ != 0) {
        if (now >= drowsy_until_) {
            drowsy_ = 0;
            pupil_radius_target_ = energy_radius;
            next_drowsy_at_ = now + 6000 + (esp_random() % 9000);
            RandomizeBlink();
        }
    } else {
        pupil_radius_target_ = energy_radius;
        if (now >= next_drowsy_at_) {
            RandomizeDrowsy();
        }
    }

    UpdateBlink();
    UpdatePupil();
    UpdateGlow();

    if (blink_phase_ == BlinkPhase::kOpen && now >= next_blink_at_) {
        blink_phase_ = BlinkPhase::kClosing;
        phase_elapsed_ = 0;
    }
    if (now >= next_move_at_) {
        RandomizeMove();
    }
}

void EyeDisplay::UpdateBlink() {
    uint32_t now = lv_tick_get();
    int half = height_ / 2;  // 上下眼皮各半屏, 在瞳孔中心会合
    int cover = 0;           // 0=睁眼, half=全闭
    switch (blink_phase_) {
    case BlinkPhase::kOpen:
        cover = 0;
        break;
    case BlinkPhase::kClosing: {
        uint32_t el = now - phase_elapsed_;
        cover = half * el / blink_closing_ms_;
        if (cover >= half) {
            cover = half;
            blink_phase_ = BlinkPhase::kClosed;
            phase_elapsed_ = now;
        }
        break;
    }
    case BlinkPhase::kClosed: {
        uint32_t el = now - phase_elapsed_;
        cover = half;
        if (el >= (uint32_t)blink_closed_ms_) {
            blink_phase_ = BlinkPhase::kOpening;
            phase_elapsed_ = now;
        }
        break;
    }
    case BlinkPhase::kOpening: {
        uint32_t el = now - phase_elapsed_;
        cover = half - half * el / blink_opening_ms_;
        if (cover <= 0) {
            cover = 0;
            blink_phase_ = BlinkPhase::kOpen;
            RandomizeBlink();
        }
        break;
    }
    }
    // 上眼皮: y = cover - half  (睁眼在屏外, 闭眼盖到中间)
    // 下眼皮: y = height_ - cover (睁眼在屏外底部, 闭眼抬到中间)
    lv_obj_set_pos(eyelid_top_, 0, cover - half);
    lv_obj_set_pos(eyelid_bottom_, 0, height_ - cover);
}

void EyeDisplay::UpdatePupil() {
    const int step = 2;
    if (pupil_tx_ > pupil_cx_) pupil_cx_ = (pupil_tx_ - pupil_cx_ <= step) ? pupil_tx_ : pupil_cx_ + step;
    else if (pupil_tx_ < pupil_cx_) pupil_cx_ = (pupil_cx_ - pupil_tx_ <= step) ? pupil_tx_ : pupil_cx_ - step;
    if (pupil_ty_ > pupil_cy_) pupil_cy_ = (pupil_ty_ - pupil_cy_ <= step) ? pupil_ty_ : pupil_cy_ + step;
    else if (pupil_ty_ < pupil_cy_) pupil_cy_ = (pupil_cy_ - pupil_ty_ <= step) ? pupil_ty_ : pupil_cy_ - step;

    if (pupil_radius_ < pupil_radius_target_) pupil_radius_++;
    else if (pupil_radius_ > pupil_radius_target_) pupil_radius_--;

    lv_obj_set_size(pupil_, pupil_radius_ * 2, pupil_radius_ * 2);
    lv_obj_set_pos(pupil_, pupil_cx_ - pupil_radius_, pupil_cy_ - pupil_radius_);
    lv_obj_set_pos(pupil_glint_, pupil_cx_ - pupil_radius_ * 2 / 3, pupil_cy_ - pupil_radius_ * 2 / 3);

    uint32_t now = lv_tick_get();
    if (pupil_cx_ == pupil_tx_ && pupil_cy_ == pupil_ty_ &&
        (pupil_tx_ != width_ / 2 || pupil_ty_ != height_ / 2) && now >= move_back_at_) {
        pupil_tx_ = width_ / 2;
        pupil_ty_ = height_ / 2;
    }
}

void EyeDisplay::UpdateGlow() {
    // 呼吸闪烁: 虹膜整体透明度在 88%~100% 之间缓慢呼吸
    uint32_t t = lv_tick_get() - anim_start_;
    float breath = 0.5f + 0.5f * sinf((float)t / 1400.0f * 2.0f * 3.14159265f);
    int iris_opa = 200 + (int)(breath * 55);  // 200~255
    lv_obj_set_style_opa(iris_img_, (lv_opa_t)iris_opa, 0);

    // 星点闪烁: 各自不同相位
    for (int i = 0; i < 4; i++) {
        float tw = 0.5f + 0.5f * sinf((float)t / (700.0f + i * 250.0f) * 2.0f * 3.14159265f + i);
        int opa = 40 + (int)(tw * 215);
        lv_obj_set_style_opa(star_points_[i], (lv_opa_t)opa, 0);
    }
}

void EyeDisplay::RandomizeBlink() {
    uint32_t now = lv_tick_get();
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
    int range = width_ / 7;
    pupil_tx_ = width_ / 2 + (int)(esp_random() % (range * 2)) - range;
    pupil_ty_ = height_ / 2 + (int)(esp_random() % (range * 2)) - range;
    if (pupil_tx_ < width_ / 5) pupil_tx_ = width_ / 5;
    if (pupil_tx_ > width_ * 4 / 5) pupil_tx_ = width_ * 4 / 5;
    if (pupil_ty_ < height_ / 5) pupil_ty_ = height_ / 5;
    if (pupil_ty_ > height_ * 4 / 5) pupil_ty_ = height_ * 4 / 5;
    move_back_at_ = now + 600 + (esp_random() % 800);
    next_move_at_ = now + 2000 + (esp_random() % 2000);
}

void EyeDisplay::RandomizeDrowsy() {
    drowsy_ = (esp_random() % 2) ? 1 : 2;
    drowsy_until_ = lv_tick_get() + 1500 + (esp_random() % 2500);
    if (drowsy_ == 1) {
        pupil_radius_target_ = pupil_radius_ - 4;
    }
    RandomizeBlink();
}
