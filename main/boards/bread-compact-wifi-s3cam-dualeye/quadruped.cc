#include "quadruped.h"

#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <math.h>
#include <string.h>

#define TAG "Quadruped"

#define NVS_NS "quadruped"
#define KEY_NEUTRAL "neutral"   // 中立校准: 8 个 float

// 平滑步进(每 tick 最大变化度), 防暴冲
#define SMOOTH_STEP 3.0f
// 步态参数
#define HIP_AMP_MAX  45.0f    // 髋最大摆幅(度)
#define KNEE_AMP_MAX 40.0f    // 膝最大弯曲(度)
#define PHASE_STEP_WAVE  0.015f   // 波浪步态相位步进
#define PHASE_STEP_TROT  0.035f   // 对角步态相位步进

QuadrupedController::QuadrupedController(Pca9685* pca) : pca_(pca) {}

void QuadrupedController::SetLegCh(int leg, int hip_ch, int knee_ch) {
    if (leg >= 0 && leg < 4) {
        legs_[leg].hip = hip_ch;
        legs_[leg].knee = knee_ch;
    }
}

void QuadrupedController::SetTrim(int leg, int hip_trim, int knee_trim) {
    if (leg >= 0 && leg < 4) {
        hip_trim_[leg] = hip_trim;
        knee_trim_[leg] = knee_trim;
    }
}

void QuadrupedController::SetServoReverse(int leg, bool hip_rev, bool knee_rev) {
    if (leg >= 0 && leg < 4) {
        hip_rev_[leg] = hip_rev;
        knee_rev_[leg] = knee_rev;
    }
}

void QuadrupedController::SetSpeed(float s) {
    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;
    speed_ = s;
}

float QuadrupedController::SignedAngle(int leg, bool is_knee, float deg) {
    bool rev = is_knee ? knee_rev_[leg] : hip_rev_[leg];
    int trim = is_knee ? knee_trim_[leg] : hip_trim_[leg];
    return rev ? -(deg + trim) : (deg + trim);
}

void QuadrupedController::ApplyLeg(int leg, float hip_deg, float knee_deg) {
    pca_->SetServoAngle(legs_[leg].hip, 90.0f + SignedAngle(leg, false, hip_deg));
    pca_->SetServoAngle(legs_[leg].knee, 90.0f + SignedAngle(leg, true, knee_deg));
}

void QuadrupedController::LoadNeutralFromNvs() {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    float neutral[8] = {0};
    size_t len = sizeof(neutral);
    if (nvs_get_blob(h, KEY_NEUTRAL, neutral, &len) == ESP_OK && len == sizeof(neutral)) {
        // 应用到 hip_cur_/knee_cur_ 作为起点
        for (int leg = 0; leg < 4; leg++) {
            hip_cur_[leg] = neutral[leg * 2];
            knee_cur_[leg] = neutral[leg * 2 + 1];
            pose_hip_[leg] = hip_cur_[leg];
            pose_knee_[leg] = knee_cur_[leg];
        }
        ESP_LOGI(TAG, "Neutral loaded from NVS");
    }
    nvs_close(h);
}

void QuadrupedController::Init() {
    LoadNeutralFromNvs();
    ESP_LOGI(TAG, "Quadruped ready, gait=TROT, 8 servos");
}

void QuadrupedController::CalibrateNeutral() {
    // 把当前舵机角度记为中立 (相对 90 的偏移)
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed");
        return;
    }
    float neutral[8];
    for (int leg = 0; leg < 4; leg++) {
        neutral[leg * 2] = hip_cur_[leg];
        neutral[leg * 2 + 1] = knee_cur_[leg];
    }
    nvs_set_blob(h, KEY_NEUTRAL, neutral, sizeof(neutral));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Neutral calibrated & saved");
}

void QuadrupedController::SetPose(QuadPose p) {
    pose_ = p;
    pose_mode_ = true;
    // 9 种姿势: 8 个舵机相对中立的角度(度)
    // 腿: 0左前 1右前 2左后 3右后
    static const float poses[9][8] = {
        // 髋0  膝0  髋1  膝1  髋2  膝2  髋3  膝3
        {  0,   0,   0,   0,   0,   0,   0,   0},   // kNeutral 立正
        {  0, -60,   0, -60,   0, -60,   0, -60},   // kLieDown 趴下
        {  0, -85,   0, -85,   0, -45,   0, -45},   // kPuppy 小狗趴
        {  0,   0,   0,   0,  30, -90,  30, -90},   // kSit 坐下
        { 15,   0, -15,   0,  10,   0, -10,   0},   // kRelax 稍息
        {  0, -40,   0, -40,   0, -40,   0, -40},   // kBattle 战斗
        { 55, -40,   0,   0,   0,   0,   0,   0},   // kHandshakeL 握手左前
        {  0,   0, -55, -40,   0,   0,   0,   0},   // kHandshakeR 握手右前
        {  0,   0,   0,   0,  20,  10, -20,  10},   // kWiggle 扭屁股(静态偏置, 动画在Tick里加)
    };
    int idx = (int)p;
    if (idx < 0 || idx > 8) idx = 0;
    for (int leg = 0; leg < 4; leg++) {
        pose_hip_[leg] = poses[idx][leg * 2];
        pose_knee_[leg] = poses[idx][leg * 2 + 1];
    }
    ESP_LOGI(TAG, "Set pose %d", idx);
}

void QuadrupedController::Tick() {
    float step = SMOOTH_STEP * (speed_ < 0.3f ? 0.6f : 1.0f);

    if (pose_mode_) {
        // 平滑过渡到姿势
        bool done = true;
        for (int leg = 0; leg < 4; leg++) {
            float dh = pose_hip_[leg] - hip_cur_[leg];
            float dk = pose_knee_[leg] - knee_cur_[leg];
            if (fabsf(dh) > step) { hip_cur_[leg] += (dh > 0 ? step : -step); done = false; }
            else hip_cur_[leg] = pose_hip_[leg];
            if (fabsf(dk) > step) { knee_cur_[leg] += (dk > 0 ? step : -step); done = false; }
            else knee_cur_[leg] = pose_knee_[leg];
        }
        if (done) pose_mode_ = false;
        // 扭屁股动画: 后腿髋交替摆动
        if (pose_ == QuadPose::kWiggle) {
            float wig = sinf(phase_ * 4.0f * 3.14159f) * 25.0f;
            hip_cur_[2] = 20.0f + wig;
            hip_cur_[3] = -20.0f - wig;
        }
    } else {
        // 步态相位推进
        float pstep = (gait_ == QuadGait::kTrot) ? PHASE_STEP_TROT : PHASE_STEP_WAVE;
        phase_ += pstep * (0.4f + speed_ * 0.8f);
        if (phase_ > 1.0f) phase_ -= 1.0f;

        // 每条腿的相位
        float leg_ph[4];
        if (gait_ == QuadGait::kTrot) {
            leg_ph[0] = phase_;            // 左前
            leg_ph[1] = phase_ + 0.5f;     // 右前
            leg_ph[2] = phase_ + 0.5f;     // 左后
            leg_ph[3] = phase_;            // 右后
        } else {
            leg_ph[0] = phase_;
            leg_ph[1] = phase_ + 0.25f;
            leg_ph[2] = phase_ + 0.5f;
            leg_ph[3] = phase_ + 0.75f;
        }

        float amp = HIP_AMP_MAX * speed_;
        float kamp = KNEE_AMP_MAX * speed_;

        for (int leg = 0; leg < 4; leg++) {
            float ph = leg_ph[leg];
            float s = sinf(ph * 2.0f * 3.14159f);
            float swing = (sinf(ph * 2.0f * 3.14159f) > 0.0f) ? sinf(ph * 2.0f * 3.14159f) : 0.0f;

            float target_hip = 0, target_knee = 0;
            switch (dir_) {
            case QuadDir::kForward:
                target_hip = amp * s;
                target_knee = -kamp * swing;  // 抬腿时膝弯曲
                break;
            case QuadDir::kBackward:
                target_hip = -amp * s;
                target_knee = -kamp * swing;
                break;
            case QuadDir::kTurnLeft:
                // 左侧腿反向, 原地转
                target_hip = (leg == 0 || leg == 2) ? amp * s : -amp * s;
                target_knee = -kamp * swing;
                break;
            case QuadDir::kTurnRight:
                target_hip = (leg == 0 || leg == 2) ? -amp * s : amp * s;
                target_knee = -kamp * swing;
                break;
            case QuadDir::kShiftLeft:
                // 横向平移: 所有腿髋同相偏
                target_hip = amp * s;
                target_knee = -kamp * swing * 0.5f;
                break;
            case QuadDir::kShiftRight:
                target_hip = -amp * s;
                target_knee = -kamp * swing * 0.5f;
                break;
            default:  // kStop
                target_hip = 0;
                target_knee = 0;
                break;
            }

            // 平滑插值 (防暴冲)
            float dh = target_hip - hip_cur_[leg];
            float dk = target_knee - knee_cur_[leg];
            if (fabsf(dh) > step) hip_cur_[leg] += (dh > 0 ? step : -step);
            else hip_cur_[leg] = target_hip;
            if (fabsf(dk) > step) knee_cur_[leg] += (dk > 0 ? step : -step);
            else knee_cur_[leg] = target_knee;
        }
    }

    // 输出到舵机
    for (int leg = 0; leg < 4; leg++) {
        ApplyLeg(leg, hip_cur_[leg], knee_cur_[leg]);
    }
}
