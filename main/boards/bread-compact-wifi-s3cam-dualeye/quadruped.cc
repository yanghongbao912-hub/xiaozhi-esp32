#include "quadruped.h"

#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <math.h>
#include <string.h>

#define TAG "Quadruped"

#define NVS_NS "quadruped"
#define KEY_PARAMS  "params"    // 参数表: QuadParams blob

static_assert(sizeof(QuadParams) == 32 * 4, "QuadParams must be 32 contiguous floats");

static float ClampF(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// 参数边界校验: 防止 l1/l2=0 除零、duty=1 除零、超程等
static float ClampParamId(int id, float v) {
    switch (id) {
    case 2:  v = ClampF(v, 0.0f, 1.0f); break;                    // speed
    case 3:  case 4: v = ClampF(v, 0.0f, 90.0f); break;           // hip_amp/knee_amp
    case 5:  case 6: v = ClampF(v, 0.001f, 0.2f); break;          // phase_step
    case 7:  v = ClampF(v, 0.5f, 20.0f); break;                   // smooth_step
    case 8:  v = ClampF(v, 0.1f, 10.0f); break;                   // accel_limit
    case 9:  v = ClampF(v, 10.0f, 90.0f); break;                  // angle_limit
    case 10: case 11: v = ClampF(v, 20.0f, 200.0f); break;        // l1/l2 (防0除零)
    case 12: v = ClampF(v, 20.0f, 300.0f); break;                 // body_height
    case 13: v = ClampF(v, 0.0f, 100.0f); break;                  // step_len
    case 14: v = ClampF(v, 0.0f, 50.0f); break;                   // lift_height
    case 15: v = ClampF(v, 0.5f, 0.9f); break;                    // duty (防1除零)
    case 24: case 25: case 26: case 27:
    case 28: case 29: case 30: case 31: v = ClampF(v, -30.0f, 30.0f); break; // trim
    default: break;
    }
    return v;
}

QuadrupedController::QuadrupedController(Pca9685* pca) : pca_(pca) {}

// ---------------- 参数表 ----------------

const char* QuadrupedController::ParamName(int id) {
    static const char* names[32] = {
        "gait", "direction", "speed", "hip_amp", "knee_amp",
        "phase_step", "phase_step_wave", "smooth_step", "accel_limit", "angle_limit",
        "l1", "l2", "body_height", "step_len", "lift_height", "duty",
        "hip_rev_0", "hip_rev_1", "hip_rev_2", "hip_rev_3",
        "knee_rev_0", "knee_rev_1", "knee_rev_2", "knee_rev_3",
        "hip_trim_0", "hip_trim_1", "hip_trim_2", "hip_trim_3",
        "knee_trim_0", "knee_trim_1", "knee_trim_2", "knee_trim_3",
    };
    return (id >= 0 && id < 32) ? names[id] : "";
}

float QuadrupedController::GetParam(int id) const {
    if (id < 0 || id >= ParamCount()) return 0.0f;
    return ((const float*)&params_.gait)[id];
}

bool QuadrupedController::SetParam(int id, float v) {
    if (id < 0 || id >= ParamCount()) return false;
    v = ClampParamId(id, v);
    Lock();
    float* p = (float*)&params_.gait;
    p[id] = v;
    switch (id) {
    case 0:  // gait: 0=WALK 1=WAVE 2=TROT
        if (v < 0.5f) gait_ = QuadGait::kWalk;
        else if (v < 1.5f) gait_ = QuadGait::kWave;
        else gait_ = QuadGait::kTrot;
        break;
    case 1:  // direction: 走零突变切换
        if (v < 0) v = 0;
        if (v > (float)QuadDir::kStop) v = (float)QuadDir::kStop;
        p[id] = v;
        SetDirection((QuadDir)(int)v);
        break;
    case 2:  // speed
        speed_ = ClampF(v, 0.0f, 1.0f);
        p[id] = speed_;
        break;
    case 16: case 17: case 18: case 19:
        hip_rev_[id - 16] = (v != 0.0f);
        break;
    case 20: case 21: case 22: case 23:
        knee_rev_[id - 20] = (v != 0.0f);
        break;
    case 24: case 25: case 26: case 27:
        hip_trim_[id - 24] = (int)v;
        break;
    case 28: case 29: case 30: case 31:
        knee_trim_[id - 28] = (int)v;
        break;
    default:
        break;  // 3..15 直接生效
    }
    Unlock();
    SaveParams();
    return true;
}

void QuadrupedController::SaveParams() {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed");
        return;
    }
    nvs_set_blob(h, KEY_PARAMS, &params_, sizeof(params_));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Params saved");
}

void QuadrupedController::ResetParams() {
    params_ = QuadParams();
    SyncFromParams();
    SaveParams();
    ESP_LOGI(TAG, "Params reset to defaults");
}

void QuadrupedController::LoadParamsFromNvs() {
    QuadParams def;
    params_ = def;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(params_);
    if (nvs_get_blob(h, KEY_PARAMS, &params_, &len) == ESP_OK && len == sizeof(params_)) {
        ESP_LOGI(TAG, "Params loaded from NVS");
    }
    nvs_close(h);
}

void QuadrupedController::SyncFromParams() {
    if (params_.gait < 0.5f) gait_ = QuadGait::kWalk;
    else if (params_.gait < 1.5f) gait_ = QuadGait::kWave;
    else gait_ = QuadGait::kTrot;
    float d = params_.direction;
    if (d < 0) d = 0;
    if (d > (float)QuadDir::kStop) d = (float)QuadDir::kStop;
    dir_ = (QuadDir)(int)d;
    speed_ = ClampF(params_.speed, 0.0f, 1.0f);
    for (int leg = 0; leg < 4; leg++) {
        hip_rev_[leg] = params_.hip_rev[leg] != 0.0f;
        knee_rev_[leg] = params_.knee_rev[leg] != 0.0f;
        hip_trim_[leg] = (int)params_.hip_trim[leg];
        knee_trim_[leg] = (int)params_.knee_trim[leg];
    }
}

// ---------------- 控制 ----------------

void QuadrupedController::SetGait(QuadGait g) {
    Lock();
    gait_ = g;
    pose_mode_ = false;   // 换步态 = 开始运动, 打断姿势
    Unlock();
    // 注: 不改 params_ (运行状态与持久配置分离, 步态偏好用 SetParam(0) 持久化)
}

void QuadrupedController::SetDirection(QuadDir d) {
    Lock();
    if (d == dir_ && d != QuadDir::kStop) { Unlock(); return; }  // 同方向忽略
    if (d != QuadDir::kStop) pose_mode_ = false;   // 运动命令打断姿势
    dir_ = d;
    // 注: 不改 params_ (方向是临时状态, 重启强制回 stop)
    if (d == QuadDir::kStop) {
        amp_target_ = 0.0f;
        ramp_state_ = 0;
    } else if (amp_ > 0.001f) {
        ramp_state_ = 1;
        amp_target_ = 0.0f;
    } else {
        ramp_state_ = 2;
        amp_target_ = 1.0f;
    }
    Unlock();
}

void QuadrupedController::SetSpeed(float s) {
    Lock();
    speed_ = ClampF(s, 0.0f, 1.0f);
    Unlock();
    // 注: 不改 params_ (速度是临时状态)
}

void QuadrupedController::SetServoReverse(int leg, bool hip_rev, bool knee_rev) {
    if (leg < 0 || leg > 3) return;
    Lock();
    hip_rev_[leg] = hip_rev;
    knee_rev_[leg] = knee_rev;
    params_.hip_rev[leg] = hip_rev ? 1.0f : 0.0f;
    params_.knee_rev[leg] = knee_rev ? 1.0f : 0.0f;
    Unlock();
    SaveParams();
}

void QuadrupedController::StartTween(Tween& tw, float start, float target) {
    tw.start = start;
    tw.target = target;
    tw.t = 0.0f;
    tw.active = true;
}

void QuadrupedController::SetPose(QuadPose p) {
    Lock();
    pose_ = p;
    pose_mode_ = true;
    SetDirection(QuadDir::kStop);   // 姿势前停止步态, 姿势结束后不会继续走
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
        {  0,   0,   0,   0,  20,  10, -20,  10},   // kWiggle 扭屁股(动画在Tick里)
    };
    int idx = (int)p;
    if (idx < 0 || idx > 8) idx = 0;
    // 角度限幅: 防姿势目标超出机械行程导致舵机堵转大电流 (TickGait已有, TickPose必须一致)
    float lim = params_.angle_limit;
    for (int leg = 0; leg < 4; leg++) {
        float th = ClampF(poses[idx][leg * 2], -lim, lim);
        float tk = ClampF(poses[idx][leg * 2 + 1], -lim, lim);
        // 用当前角 hip_cur_ 判断(而非上次目标 pose_hip_), 否则行走后立正(目标0)不触发回中
        if (th != hip_cur_[leg]) {
            pose_hip_[leg] = th;
            StartTween(hip_tw_[leg], hip_cur_[leg], th);
        }
        if (tk != knee_cur_[leg]) {
            pose_knee_[leg] = tk;
            StartTween(knee_tw_[leg], knee_cur_[leg], tk);
        }
    }
    Unlock();
    ESP_LOGI(TAG, "Set pose %d (angle_limit=%.0f)", idx, lim);
}

void QuadrupedController::Init() {
    mutex_ = xSemaphoreCreateRecursiveMutex();
    LoadParamsFromNvs();
    SyncFromParams();
    // 安全: 上电总是站立待命, 不恢复上次运动方向 (防上电乱走); 校准走 trim
    dir_ = QuadDir::kStop;
    amp_ = 0.0f;
    amp_target_ = 0.0f;
    ramp_state_ = 0;
    ESP_LOGI(TAG, "Quadruped ready, gait=%s dir=STOP speed=%.2f, 8 servos",
             gait_ == QuadGait::kWalk ? "WALK" : gait_ == QuadGait::kWave ? "WAVE" : "TROT",
             speed_);
}

void QuadrupedController::CalibrateNeutral() {
    // 把当前角度记为中立: 写入 trim(微调偏移), 然后当前角归零.
    // trim 是所有目标角度的基准偏移, 校准后全部动作自动跟随, 存 NVS 断电不丢.
    Lock();
    for (int leg = 0; leg < 4; leg++) {
        hip_trim_[leg] = (int)ClampF(roundf(hip_cur_[leg]), -30.0f, 30.0f);
        knee_trim_[leg] = (int)ClampF(roundf(knee_cur_[leg]), -30.0f, 30.0f);
        params_.hip_trim[leg] = (float)hip_trim_[leg];
        params_.knee_trim[leg] = (float)knee_trim_[leg];
        hip_cur_[leg] = 0.0f;
        knee_cur_[leg] = 0.0f;
        prev_delta_[leg][0] = 0.0f;
        prev_delta_[leg][1] = 0.0f;
    }
    Unlock();
    SaveParams();
    ESP_LOGI(TAG, "Neutral calibrated into trim & saved");
}

// ---------------- 运动学 ----------------

float QuadrupedController::SignedAngle(int leg, bool is_knee, float deg) {
    bool rev = is_knee ? knee_rev_[leg] : hip_rev_[leg];
    int trim = is_knee ? knee_trim_[leg] : hip_trim_[leg];
    return rev ? -(deg + trim) : (deg + trim);
}

void QuadrupedController::ApplyLeg(int leg, float hip_deg, float knee_deg) {
    // trim 可能叠加到 angle_limit 之外, clamp 到 MG90S 安全行程(避免 0/180 极限堵转)
    float h = 90.0f + SignedAngle(leg, false, hip_deg);
    float k = 90.0f + SignedAngle(leg, true, knee_deg);
    h = ClampF(h, 20.0f, 160.0f);
    k = ClampF(k, 20.0f, 160.0f);
    pca_->SetServoAngle(legs_[leg].hip, h);
    pca_->SetServoAngle(legs_[leg].knee, k);
}

// ---------------- Tick ----------------

void QuadrupedController::Tick() {
    Lock();
    TickAmpRamp();
    float tstep = 0.025f + speed_ * 0.05f;   // 缓动速度: 慢速时过渡更柔和
    if (pose_mode_) {
        TickPose(tstep);
    } else if (gait_ == QuadGait::kWalk) {
        TickWalk();
    } else {
        TickGait();
    }
    Unlock();
    // 上电软启动: 前4个tick依次加入各腿(每20ms一条, 80ms完成), 避免8舵机同时上电冲击电流
    if (boot_tick_ < 4) boot_tick_++;
    int nleg = (boot_tick_ < 4) ? boot_tick_ : 4;
    // 输出到舵机 (I2C 写, 锁外)
    for (int leg = 0; leg < 4; leg++) {
        if (leg < nleg) ApplyLeg(leg, hip_cur_[leg], knee_cur_[leg]);
    }
}

// 方向切换两段 ramp: 归零 -> 反向启动, 防抽筋
void QuadrupedController::TickAmpRamp() {
    float ramp_step = 0.06f + speed_ * 0.10f;
    if (amp_ < amp_target_) {
        amp_ += ramp_step;
        if (amp_ > amp_target_) amp_ = amp_target_;
    } else if (amp_ > amp_target_) {
        amp_ -= ramp_step;
        if (amp_ < amp_target_) amp_ = amp_target_;
    }
    if (ramp_state_ == 1 && amp_ <= 0.001f) {
        ramp_state_ = 2;      // 已归零, 反向启动
        amp_target_ = 1.0f;
    } else if (ramp_state_ == 2 && amp_ >= 0.999f) {
        ramp_state_ = 0;      // 完成
    }
}

// 姿势模式: smoothstep 缓动过渡 (贝塞尔式平滑, 零突变)
void QuadrupedController::TickPose(float tstep) {
    for (int leg = 0; leg < 4; leg++) {
        if (hip_tw_[leg].active) {
            hip_tw_[leg].t += tstep;
            if (hip_tw_[leg].t >= 1.0f) {
                hip_cur_[leg] = hip_tw_[leg].target;
                hip_tw_[leg].active = false;
            } else {
                hip_cur_[leg] = hip_tw_[leg].start +
                    (hip_tw_[leg].target - hip_tw_[leg].start) * SmoothStep(hip_tw_[leg].t);
            }
        }
        if (knee_tw_[leg].active) {
            knee_tw_[leg].t += tstep;
            if (knee_tw_[leg].t >= 1.0f) {
                knee_cur_[leg] = knee_tw_[leg].target;
                knee_tw_[leg].active = false;
            } else {
                knee_cur_[leg] = knee_tw_[leg].start +
                    (knee_tw_[leg].target - knee_tw_[leg].start) * SmoothStep(knee_tw_[leg].t);
            }
        }
    }

    // 扭屁股持续动画: 后腿髋交替摆动 (tween 每tick重启 = 低通跟随)
    if (pose_ == QuadPose::kWiggle) {
        phase_ += 0.025f;
        if (phase_ > 1.0f) phase_ -= 1.0f;
        float wig = sinf(phase_ * 4.0f * 3.14159f) * 25.0f;
        float t2 = 20.0f + wig;
        if (fabsf(t2 - pose_hip_[2]) > 0.1f) {
            pose_hip_[2] = t2;
            StartTween(hip_tw_[2], hip_cur_[2], t2);
        }
        float t3 = -20.0f - wig;
        if (fabsf(t3 - pose_hip_[3]) > 0.1f) {
            pose_hip_[3] = t3;
            StartTween(hip_tw_[3], hip_cur_[3], t3);
        }
    }

    // 姿势完成后保持 (不自动退出), 直到运动命令(前进/后退等)或立正/新姿势才改变
    // wiggle 持续动画, 其余姿势到位后原地保持
}

// 步态模式: 相位连续 + 限速 + 加速度限幅 + 角度限幅
void QuadrupedController::TickGait() {
    // 步态相位推进
    float pstep = (gait_ == QuadGait::kTrot) ? params_.phase_step : params_.phase_step_wave;
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

    float amp = params_.hip_amp * speed_ * amp_;   // 幅度(含方向切换 ramp)
    float kamp = params_.knee_amp * speed_ * amp_;
    float lim = params_.angle_limit;

    for (int leg = 0; leg < 4; leg++) {
        float ph = leg_ph[leg];
        float s = sinf(ph * 2.0f * 3.14159f);
        float swing = (s > 0.0f) ? s : 0.0f;

        float target_hip = 0, target_knee = 0;
        switch (dir_) {
        case QuadDir::kForward:
            target_hip = amp * s;
            target_knee = -kamp * swing;
            break;
        case QuadDir::kBackward:
            target_hip = -amp * s;
            target_knee = -kamp * swing;
            break;
        case QuadDir::kTurnLeft:
            target_hip = (leg == 0 || leg == 2) ? amp * s : -amp * s;
            target_knee = -kamp * swing;
            break;
        case QuadDir::kTurnRight:
            target_hip = (leg == 0 || leg == 2) ? -amp * s : amp * s;
            target_knee = -kamp * swing;
            break;
        case QuadDir::kShiftLeft:
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

        // 角度限幅 (相对中立)
        target_hip = ClampF(target_hip, -lim, lim);
        target_knee = ClampF(target_knee, -lim, lim);

        hip_cur_[leg] = SmoothJoint(leg, false, target_hip);
        knee_cur_[leg] = SmoothJoint(leg, true, target_knee);
    }
}

// 限速 + 加速度限幅 (平滑加速, 防暴冲)
float QuadrupedController::SmoothJoint(int leg, bool is_knee, float target) {
    float cur = is_knee ? knee_cur_[leg] : hip_cur_[leg];
    float smooth = params_.smooth_step;
    float alimit = params_.accel_limit;
    int idx = is_knee ? 1 : 0;
    float d = target - cur;
    if (fabsf(d) <= smooth) {
        prev_delta_[leg][idx] = 0.0f;
        return target;
    }
    float delta = (d > 0) ? smooth : -smooth;
    float dd = delta - prev_delta_[leg][idx];
    if (dd > alimit) delta = prev_delta_[leg][idx] + alimit;
    else if (dd < -alimit) delta = prev_delta_[leg][idx] - alimit;
    prev_delta_[leg][idx] = delta;
    return cur + delta;
}

// 2-DOF 腿逆运动学: 已知足端(x前,z下), 求髋角/膝角(度). z轴向下为正
bool QuadrupedController::SolveIK(float x, float z, float& hip_deg, float& knee_deg) {
    float l1 = params_.l1;
    float l2 = params_.l2;
    float d = sqrtf(x * x + z * z);
    if (d > l1 + l2) d = l1 + l2;              // 超程 clamp 到可达边界
    if (d < fabsf(l1 - l2)) d = fabsf(l1 - l2);
    float cos_k = (d * d - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    cos_k = ClampF(cos_k, -1.0f, 1.0f);
    float knee = acosf(cos_k);                 // 膝弯曲角 (0=伸直)
    // 髋角 = 髋-足连线角 + 大腿与连线夹角 (标准2-DOF腿IK, z轴向下, 加号)
    float hip = atan2f(x, z) + atan2f(l2 * sinf(knee), l1 + l2 * cosf(knee));
    hip_deg = hip * 180.0f / 3.14159265f;      // 髋角: 0=腿垂直, 正=向前
    // 膝角: 负=弯曲 (与 WAVE/TROT 的 target_knee=-kamp*swing 符号一致)
    knee_deg = -knee * 180.0f / 3.14159265f;
    return true;
}

// WALK 步态: 逆运动学 + 摆线足端轨迹 + 恒机身高度 (最稳, 3腿着地)
void QuadrupedController::TickWalk() {
    float pstep = params_.phase_step_wave;
    phase_ += pstep * speed_;              // speed=0 -> 停 (与 WAVE/TROT 一致)
    if (phase_ > 1.0f) phase_ -= 1.0f;

    // Walk 对角落腿序列相位 (LF, RF, LH, RH)
    static const float walk_ph[4] = {0.0f, 0.5f, 0.25f, 0.75f};
    float duty = params_.duty;
    float S = params_.step_len * speed_ * amp_;   // 步幅 (含速度与方向切换 ramp)
    float H = params_.lift_height;
    float z0 = params_.body_height;

    for (int leg = 0; leg < 4; leg++) {
        float p = phase_ + walk_ph[leg];
        if (p >= 1.0f) p -= 1.0f;

        float fx, fz;
        if (p < duty) {
            // 支撑相: 足端相对身体向后直线移动, z恒定(机身不颠)
            float t = p / duty;
            fx = S / 2.0f - S * t;
            fz = z0;
        } else {
            // 摆动相: 摆线, 起落点速度=0(零冲击), 抬腿H
            float u = (p - duty) / (1.0f - duty) * 2.0f * 3.14159265f;
            fx = -S / 2.0f + (S / (2.0f * 3.14159265f)) * (u - sinf(u));
            fz = z0 - (H / 2.0f) * (1.0f - cosf(u));
        }

        // 方向: 前进正常, 后退反向, 原地转左右腿反向 (2-DOF无横移)
        float x = fx;
        if (dir_ == QuadDir::kBackward) x = -fx;
        else if (dir_ == QuadDir::kTurnLeft) x = (leg == 0 || leg == 2) ? fx : -fx;
        else if (dir_ == QuadDir::kTurnRight) x = (leg == 0 || leg == 2) ? -fx : fx;
        else if (dir_ == QuadDir::kShiftLeft || dir_ == QuadDir::kShiftRight) x = 0;
        else if (dir_ == QuadDir::kStop) { x = 0; fz = params_.l1 + params_.l2; }  // 停止: 直腿立正
        // kForward: 用 fx

        float hip_deg, knee_deg;
        SolveIK(x, fz, hip_deg, knee_deg);
        float lim = params_.angle_limit;
        hip_deg = ClampF(hip_deg, -lim, lim);           // 髋限幅
        knee_deg = ClampF(knee_deg, -85.0f, 0.0f);      // 膝弯曲限幅(独立, 抬腿需~79°, 不卡angle_limit)

        hip_cur_[leg] = SmoothJoint(leg, false, hip_deg);
        knee_cur_[leg] = SmoothJoint(leg, true, knee_deg);
    }
}
