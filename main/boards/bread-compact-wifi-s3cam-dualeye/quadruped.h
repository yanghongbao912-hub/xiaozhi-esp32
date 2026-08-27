#pragma once

#include "pca9685.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @brief 四足运动控制器 (8 舵机: 每条腿 髋+膝)
 *
 * 借鉴 Petoi OpenCat (Bittle) 的步态引擎思路:
 *  - 两种步态: WAVE(波浪,3腿着地稳) / TROT(对角,2腿着地快)
 *  - 六个方向: 前进/后退/左转/右转/左平移/右平移
 *  - smoothstep 缓动: 姿势切换用贝塞尔式平滑插值, 零突变
 *  - 零突变方向切换: 换方向先幅度归零再反向(两段), 不会抽筋
 *  - 平滑加速: 限速 + 加速度限幅, 速度从 0 到 100% 连续
 *  - 防暴冲: 大角度动作经 ramp + tween 分两段平滑执行
 *  - 9 种预设姿势 + 中立位校准(存 NVS 断电不丢)
 *  - 26 项参数表(步幅/抬腿高度/步频/速度/角度限幅/舵机反转等)
 *    支持网页/串口/AI 通过 SetParam(id, value) 调整, 存 NVS
 */

// 舵机通道映射 (可改): [腿0..3 的 髋/膝]
// 腿: 0=左前 1=右前 2=左后 3=右后
struct QuadLegCh {
    int hip;    // 髋关节通道
    int knee;   // 膝关节通道
};

// 步态
enum class QuadGait {
    kWalk,   // 对角步行(3腿着地, 最稳, 逆运动学+摆线足端轨迹)
    kWave,   // 波浪: 每次抬一腿(简谐)
    kTrot,   // 对角: 对角两腿同步(快, 需大扭矩)
};

// 方向 (与参数表 direction 的值一一对应: 0..6)
enum class QuadDir {
    kForward, kBackward, kTurnLeft, kTurnRight, kShiftLeft, kShiftRight, kStop
};

// 姿势
enum class QuadPose {
    kNeutral,      // 立正 (说停就停)
    kLieDown,      // 趴下
    kPuppy,        // 小狗趴
    kSit,          // 蹲下/坐下
    kRelax,        // 稍息
    kBattle,       // 战斗姿态
    kHandshakeL,   // 握手(左前)
    kHandshakeR,   // 握手(右前)
    kWiggle,       // 扭屁股 (持续动画)
};

// 可配置参数表: 31 项连续 float, 存 NVS 断电不丢
struct QuadParams {
    // id 0..9 运动参数
    float gait = 0.0f;               //  0: 0=WALK(对角步行,逆运动学) 1=WAVE 2=TROT
    float direction = 6.0f;          //  1: 0前进 1后退 2左转 3右转 4左移 5右移 6停
    float speed = 0.5f;              //  2: 速度 0~1 (平滑加速)
    float hip_amp = 45.0f;           //  3: 步幅(髋摆幅, 度; 仅WAVE/TROT用)
    float knee_amp = 40.0f;          //  4: 抬腿高度(膝弯曲, 度; 仅WAVE/TROT用)
    float phase_step = 0.035f;       //  5: 步频 TROT (相位/tick)
    float phase_step_wave = 0.015f;  //  6: 步频 WAVE
    float smooth_step = 3.0f;        //  7: 每tick最大角度变化(度, 防暴冲)
    float accel_limit = 1.5f;        //  8: 加速度限幅(度/tick², 平滑加速)
    float angle_limit = 60.0f;       //  9: 角度限幅(相对中立, 度)
    // id 10..15 逆运动学几何 (WALK 步态)
    float l1 = 40.0f;                // 10: 大腿长(髋到膝, mm)
    float l2 = 60.0f;                // 11: 小腿长(膝到足, mm)
    float body_height = 90.0f;       // 12: 机身高度(髋到地, mm, 略小于l1+l2)
    float step_len = 30.0f;          // 13: 步幅(足端前后行程, mm)
    float lift_height = 12.0f;       // 14: 抬腿高度(摆动相, mm)
    float duty = 0.75f;              // 15: 占空比(支撑相比例, 0.5~0.9)
    // id 16..31 每舵机配置
    float hip_rev[4] = {0, 1, 0, 1};   // 16-19: 髋舵机反转(机械装反时)
    float knee_rev[4] = {0, 1, 0, 1};  // 20-23: 膝舵机反转
    float hip_trim[4] = {0, 0, 0, 0};  // 24-27: 髋中立微调(度)
    float knee_trim[4] = {0, 0, 0, 0}; // 28-31: 膝中立微调(度)
};

class QuadrupedController {
public:
    QuadrupedController(Pca9685* pca);

    void Init();
    void Tick();                       // 周期调用 (建议 20ms)

    // ---- 控制接口 ----
    void SetGait(QuadGait g);
    void SetDirection(QuadDir d);      // 零突变: 自动先归零再反向
    void SetSpeed(float s);            // 0~1
    void SetPose(QuadPose p);          // 平滑过渡, kNeutral 随时打断
    void CalibrateNeutral();           // 把当前角度记为中立(存NVS)
    void SetServoReverse(int leg, bool hip_rev, bool knee_rev);

    // ---- 参数表 (网页/串口/AI) ----
    static int ParamCount() { return 32; }
    static const char* ParamName(int id);
    bool SetParam(int id, float v);    // 同步内部状态并持久化
    float GetParam(int id) const;
    void SaveParams();
    void ResetParams();

private:
    Pca9685* pca_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;   // 递归互斥: 保护共享状态(Tick任务 vs MCP/Demo回调)
    void Lock() { if (mutex_) xSemaphoreTakeRecursive(mutex_, portMAX_DELAY); }
    void Unlock() { if (mutex_) xSemaphoreGiveRecursive(mutex_); }

    QuadLegCh legs_[4] = {
        {0, 1},   // 左前 髋0 膝1
        {2, 3},   // 右前 髋2 膝3
        {4, 5},   // 左后 髋4 膝5
        {6, 7},   // 右后 髋6 膝7
    };
    QuadParams params_;

    // 运行状态 (由参数表驱动, 供快速访问)
    bool hip_rev_[4] = {false, true, false, true};
    bool knee_rev_[4] = {false, true, false, true};
    int hip_trim_[4] = {0, 0, 0, 0};
    int knee_trim_[4] = {0, 0, 0, 0};
    QuadGait gait_ = QuadGait::kWalk;
    QuadDir dir_ = QuadDir::kStop;
    float speed_ = 0.5f;

    // 平滑状态
    float hip_cur_[4] = {0, 0, 0, 0};       // 当前输出角度(相对中立)
    float knee_cur_[4] = {0, 0, 0, 0};
    float prev_delta_[4][2] = {{0}};        // 上一tick角度变化(加速度限幅用)
    float phase_ = 0.0f;                    // 步态相位 0~1
    int boot_tick_ = 0;                     // 上电软启动计数

    // 姿势 tween (smoothstep 缓动)
    bool pose_mode_ = false;
    QuadPose pose_ = QuadPose::kNeutral;
    float pose_hip_[4] = {0, 0, 0, 0};
    float pose_knee_[4] = {0, 0, 0, 0};
    struct Tween {
        float start = 0, target = 0, t = 0;
        bool active = false;
    };
    Tween hip_tw_[4], knee_tw_[4];

    // 方向切换两段 ramp: 0=空闲 1=归零 2=反向启动
    int ramp_state_ = 0;
    float amp_ = 1.0f, amp_target_ = 1.0f;  // 幅度系数 0~1

    void LoadParamsFromNvs();
    void SyncFromParams();
    void ApplyLeg(int leg, float hip_deg, float knee_deg);
    float SignedAngle(int leg, bool is_knee, float deg);
    float SmoothStep(float t) { return t * t * (3.0f - 2.0f * t); }
    void StartTween(Tween& tw, float start, float target);
    void TickAmpRamp();
    void TickPose(float tstep);
    void TickGait();
    void TickWalk();                       // WALK: 逆运动学 + 摆线足端轨迹
    bool SolveIK(float x, float z, float& hip_deg, float& knee_deg);
    float SmoothJoint(int leg, bool is_knee, float target);  // 限速+加速度限幅
};
