#pragma once

#include "pca9685.h"

/**
 * @brief 四足运动控制器 (8 舵机: 每条腿 髋+膝)
 *
 * 支持:
 *  - 两种步态: WAVE(波浪,3腿着地稳) / TROT(对角,2腿着地快)
 *  - 六个方向: 前进/后退/左转/右转/左平移/右平移
 *  - 平滑加速/零突变切换/防暴冲(分两段执行)
 *  - 9 种预设姿势 + 中立位校准(存 NVS)
 *  - 每舵机反转配置(机械装反时)
 */

// 舵机通道映射 (可改): [腿0..3 的 髋/膝]
// 腿: 0=左前 1=右前 2=左后 3=右后
struct QuadLegCh {
    int hip;    // 髋关节通道
    int knee;   // 膝关节通道
};

// 步态
enum class QuadGait {
    kWave,   // 波浪: 每次抬一腿
    kTrot,   // 对角: 对角两腿同步
};

// 方向
enum class QuadDir {
    kForward, kBackward, kTurnLeft, kTurnRight, kShiftLeft, kShiftRight, kStop
};

// 姿势
enum class QuadPose {
    kNeutral,      // 立正
    kLieDown,      // 趴下
    kPuppy,        // 小狗趴
    kSit,          // 蹲下/坐下
    kRelax,        // 稍息
    kBattle,       // 战斗姿态
    kHandshakeL,   // 握手(左前)
    kHandshakeR,   // 握手(右前)
    kWiggle,       // 扭屁股
};

class QuadrupedController {
public:
    QuadrupedController(Pca9685* pca);

    void Init();
    void Tick();                       // 周期调用, 平滑更新舵机
    void SetGait(QuadGait g) { gait_ = g; }
    void SetDirection(QuadDir d) { dir_ = d; }
    void SetSpeed(float s);            // 0~1
    void SetPose(QuadPose p);          // 切姿势(平滑过渡)
    void CalibrateNeutral();           // 把当前角度记为中立(存NVS)
    void SetServoReverse(int leg, bool hip_rev, bool knee_rev);  // 舵机反转

    // 调试/配置
    void SetLegCh(int leg, int hip_ch, int knee_ch);
    void SetTrim(int leg, int hip_trim, int knee_trim);  // 中立微调(度)

private:
    Pca9685* pca_ = nullptr;

    QuadLegCh legs_[4] = {
        {0, 1},   // 左前 髋0 膝1
        {2, 3},   // 右前 髋2 膝3
        {4, 5},   // 左后 髋4 膝5
        {6, 7},   // 右后 髋6 膝7
    };
    bool hip_rev_[4] = {false, true, false, true};   // 右腿默认反转(镜像)
    bool knee_rev_[4] = {false, true, false, true};
    int hip_trim_[4] = {0, 0, 0, 0};
    int knee_trim_[4] = {0, 0, 0, 0};

    QuadGait gait_ = QuadGait::kTrot;
    QuadDir dir_ = QuadDir::kStop;
    float speed_ = 0.5f;

    // 平滑状态
    float hip_cur_[4] = {0, 0, 0, 0};
    float knee_cur_[4] = {0, 0, 0, 0};
    float phase_ = 0.0f;         // 步态相位 0~1
    bool pose_mode_ = false;
    QuadPose pose_ = QuadPose::kNeutral;
    float pose_hip_[4] = {0, 0, 0, 0};
    float pose_knee_[4] = {0, 0, 0, 0};

    void LoadNeutralFromNvs();
    void ApplyLeg(int leg, float hip_deg, float knee_deg);
    float SignedAngle(int leg, bool is_knee, float deg);
};
