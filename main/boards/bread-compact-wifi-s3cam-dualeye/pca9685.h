#pragma once

#include "i2c_device.h"

#include <esp_err.h>

/**
 * @brief PCA9685 16路 PWM 舵机驱动板 (I2C)
 *
 * 50Hz 更新率, 每通道 12-bit (4096 步), 驱动 8 个 MG90S 舵机
 * 默认地址 0x40
 */
class Pca9685 : public I2cDevice {
public:
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x40);

    esp_err_t Init();
    void SetServoAngle(uint8_t channel, float angle);        // 0~180度
    void SetServoPulseUs(uint8_t channel, uint16_t pulse_us); // 直接设脉宽(us)

private:
    void WriteRegs(uint8_t reg, const uint8_t* data, size_t len);
};
