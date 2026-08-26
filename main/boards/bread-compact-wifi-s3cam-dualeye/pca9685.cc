#include "pca9685.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#define TAG "Pca9685"

// PCA9685 寄存器
#define PCA9685_MODE1    0x00
#define PCA9685_MODE2    0x01
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L  0x06   // 通道0起始, 每通道+4

#define PCA9685_OSC_FREQ 25000000UL   // 25MHz
#define PCA9685_SERVO_HZ 50           // 舵机 50Hz

Pca9685::Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {}

void Pca9685::WriteRegs(uint8_t reg, const uint8_t* data, size_t len) {
    uint8_t buf[16];
    if (1 + len > sizeof(buf)) {
        return;
    }
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    i2c_master_transmit(i2c_device_, buf, 1 + len, 100);
}

esp_err_t Pca9685::Init() {
    // 进入 sleep 才能写 PRESCALE
    WriteReg(PCA9685_MODE1, 0x10);  // SLEEP=1
    // 预分频: 25MHz / (4096 * 50Hz) = 122.07 -> 121
    uint8_t prescale = (uint8_t)(PCA9685_OSC_FREQ / (4096UL * PCA9685_SERVO_HZ)) - 1;
    WriteReg(PCA9685_PRESCALE, prescale);
    // MODE2: OUTDRV=1 (totem-pole, 驱动能力强)
    WriteReg(PCA9685_MODE2, 0x04);
    // MODE1: 唤醒 + 寄存器自动递增
    WriteReg(PCA9685_MODE1, 0x20);  // SLEEP=0, AI=1
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "Pca9685 ready @0x%02X, prescale=%u (%uHz)", device_address_, prescale, PCA9685_SERVO_HZ);
    return ESP_OK;
}

void Pca9685::SetServoPulseUs(uint8_t channel, uint16_t pulse_us) {
    if (channel > 15) return;
    // 50Hz: 周期20ms = 4096 步, 脉宽 -> OFF 计数
    uint16_t off = (uint16_t)((uint32_t)pulse_us * 4096 / 20000);
    uint8_t data[4] = {
        0x00, 0x00,               // ON = 0
        (uint8_t)(off & 0xFF),    // OFF_L
        (uint8_t)(off >> 8),      // OFF_H
    };
    WriteRegs(PCA9685_LED0_ON_L + channel * 4, data, 4);
}

void Pca9685::SetServoAngle(uint8_t channel, float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    // MG90S: 0.5ms~2.5ms 对应 0~180度
    uint16_t pulse_us = (uint16_t)(500.0f + angle / 180.0f * 2000.0f);
    SetServoPulseUs(channel, pulse_us);
}
