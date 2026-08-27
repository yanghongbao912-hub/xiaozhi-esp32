#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// ================================================================
//  ★ GPIO 总览 (一张表看懂所有引脚) ★
// ================================================================
//  GPIO0   BOOT按键        GPIO21  屏 MOSI
//  GPIO1   麦克风 WS       GPIO38  屏 RST
//  GPIO2   麦克风 SCK      GPIO39  喇叭 DIN
//  GPIO3   屏 CS           GPIO40  喇叭 BCLK
//  GPIO4   TTP223触摸      GPIO41  喇叭 LRC
//  GPIO8   PCA9685 SDA     GPIO42  麦克风 SD
//  GPIO9   PCA9685 SCL     GPIO43  串口TX(系统)
//  GPIO14  屏背光          GPIO44  串口RX(系统)
//  GPIO45  屏 DC
//  GPIO47  屏 SCK
//  GPIO48  板载RGB(禁用)
//  ----------------------------------------------------------------
//  空闲可用: 5,6,7,10,11,12,15~20,22~37,46
// ================================================================

// ================================================================
//  音频 (I2S)
// ================================================================
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 如果使用 Duplex I2S 模式，请注释下面一行
#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_1    // 麦克风字选择 (INMP441 WS)
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_2    // 麦克风时钟 (INMP441 SCK)
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_42   // 麦克风数据 (INMP441 SD)
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_39   // 喇叭数据 (MAX98357 DIN)
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_40   // 喇叭时钟 (MAX98357 BCLK)
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_41   // 喇叭字选择 (MAX98357 LRC)

#else

#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7

#endif

// ================================================================
//  按键 / 触摸 / LED
// ================================================================
// GPIO48 与板载 RGB 灯珠冲突 => 弃用状态灯
#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define BOOT_BUTTON_GPIO        GPIO_NUM_0    // BOOT 按键
#define TOUCH_BUTTON_GPIO       GPIO_NUM_4    // TTP223 触摸 (点一下唤醒, active-high)
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC

// ================================================================
//  显示 — 0.71寸 GC9D01 双目屏 (0.71-12H 转接板)
//  实测锁定: 脚7=DC 脚8=CS 脚9=SCK 脚10=MOSI 脚11=RST 脚2=LEDK
// ================================================================
// #define DUALEYE_TEST_EYE  // 测试模式(已恢复正常显示模式)

#define DISPLAY_WIDTH   160
#define DISPLAY_HEIGHT  160

#define DISPLAY_MOSI_PIN      GPIO_NUM_21   // FPC 10 脚
#define DISPLAY_CLK_PIN       GPIO_NUM_47   // FPC 9 脚
#define DISPLAY_DC_PIN        GPIO_NUM_45   // FPC 7 脚
#define DISPLAY_RST_PIN       GPIO_NUM_38   // FPC 11 脚
#define DISPLAY_CS_PIN        GPIO_NUM_3    // FPC 8 脚

#define DISPLAY_BACKLIGHT_PIN           GPIO_NUM_14   // 经 Q1 低边开关驱动 FPC 2 脚 LEDK
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false         // 实测: 高电平点亮

#define DISPLAY_MIRROR_X      false
#define DISPLAY_MIRROR_Y      false
#define DISPLAY_SWAP_XY       false
#define DISPLAY_OFFSET_X      0
#define DISPLAY_OFFSET_Y      0
#define DISPLAY_SPI_MODE      0
#define DISPLAY_INVERT_COLOR  false  // inversion 已由 vendor 序列 0xEC 0x70 处理
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB

// ================================================================
//  舵机 — PCA9685 驱动板 (I2C) — 8 个 MG90S 四足
// ================================================================
#define SERVO_SDA_PIN        GPIO_NUM_8    // → PCA9685 SDA
#define SERVO_SCL_PIN        GPIO_NUM_9    // → PCA9685 SCL
#define PCA9685_I2C_ADDR     0x40          // PCA9685 默认地址

#endif // _BOARD_CONFIG_H_
