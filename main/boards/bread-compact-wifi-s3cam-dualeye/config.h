#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 如果使用 Duplex I2S 模式，请注释下面一行
#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_1
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_2
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_42
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_39
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_40
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_41

#else

#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7

#endif

// GPIO48 用作屏幕 DC 信号, 与板载 RGB 灯珠(也在48)冲突 => 弃用状态灯
#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define TOUCH_BUTTON_GPIO       GPIO_NUM_4   // TTP223 触摸: 点一下唤醒
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC

// ================================================================
//  0.71寸 GC9D01 圆屏 (0.71-12H 转接板) — 实测锁定接线:
//    脚7=DC  脚8=CS  脚9=SCK  脚10=MOSI  脚11=RST  脚2=LEDK(背光)
//    (原假设 MOSI=脚11 是错误的, 全排列扫描实测确定)
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
#define DISPLAY_INVERT_COLOR  false  // inversion 已由 vendor 序列 0xEC 0x70 处理, 不再额外反色
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB

#endif // _BOARD_CONFIG_H_
