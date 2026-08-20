#pragma once

#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start a lively eye animation test on the given panel.
 *        Draws an animated eye (pupil scan + blink) continuously.
 */
void eye_test_start(esp_lcd_panel_handle_t panel);

#ifdef __cplusplus
}
#endif
