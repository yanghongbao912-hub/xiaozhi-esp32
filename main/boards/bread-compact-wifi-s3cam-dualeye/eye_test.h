#pragma once

#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Exhaustive permutation test: MOSI=13 fixed, SCK/RST/DC/CS take all
 *        24 permutations of {21,45,47,48}. Each combo draws a unique hue for
 *        3 seconds (HSV wheel), looping forever. The first combo that shows a
 *        clean full-screen color is the correct mapping; report the hue and it
 *        maps to combo #N in the boot log.
 */
void eye_test_permutation_start(void);

#ifdef __cplusplus
}
#endif
