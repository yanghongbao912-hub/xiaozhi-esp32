#pragma once

#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start a permutation test: tries 6 combinations of RST/DC/CS,
 *        each drawing a distinct solid color for 5 seconds, looping.
 *        Whichever combo shows a clean full-screen color is the correct
 *        mapping. Combos:
 *          1 RED    RST=45 DC=48 CS=47
 *          2 GREEN  RST=45 DC=47 CS=48
 *          3 BLUE   RST=48 DC=45 CS=47
 *          4 WHITE  RST=48 DC=47 CS=45
 *          5 YELLOW RST=47 DC=45 CS=48
 *          6 CYAN   RST=47 DC=48 CS=45
 */
void eye_test_permutation_start(void);

#ifdef __cplusplus
}
#endif
