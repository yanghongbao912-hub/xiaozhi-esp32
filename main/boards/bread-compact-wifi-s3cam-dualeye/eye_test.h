#pragma once

#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Dual-init test: 4 combos crossing two pin mappings (v3, seller)
 *        with two init sequences (GC9A01 default, GC9D01), each a solid
 *        color for 5 seconds, looping:
 *          1 RED    v3 mapping (SCK21 RST45 DC48 CS47) + GC9A01 init
 *          2 GREEN  v3 mapping + GC9D01 init
 *          3 BLUE   seller mapping (SCK48 RST21 DC47 CS45) + GC9A01 init
 *          4 WHITE  seller mapping + GC9D01 init
 */
void eye_test_permutation_start(void);

#ifdef __cplusplus
}
#endif
