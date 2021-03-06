#ifndef __ASM_ARCH_PXA27x_KEYPAD_H
#define __ASM_ARCH_PXA27x_KEYPAD_H

#include <linux/input.h>

#define MAX_MATRIX_KEY_ROWS	(8)
#define MAX_MATRIX_KEY_COLS	(8)

/* pxa3xx keypad platform specific parameters
 *
 * NOTE:
 * 1. direct_key_num indicates the number of keys in the direct keypad
 *    _plus_ the number of rotary-encoder sensor inputs,  this can be
 *    left as 0 if only rotary encoders are enabled,  the driver will
 *    automatically calculate this
 *
 * 2. direct_key_map is the key code map for the direct keys, if rotary
 *    encoder(s) are enabled, direct key 0/1(2/3) will be ignored
 *
 * 3. rotary can be either interpreted as a relative input event (e.g.
 *    REL_WHEEL/REL_HWHEEL) or specific keys (e.g. UP/DOWN/LEFT/RIGHT)
 *
 * 4. matrix key and direct key will use the same debounce_interval by
 *    default, which should be sufficient in most cases
 */
struct pxa27x_keypad_platform_data {

	/* code map for the matrix keys */
	unsigned int	matrix_key_rows;
	unsigned int	matrix_key_cols;
	unsigned int	*matrix_key_map;
	int		matrix_key_map_size;

	/* direct keys */
	int		direct_key_num;
	unsigned int	*direct_key_map;

	/* rotary encoders 0 */
	int		enable_rotary0;
	int		rotary0_rel_code;
	int		rotary0_up_key;
	int		rotary0_down_key;

	/* rotary encoders 1 */
	int		enable_rotary1;
	int		rotary1_rel_code;
	int		rotary1_up_key;
	int		rotary1_down_key;

#ifdef CONFIG_CPU_PXA930
	/* enhanced rotary encoder */
	int		enable_enhanced_rotary;
	int		enhanced_rotary_rel_code;
	int		enhanced_rotary_up_key;
	int		enhanced_rotary_down_key;
#endif

	/* key debounce interval */
	unsigned int	debounce_interval;

	int slideint_mfp;
};

#define KEY(row, col, val)	(((row) << 28) | ((col) << 24) | (val))

extern void pxa_set_keypad_info(struct pxa27x_keypad_platform_data *info);

#endif /* __ASM_ARCH_PXA27x_KEYPAD_H */
