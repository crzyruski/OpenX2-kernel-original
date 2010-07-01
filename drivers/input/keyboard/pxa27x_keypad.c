/*
 * linux/drivers/input/keyboard/pxa27x_keypad.c
 *
 * Driver for the pxa27x matrix keyboard controller.
 *
 * Created:	Feb 22, 2007
 * Author:	Rodolfo Giometti <giometti@linux.it>
 *
 * Based on a previous implementations by Kevin O'Connor
 * <kevin_at_koconnor.net> and Alex Osborne <bobofdoom@gmail.com> and
 * on some suggestions by Nicolas Pitre <nico@cam.org>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/pxa27x_keypad.h>
#include <mach/camera.h>

#if defined(CONFIG_PXA3xx_DVFM)
#include <linux/notifier.h>
#include <linux/timer.h>
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#include <mach/pxa3xx_pm.h>
#endif
#include <mach/mfp.h>
#include <mach/gpio.h>

/*
 * Keypad Controller registers
 */
#define KPC             0x0000 /* Keypad Control register */
#define KPDK            0x0008 /* Keypad Direct Key register */
#define KPREC           0x0010 /* Keypad Rotary Encoder register */
#define KPMK            0x0018 /* Keypad Matrix Key register */
#define KPAS            0x0020 /* Keypad Automatic Scan register */

/* Keypad Automatic Scan Multiple Key Presser register 0-3 */
#define KPASMKP0        0x0028
#define KPASMKP1        0x0030
#define KPASMKP2        0x0038
#define KPASMKP3        0x0040
#define KPKDI           0x0048

#ifdef CONFIG_CPU_PXA930
#define ERCR_OFF	0xc
#define SBCR_OFF	0x4
#endif

/* bit definitions */
#define KPC_MKRN(n)	((((n) - 1) & 0x7) << 26) /* matrix key row number */
#define KPC_MKCN(n)	((((n) - 1) & 0x7) << 23) /* matrix key column number */
#define KPC_DKN(n)	((((n) - 1) & 0x7) << 6)  /* direct key number */

#define KPC_AS          (0x1 << 30)  /* Automatic Scan bit */
#define KPC_ASACT       (0x1 << 29)  /* Automatic Scan on Activity */
#define KPC_MI          (0x1 << 22)  /* Matrix interrupt bit */
#define KPC_IMKP        (0x1 << 21)  /* Ignore Multiple Key Press */

#define KPC_MS(n)	(0x1 << (13 + (n)))	/* Matrix scan line 'n' */
#define KPC_MS_ALL      (0xff << 13)

#define KPC_ME          (0x1 << 12)  /* Matrix Keypad Enable */
#define KPC_MIE         (0x1 << 11)  /* Matrix Interrupt Enable */
#define KPC_DK_DEB_SEL	(0x1 <<  9)  /* Direct Keypad Debounce Select */
#define KPC_DI          (0x1 <<  5)  /* Direct key interrupt bit */
#define KPC_RE_ZERO_DEB (0x1 <<  4)  /* Rotary Encoder Zero Debounce */
#define KPC_REE1        (0x1 <<  3)  /* Rotary Encoder1 Enable */
#define KPC_REE0        (0x1 <<  2)  /* Rotary Encoder0 Enable */
#define KPC_DE          (0x1 <<  1)  /* Direct Keypad Enable */
#define KPC_DIE         (0x1 <<  0)  /* Direct Keypad interrupt Enable */

#define KPDK_DKP        (0x1 << 31)
#define KPDK_DK(n)	((n) & 0xff)

#define KPREC_OF1       (0x1 << 31)
#define kPREC_UF1       (0x1 << 30)
#define KPREC_OF0       (0x1 << 15)
#define KPREC_UF0       (0x1 << 14)

#define KPREC_RECOUNT0(n)	((n) & 0xff)
#define KPREC_RECOUNT1(n)	(((n) >> 16) & 0xff)

#define KPMK_MKP        (0x1 << 31)
#define KPAS_SO         (0x1 << 31)
#define KPASMKPx_SO     (0x1 << 31)

#define KPAS_MUKP(n)	(((n) >> 26) & 0x1f)
#define KPAS_RP(n)	(((n) >> 4) & 0xf)
#define KPAS_CP(n)	((n) & 0xf)

#define KPASMKP_MKC_MASK	(0xff)

#define keypad_readl(off)	__raw_readl(keypad->mmio_base + (off))
#define keypad_writel(off, v)	__raw_writel((v), keypad->mmio_base + (off))

#ifdef CONFIG_CPU_PXA930
#define ercr_readl(off)		__raw_readl(keypad->ercr_base + (off))
#define ercr_writel(off, v)	__raw_writel((v), keypad->ercr_base + (off))
#endif

#define MAX_MATRIX_KEY_NUM	(8 * 8)

#define KEY_FOCUS	KEY_SOUND

#if defined(CONFIG_PXA3xx_DVFM)
#define D2_STABLE_JIFFIES               6
static int keypad_notifier_freq(struct notifier_block *nb,
		unsigned long val, void *data);
static struct notifier_block notifier_freq_block = {
	.notifier_call = keypad_notifier_freq,
};

static struct dvfm_lock dvfm_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.dev_idx	= -1,
	.count		= 0,
};

static struct timer_list kp_timer;
#endif

enum DETECT_FOCUS_STAGE{
	DETECT_FOCUS_NONE,
	DETECT_FOCUS_DOWN,
	DETECT_FOCUS_UP,
};

struct pxa27x_keypad {
	struct pxa27x_keypad_platform_data *pdata;

	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;
#ifdef CONFIG_CPU_PXA930
	void __iomem *ercr_base;
	u32 prev_ercr;
#endif

	int irq;

	/* matrix key code map */
	unsigned int matrix_keycodes[MAX_MATRIX_KEY_NUM];

	/* state row bits of each column scan */
	uint32_t matrix_key_state[MAX_MATRIX_KEY_COLS];
	uint32_t direct_key_state;

	unsigned int direct_key_mask;

	int rotary_rel_code[2];
	int rotary_up_key[2];
	int rotary_down_key[2];
#ifdef CONFIG_CPU_PXA930
	int enhanced_rotary_rel_code;
	int enhanced_rotary_up_key;
	int enhanced_rotary_down_key;
#endif

	int direct_focus_to_enter; /* only support half-press */
	enum DETECT_FOCUS_STAGE detect_focus_stage;
	struct work_struct report_enter_work;
	
	int slideint_mfp;
	int slide_state;
};

static void pxa27x_keypad_build_keycode(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	struct input_dev *input_dev = keypad->input_dev;
	unsigned int *key;
	int i;

	key = &pdata->matrix_key_map[0];
	for (i = 0; i < pdata->matrix_key_map_size; i++, key++) {
		int row = ((*key) >> 28) & 0xf;
		int col = ((*key) >> 24) & 0xf;
		int code = (*key) & 0xffffff;

		keypad->matrix_keycodes[(row << 3) + col] = code;
		set_bit(code, input_dev->keybit);
	}

	/* register direct key */
	key = pdata->direct_key_map;
	for (i = 0; i < pdata->direct_key_num; i++, key++) {
		set_bit(*key, input_dev->keybit);
	}

	keypad->rotary_up_key[0] = pdata->rotary0_up_key;
	keypad->rotary_up_key[1] = pdata->rotary1_up_key;
	keypad->rotary_down_key[0] = pdata->rotary0_down_key;
	keypad->rotary_down_key[1] = pdata->rotary1_down_key;
	keypad->rotary_rel_code[0] = pdata->rotary0_rel_code;
	keypad->rotary_rel_code[1] = pdata->rotary1_rel_code;
#ifdef CONFIG_CPU_PXA930
	keypad->enhanced_rotary_up_key = pdata->enhanced_rotary_up_key;
	keypad->enhanced_rotary_down_key = pdata->enhanced_rotary_down_key;
	keypad->enhanced_rotary_rel_code = pdata->enhanced_rotary_rel_code;
#endif

	if (pdata->enable_rotary0) {
		if (pdata->rotary0_up_key && pdata->rotary0_down_key) {
			set_bit(pdata->rotary0_up_key, input_dev->keybit);
			set_bit(pdata->rotary0_down_key, input_dev->keybit);
		} else
			set_bit(pdata->rotary0_rel_code, input_dev->relbit);
	}

	if (pdata->enable_rotary1) {
		if (pdata->rotary1_up_key && pdata->rotary1_down_key) {
			set_bit(pdata->rotary1_up_key, input_dev->keybit);
			set_bit(pdata->rotary1_down_key, input_dev->keybit);
		} else
			set_bit(pdata->rotary1_rel_code, input_dev->relbit);
	}

#ifdef CONFIG_CPU_PXA930
	if (pdata->enhanced_rotary_up_key && pdata->enhanced_rotary_down_key) {
		set_bit(pdata->enhanced_rotary_up_key, input_dev->keybit);
		set_bit(pdata->enhanced_rotary_down_key, input_dev->keybit);
	} else
		set_bit(pdata->enhanced_rotary_rel_code, input_dev->relbit);
#endif
}

static inline unsigned int lookup_matrix_keycode(
		struct pxa27x_keypad *keypad, int row, int col)
{
	return keypad->matrix_keycodes[(row << 3) + col];
}


extern int is_android(void);
static void pxa27x_keypad_scan_matrix(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	int row, col, num_keys_pressed = 0;
	uint32_t new_state[MAX_MATRIX_KEY_COLS];
	uint32_t kpas = keypad_readl(KPAS);

	num_keys_pressed = KPAS_MUKP(kpas);

	memset(new_state, 0, sizeof(new_state));

	if (num_keys_pressed == 0)
		goto scan;

	if (num_keys_pressed == 1) {
		col = KPAS_CP(kpas);
		row = KPAS_RP(kpas);

		/* if invalid row/col, treat as no key pressed */
		if (col >= pdata->matrix_key_cols ||
		    row >= pdata->matrix_key_rows)
			goto scan;

		new_state[col] = (1 << row);
		goto scan;
	}

	if (num_keys_pressed > 1) {
		uint32_t kpasmkp0 = keypad_readl(KPASMKP0);
		uint32_t kpasmkp1 = keypad_readl(KPASMKP1);
		uint32_t kpasmkp2 = keypad_readl(KPASMKP2);
		uint32_t kpasmkp3 = keypad_readl(KPASMKP3);

		new_state[0] = kpasmkp0 & KPASMKP_MKC_MASK;
		new_state[1] = (kpasmkp0 >> 16) & KPASMKP_MKC_MASK;
		new_state[2] = kpasmkp1 & KPASMKP_MKC_MASK;
		new_state[3] = (kpasmkp1 >> 16) & KPASMKP_MKC_MASK;
		new_state[4] = kpasmkp2 & KPASMKP_MKC_MASK;
		new_state[5] = (kpasmkp2 >> 16) & KPASMKP_MKC_MASK;
		new_state[6] = kpasmkp3 & KPASMKP_MKC_MASK;
		new_state[7] = (kpasmkp3 >> 16) & KPASMKP_MKC_MASK;
	}
scan:
	for (col = 0; col < pdata->matrix_key_cols; col++) {
		uint32_t bits_changed;

		bits_changed = keypad->matrix_key_state[col] ^ new_state[col];
		if (bits_changed == 0)
			continue;

		for (row = 0; row < pdata->matrix_key_rows; row++) {
			if ((bits_changed & (1 << row)) == 0)
				continue;

			printk(KERN_DEBUG"key %d %d pressed\n", row, col);
			input_report_key(keypad->input_dev,
				lookup_matrix_keycode(keypad, row, col),
				new_state[col] & (1 << row));			
		}
	}
	
	if (!is_android())
		input_sync(keypad->input_dev);
	memcpy(keypad->matrix_key_state, new_state, sizeof(new_state));
}

#define DEFAULT_KPREC	(0x007f007f)

static inline int rotary_delta(uint32_t kprec)
{
	if (kprec & KPREC_OF0)
		return (kprec & 0xff) + 0x7f;
	else if (kprec & KPREC_UF0)
		return (kprec & 0xff) - 0x7f - 0xff;
	else
		return (kprec & 0xff) - 0x7f;
}

#ifdef	CONFIG_CPU_PXA930
/*Read enhanced rotary key*/
static int read_enh_rot_key(struct pxa27x_keypad *keypad, int *key)
{
	u32 curr_ercr = ercr_readl(ERCR_OFF) & 0xf;

	/*assume that increases 10 at a time is impossible,
	otherwise overflow/underflow happens.*/
	if (0x0A < abs(curr_ercr - keypad->prev_ercr))
		if (curr_ercr > keypad->prev_ercr)	/*Underflow happens*/
			*key = keypad->enhanced_rotary_up_key;
		else					/*Overflow happens*/
			*key = keypad->enhanced_rotary_down_key;

	else				/*Normal increament or decreament*/
		*key = curr_ercr > keypad->prev_ercr ?
			keypad->enhanced_rotary_down_key:
			keypad->enhanced_rotary_up_key;

	keypad->prev_ercr = curr_ercr;

	return 0;
}

static void clear_sbcr(struct pxa27x_keypad *keypad)
{
	ercr_writel(SBCR_OFF, ercr_readl(SBCR_OFF) | (1 << 5));
	ercr_writel(SBCR_OFF, ercr_readl(SBCR_OFF) & ~(1 << 5));
}

static irqreturn_t enhanced_rotary_interrupt(int irq, void *dev_id)
{
	struct pxa27x_keypad *keypad = dev_id;
	int key;

	read_enh_rot_key(keypad, &key);
	clear_sbcr(keypad);

	if ((key == keypad->enhanced_rotary_up_key) ||
			(key == keypad->enhanced_rotary_down_key)) {
		input_report_key(keypad->input_dev, key, 1);
		input_report_key(keypad->input_dev, key, 0);
		return IRQ_HANDLED;
	}

	input_sync(keypad->input_dev);

	return IRQ_HANDLED;
}
#endif


static void report_rotary_event(struct pxa27x_keypad *keypad, int r, int delta)
{
	struct input_dev *dev = keypad->input_dev;

	if (delta == 0)
		return;

	if (keypad->rotary_up_key[r] && keypad->rotary_down_key[r]) {
		int keycode = (delta > 0) ? keypad->rotary_up_key[r] :
					    keypad->rotary_down_key[r];

		/* simulate a press-n-release */
		input_report_key(dev, keycode, 1);
		input_sync(dev);
		input_report_key(dev, keycode, 0);
		input_sync(dev);
	} else {
		input_report_rel(dev, keypad->rotary_rel_code[r], delta);
		input_sync(dev);
	}
}

static void pxa27x_keypad_scan_rotary(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	uint32_t kprec;

	/* read and reset to default count value */
	kprec = keypad_readl(KPREC);
	keypad_writel(KPREC, DEFAULT_KPREC);

	if (pdata->enable_rotary0)
		report_rotary_event(keypad, 0, rotary_delta(kprec));

	if (pdata->enable_rotary1)
		report_rotary_event(keypad, 1, rotary_delta(kprec >> 16));
}

static void pxa27x_keypad_report_enter_work(struct work_struct *work)
{
	struct pxa27x_keypad *data = container_of(work, struct pxa27x_keypad, report_enter_work);

	if (DETECT_FOCUS_UP==data->detect_focus_stage) {
		input_report_key(data->input_dev, KEY_ENTER, 1);
		msleep(1);
		input_report_key(data->input_dev, KEY_ENTER, 0);
	}
	data->detect_focus_stage = DETECT_FOCUS_NONE;
}

static void pxa27x_keypad_scan_direct(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	unsigned int new_state;
	uint32_t kpdk, bits_changed;
	int i;
	int direct_focus_to_enter = keypad->direct_focus_to_enter && (!ci_active());
	int report_key = 1;

	kpdk = keypad_readl(KPDK);

	if (pdata->enable_rotary0 || pdata->enable_rotary1)
		pxa27x_keypad_scan_rotary(keypad);

	if (pdata->direct_key_map == NULL)
		return;

	new_state = KPDK_DK(kpdk) & keypad->direct_key_mask;
	bits_changed = keypad->direct_key_state ^ new_state;

	if (bits_changed == 0)
		return;

	printk(KERN_DEBUG"bits_changed 0x%x 0x%x %d\n", bits_changed, new_state, direct_focus_to_enter);
	if (direct_focus_to_enter) {
		if (bits_changed && !(bits_changed&0x1)) {/* focus key*/
			if (new_state)
				keypad->detect_focus_stage = DETECT_FOCUS_DOWN;
			else if (DETECT_FOCUS_DOWN==keypad->detect_focus_stage) {
				/* not deep press, report a enter key */
				keypad->detect_focus_stage = DETECT_FOCUS_UP;
				schedule_work(&keypad->report_enter_work);
			}
			report_key = 0;
		} else
			keypad->detect_focus_stage = DETECT_FOCUS_NONE;
	}

	if (report_key) {
		for (i = 0; i < pdata->direct_key_num; i++) {
			if (bits_changed & (1 << i))
				input_report_key(keypad->input_dev,
						pdata->direct_key_map[i],
						(new_state & (1 << i)));
		}
		input_sync(keypad->input_dev);
	}

	keypad->direct_key_state = new_state;
}

static irqreturn_t pxa27x_keypad_irq_handler(int irq, void *dev_id)
{
	struct pxa27x_keypad *keypad = dev_id;
	unsigned long kpc = keypad_readl(KPC);

	if (kpc & KPC_DI)
		pxa27x_keypad_scan_direct(keypad);

	if (kpc & KPC_MI)
		pxa27x_keypad_scan_matrix(keypad);

	return IRQ_HANDLED;
}

static void pxa27x_keypad_config(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	unsigned int mask = 0, direct_key_num = 0;
	unsigned long kpc = 0;
	unsigned long debounce = 0;

	/* enable matrix keys with automatic scan */
	if (pdata->matrix_key_rows && pdata->matrix_key_cols) {
		kpc |= KPC_ASACT | KPC_MIE | KPC_ME | KPC_MS_ALL;
		kpc |= KPC_MKRN(pdata->matrix_key_rows) |
		       KPC_MKCN(pdata->matrix_key_cols);
	}

	/* enable rotary key, debounce interval same as direct keys */
	if (pdata->enable_rotary0) {
		mask |= 0x03;
		direct_key_num = 2;
		kpc |= KPC_REE0;
	}

	if (pdata->enable_rotary1) {
		mask |= 0x0c;
		direct_key_num = 4;
		kpc |= KPC_REE1;
	}

	if (pdata->direct_key_num > direct_key_num)
		direct_key_num = pdata->direct_key_num;

	keypad->direct_key_mask = ((2 << direct_key_num) - 1) & ~mask;

	/* enable direct key */
	if (direct_key_num)
		kpc |= KPC_DE | KPC_DIE | KPC_DKN(direct_key_num);

	debounce = ((pdata->debounce_interval & 0xff) << 8) |
		    (pdata->debounce_interval & 0xff);

	keypad_writel(KPC, kpc | KPC_RE_ZERO_DEB);
	keypad_writel(KPREC, DEFAULT_KPREC);
	keypad_writel(KPKDI, debounce);

#ifdef CONFIG_CPU_PXA930
	if ((cpu_is_pxa930() || cpu_is_pxa935()) &&  \
			pdata->enable_enhanced_rotary)
		clear_sbcr(keypad);
#endif
}

static int pxa27x_keypad_open(struct input_dev *dev)
{
	struct pxa27x_keypad *keypad = input_get_drvdata(dev);

	/* Enable unit clock */
	clk_enable(keypad->clk);
	pxa27x_keypad_config(keypad);

	return 0;
}

static void pxa27x_keypad_close(struct input_dev *dev)
{
	struct pxa27x_keypad *keypad = input_get_drvdata(dev);

	/* Disable clock unit */
	clk_disable(keypad->clk);
}

#if defined(CONFIG_PXA3xx_DVFM)
static void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count++ == 0) {
		/* Disable lowpower mode */
		dvfm_disable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_disable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
			dvfm_disable_op_name("CG", dvfm_lock.dev_idx);
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void unset_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count == 0) {
		printk(KERN_WARNING "Keypad constraint has been removed.\n");
	} else if (--dvfm_lock.count == 0) {
		/* Enable lowpower mode */
		dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
			dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

/*
 * FIXME: Here a timer is used to disable entering D1/D2 for a while.
 * Because keypad event wakeup system from D1/D2 mode. But keypad device
 * can't detect the interrupt since it's in standby state.
 * Keypad device need time to detect it again. So we use a timer here.
 * D1/D2 idle is determined by idle time. It's better to comine these
 * timers together.
 */
static void keypad_timer_handler(unsigned long data)
{
	unset_dvfm_constraint();
}

extern void get_wakeup_source(pm_wakeup_src_t *);

static int keypad_notifier_freq(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct dvfm_freqs *freqs = (struct dvfm_freqs *)data;
	struct op_info *new = NULL;
	struct dvfm_md_opt *op;
	pm_wakeup_src_t src;

	if (freqs)
		new = &freqs->new_info;
	else
		return 0;

	op = (struct dvfm_md_opt *)new->op;
	if (val == DVFM_FREQ_POSTCHANGE) {
		if ((op->power_mode == POWER_MODE_D1) ||
				(op->power_mode == POWER_MODE_D2) ||
				(op->power_mode == POWER_MODE_CG)) {
			get_wakeup_source(&src);
			if (src.bits.mkey || src.bits.dkey) {
				/* If keypad event happens and wake system
				 * from D1/D2. Disable D1/D2 to make keypad
				 * work for a while.
				 */
				kp_timer.expires = jiffies + D2_STABLE_JIFFIES;
				add_timer(&kp_timer);
				set_dvfm_constraint();
			}
		}
	}
	return 0;
}
#endif

static irqreturn_t slide_interrupt(int irq, void *dev_id) 
{
	struct pxa27x_keypad *keypad = (struct pxa27x_keypad *)dev_id;
	int slideint_mfp = keypad->slideint_mfp;
	int slide_state = !gpio_get_value(slideint_mfp);
	
	if (keypad->slide_state!=slide_state) {
		printk(KERN_DEBUG "slide_interrupt %d\n", slide_state);
		if (-100==irq && slide_state) { /* magic for resume, fake a wake up event here */
			input_report_key(keypad->input_dev, KEY_BACK, 1);
			input_report_key(keypad->input_dev, KEY_BACK, 0);
		}
		input_report_switch(keypad->input_dev, SW_LID, slide_state);
		keypad->slide_state = slide_state;
	}
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int pxa27x_keypad_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa27x_keypad *keypad = platform_get_drvdata(pdev);

	clk_disable(keypad->clk);

	return 0;
}

static int pxa27x_keypad_resume(struct platform_device *pdev)
{
	struct pxa27x_keypad *keypad = platform_get_drvdata(pdev);
	struct input_dev *input_dev = keypad->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		/* Enable unit clock */
		clk_enable(keypad->clk);
		pxa27x_keypad_config(keypad);
	}

	mutex_unlock(&input_dev->mutex);

	if (keypad->slideint_mfp >= 0)
		slide_interrupt(-100, keypad);

	return 0;
}
#else
#define pxa27x_keypad_suspend	NULL
#define pxa27x_keypad_resume	NULL
#endif

#define res_size(res)	((res)->end - (res)->start + 1)
#ifdef CONFIG_INPUT_ANDROID
struct input_dev* pxa_keypad_input_dev;
static ssize_t key_emulator_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pxa27x_keypad *p_keypad = (struct pxa27x_keypad *)dev_get_drvdata(dev);	
	int key;	

	sscanf(buf, "%d\n", &key);
	if (key>KEY_RESERVED && key<KEY_MAX) {
		input_report_key(p_keypad->input_dev, key, 1);
		msleep(50);
		input_report_key(p_keypad->input_dev, key, 0);
	}
	
	return count;
}
static DEVICE_ATTR(key_emulator,0644,NULL,key_emulator_store);
#endif

static struct attribute *pxa_keypad_attributes[] = {
#ifdef CONFIG_INPUT_ANDROID
	&dev_attr_key_emulator.attr,
#endif
	NULL,
};
static struct attribute_group pxa_keypad_attr_group ={
	.attrs=pxa_keypad_attributes,
};

static int __devinit pxa27x_keypad_probe(struct platform_device *pdev)
{
	struct pxa27x_keypad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int irq, error;
	int irq2;

	keypad = kzalloc(sizeof(struct pxa27x_keypad), GFP_KERNEL);
	if (keypad == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	keypad->pdata = pdev->dev.platform_data;
	if (keypad->pdata == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		error = -EINVAL;
		goto failed_free;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get keypad irq\n");
		error = -ENXIO;
		goto failed_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto failed_free;
	}

	res = request_mem_region(res->start, res_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto failed_free;
	}

	keypad->mmio_base = ioremap(res->start, res_size(res));
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -ENXIO;
		goto failed_free_mem;
	}

	keypad->clk = clk_get(&pdev->dev, "KBDCLK");
	if (IS_ERR(keypad->clk)) {
		dev_err(&pdev->dev, "failed to get keypad clock\n");
		error = PTR_ERR(keypad->clk);
		goto failed_free_io;
	}

	/* Create and register the input driver. */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto failed_put_clk;
	}

	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_HOST;
	input_dev->open = pxa27x_keypad_open;
	input_dev->close = pxa27x_keypad_close;
	input_dev->dev.parent = &pdev->dev;

	keypad->input_dev = input_dev;
	input_set_drvdata(input_dev, keypad);

#if defined(CONFIG_INPUT_ANDROID) /* donot enable EV_REP */
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);	
	pxa_keypad_input_dev = keypad->input_dev;
#else
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) |
		BIT_MASK(EV_REL);
#endif

	set_bit(KEY_END, keypad->input_dev->keybit); /* onkey */
	set_bit(KEY_MEDIA, keypad->input_dev->keybit); /* hook key */		
	
	keypad->slideint_mfp = keypad->pdata->slideint_mfp;
	keypad->slide_state = -1;
	if (keypad->slideint_mfp >= 0) {
		int slideint_mfp = keypad->slideint_mfp;
		int ret;

		set_bit(EV_SW, keypad->input_dev->evbit);
		set_bit(SW_LID, keypad->input_dev->swbit);
		
		pxa3xx_mfp_set_afds(slideint_mfp, MFP_AF0, MFP_DS04X);
		gpio_direction_input(slideint_mfp);
		pxa3xx_mfp_set_lpm(slideint_mfp, MFP_LPM_PULL_HIGH);
		
		ret = request_irq(IRQ_GPIO(MFP2GPIO(slideint_mfp)), slide_interrupt, 
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "SLIDE_INT", keypad);
		printk(KERN_DEBUG "Request slide_interrupt %d, ret %d\n", slideint_mfp, ret);
	}

	pxa27x_keypad_build_keycode(keypad);
	platform_set_drvdata(pdev, keypad);

	INIT_WORK(&keypad->report_enter_work, pxa27x_keypad_report_enter_work);
	if (test_bit(KEY_FOCUS, keypad->input_dev->keybit) 
			&& !test_bit(KEY_ENTER, keypad->input_dev->keybit)) {
		printk("Keypad: enable direct focus to enter\n");
		keypad->direct_focus_to_enter = 1;
		set_bit(KEY_ENTER, keypad->input_dev->keybit); /* hook key */
	} else
		keypad->direct_focus_to_enter = 0;

	error = request_irq(irq, pxa27x_keypad_irq_handler, IRQF_DISABLED,
			    pdev->name, keypad);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto failed_free_dev;
	}

#if defined(CONFIG_PXA3xx_DVFM)
	dvfm_register("Keypad", &dvfm_lock.dev_idx);
	dvfm_register_notifier(&notifier_freq_block,
			DVFM_FREQUENCY_NOTIFIER);
	init_timer(&kp_timer);
	kp_timer.function = keypad_timer_handler;
	kp_timer.data = 0;
#endif

#ifdef CONFIG_CPU_PXA930
	/* Enhanced Rotary Controller */
	irq2 = -1;
	if ((cpu_is_pxa930() || cpu_is_pxa935()) &&  \
			keypad->pdata->enable_enhanced_rotary) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		keypad->ercr_base = ioremap_nocache(res->start, res_size(res));
		if (keypad->ercr_base == NULL) {
			dev_err(&pdev->dev, "failed to ioremap registers\n");
			error = -ENXIO;
			goto failed_free_irq;
		}
		irq2 = platform_get_irq(pdev, 1);
		if (irq2 < 0) {
			dev_err(&pdev->dev, "failed to get"
				" enhanced rotary controller irq\n");
			error = -ENXIO;
			goto failed_free_irq;
		}
		error = request_irq(irq2, enhanced_rotary_interrupt,
			    IRQF_DISABLED, "Enhanced Rotary", keypad);
		if (error) {
			dev_err(&pdev->dev, "failed to request Enhanced Rotary Controller IRQ\n");
			goto failed_free_irq2;
		}
	}
#endif

	error = sysfs_create_group(&pdev->dev.kobj, &pxa_keypad_attr_group);
 
	keypad->irq = irq;

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_irq2;
	}

	if (keypad->slideint_mfp >= 0)
		slide_interrupt(0, keypad);
	
	device_init_wakeup(&pdev->dev, 1);

	return 0;

failed_free_irq2:
	free_irq(irq2, pdev);
failed_free_irq:
	free_irq(irq, pdev);
	platform_set_drvdata(pdev, NULL);
failed_free_dev:
	input_free_device(input_dev);
failed_put_clk:
	clk_put(keypad->clk);
failed_free_io:
	iounmap(keypad->mmio_base);
failed_free_mem:
	release_mem_region(res->start, res_size(res));
failed_free:
	kfree(keypad);
	return error;
}

static int __devexit pxa27x_keypad_remove(struct platform_device *pdev)
{
	struct pxa27x_keypad *keypad = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(keypad->irq, pdev);

	clk_disable(keypad->clk);
	clk_put(keypad->clk);

	input_unregister_device(keypad->input_dev);
	input_free_device(keypad->input_dev);

	iounmap(keypad->mmio_base);

#if defined(CONFIG_PXA3xx_DVFM)
	dvfm_unregister_notifier(&notifier_freq_block,
			DVFM_FREQUENCY_NOTIFIER);
	dvfm_unregister("Keypad", &dvfm_lock.dev_idx);
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res_size(res));

	platform_set_drvdata(pdev, NULL);
	kfree(keypad);
	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:pxa27x-keypad");

static struct platform_driver pxa27x_keypad_driver = {
	.probe		= pxa27x_keypad_probe,
	.remove		= __devexit_p(pxa27x_keypad_remove),
	.suspend	= pxa27x_keypad_suspend,
	.resume		= pxa27x_keypad_resume,
	.driver		= {
		.name	= "pxa27x-keypad",
		.owner	= THIS_MODULE,
	},
};

static int __init pxa27x_keypad_init(void)
{
	return platform_driver_register(&pxa27x_keypad_driver);
}

static void __exit pxa27x_keypad_exit(void)
{
	platform_driver_unregister(&pxa27x_keypad_driver);
}

late_initcall(pxa27x_keypad_init);
module_exit(pxa27x_keypad_exit);

MODULE_DESCRIPTION("PXA27x Keypad Controller Driver");
MODULE_LICENSE("GPL");
