/*
 * Monahans PMIC abstrction layer
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <mach/pxa3xx_pmic.h>

static struct pmic_ops *pxa3xx_pmic_ops;

#ifdef	DEBUG
/* calculate the elapsed time on operating PMIC */
static unsigned int start_time, end_time;
void start_calc_time(void)
{
	start_time = OSCR;
}

void end_calc_time(void)
{
	unsigned int time;
	end_time = OSCR
	time = (end_time - start_time) * 100 / 325;

	pr_debug("\n%s:\t:%dus\n", __func__, time);
}
#else
void start_calc_time(void) {}
void end_calc_time(void) {}
#endif

void pmic_set_ops(struct pmic_ops *ops)
{
	if (pxa3xx_pmic_ops != NULL) {
		printk(KERN_ERR "set pmic_ops when pmic_ops is not NULL\n");
		return;
	}
	pxa3xx_pmic_ops = ops;
	INIT_LIST_HEAD(&pxa3xx_pmic_ops->list);
	spin_lock_init(&pxa3xx_pmic_ops->cb_lock);
}

/*****************************************************************************
 *			Operation of PMIC				     *
 *****************************************************************************/
int check_pmic_ops(void)
{
	if (!pxa3xx_pmic_ops) {
		printk(KERN_WARNING "No pmic_ops registered!\n");
		return -EINVAL;
	} else
		return 0;
}

int pxa3xx_pmic_get_voltage(int cmd, int *pval)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (pxa3xx_pmic_ops->get_voltage)
		return pxa3xx_pmic_ops->get_voltage(cmd, pval);
	else
		return -EINVAL;
}
EXPORT_SYMBOL(pxa3xx_pmic_get_voltage);

int pxa3xx_pmic_set_voltage(int cmd, int val)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (pxa3xx_pmic_ops->set_voltage)
		return pxa3xx_pmic_ops->set_voltage(cmd, val);
	else
		return -EINVAL;
}
EXPORT_SYMBOL(pxa3xx_pmic_set_voltage);

int pxa3xx_pmic_is_vbus_assert(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0) /* If illegal pmic_ops, always no vbus activity */
		return 0;

	if (pxa3xx_pmic_ops->is_vbus_assert)
		return pxa3xx_pmic_ops->is_vbus_assert();

	return 0;
}
EXPORT_SYMBOL(pxa3xx_pmic_is_vbus_assert);

int pxa3xx_pmic_is_avbusvld(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0) /* If illegal pmic_ops, always no A vbus valid */
		return 0;

	if (pxa3xx_pmic_ops->is_avbusvld)
		return pxa3xx_pmic_ops->is_avbusvld();

	return 0;
}
EXPORT_SYMBOL(pxa3xx_pmic_is_avbusvld);

int pxa3xx_pmic_is_asessvld(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0) /* If illegal pmic_ops, always no A assert valid */
		return 0;

	if (pxa3xx_pmic_ops->is_asessvld)
		return pxa3xx_pmic_ops->is_asessvld();

	return 0;
}
EXPORT_SYMBOL(pxa3xx_pmic_is_asessvld);

int pxa3xx_pmic_is_bsessvld(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0) /* If illegal pmic_ops, always no B assert valid */
		return 0;

	if (pxa3xx_pmic_ops->is_bsessvld)
		return pxa3xx_pmic_ops->is_bsessvld();

	return 0;
}
EXPORT_SYMBOL(pxa3xx_pmic_is_bsessvld);

int pxa3xx_pmic_is_srp_ready(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0) /* If illegal pmic_ops, always no SRP detect */
		return 0;

	if (pxa3xx_pmic_ops->is_srp_ready)
		return pxa3xx_pmic_ops->is_srp_ready();

	return 0;

}
EXPORT_SYMBOL(pxa3xx_pmic_is_srp_ready);

int pxa3xx_pmic_set_pump(int enable)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (pxa3xx_pmic_ops->set_pump)
		return pxa3xx_pmic_ops->set_pump(enable);

	return 0;
}
EXPORT_SYMBOL(pxa3xx_pmic_set_pump);

int pxa3xx_pmic_set_vbus_supply(int enable, int srp)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (pxa3xx_pmic_ops->set_vbus_supply)
		return pxa3xx_pmic_ops->set_vbus_supply(enable, srp);

	return 0;
}
EXPORT_SYMBOL(pxa3xx_pmic_set_vbus_supply);

int pxa3xx_pmic_set_usbotg_a_mask(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (pxa3xx_pmic_ops->set_usbotg_a_mask)
		return pxa3xx_pmic_ops->set_usbotg_a_mask();

	return 0;
}
EXPORT_SYMBOL(pxa3xx_pmic_set_usbotg_a_mask);

int pxa3xx_pmic_set_usbotg_b_mask(void)
{
	int ret;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	if (pxa3xx_pmic_ops->set_usbotg_b_mask)
		return pxa3xx_pmic_ops->set_usbotg_b_mask();

	return 0;
}
EXPORT_SYMBOL(pxa3xx_pmic_set_usbotg_b_mask);

/* Register pmic callback */
int pmic_callback_register(unsigned long event,
	       void (*func)(unsigned long event))
{
	int ret;
	unsigned long flags;
	struct pmic_callback *pmic_cb;

	might_sleep();

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	pmic_cb = kzalloc(sizeof(*pmic_cb), GFP_KERNEL);
	if (!pmic_cb)
		return -ENOMEM;

	INIT_LIST_HEAD(&pmic_cb->list);
	pmic_cb->event = event;
	pmic_cb->func = func;

	spin_lock_irqsave(&pxa3xx_pmic_ops->cb_lock, flags);
	list_add(&pmic_cb->list, &pxa3xx_pmic_ops->list);
	spin_unlock_irqrestore(&pxa3xx_pmic_ops->cb_lock, flags);

	return 0;
}
EXPORT_SYMBOL(pmic_callback_register);

/* Unregister pmic callback */
int pmic_callback_unregister(unsigned long event,
		void (*func)(unsigned long event))
{
	unsigned long flags;
	struct pmic_callback *pmic_cb, *next;

	spin_lock_irqsave(&pxa3xx_pmic_ops->cb_lock, flags);
	list_for_each_entry_safe(pmic_cb, next, &pxa3xx_pmic_ops->list, list) {
		if ((pmic_cb->event == event) && (pmic_cb->func == func)) {
			list_del_init(&pmic_cb->list);
			kfree(pmic_cb);
		}
	}
	spin_unlock_irqrestore(&pxa3xx_pmic_ops->cb_lock, flags);
	return 0;
}
EXPORT_SYMBOL(pmic_callback_unregister);

int pmic_event_handle(unsigned long event)
{
	int ret;
	unsigned long flags;
	struct pmic_callback *pmic_cb;

	ret = check_pmic_ops();
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pxa3xx_pmic_ops->cb_lock, flags);
	list_for_each_entry(pmic_cb, &pxa3xx_pmic_ops->list, list) {
		spin_unlock_irqrestore(&pxa3xx_pmic_ops->cb_lock, flags);
		/* event is bit-wise parameter, need bit AND here as filter */
		if ((pmic_cb->event & event) && (pmic_cb->func))
			pmic_cb->func(event);
		spin_lock_irqsave(&pxa3xx_pmic_ops->cb_lock, flags);
	}
	spin_unlock_irqrestore(&pxa3xx_pmic_ops->cb_lock, flags);
	return 0;
}
EXPORT_SYMBOL(pmic_event_handle);

