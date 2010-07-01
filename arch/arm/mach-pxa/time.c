/*
 * arch/arm/mach-pxa/time.c
 *
 * PXA clocksource, clockevents, and OST interrupt handlers.
 * Copyright (c) 2007 by Bill Gatliff <bgat@billgatliff.com>.
 *
 * Derived from Nicolas Pitre's PXA timer handler Copyright (c) 2001
 * by MontaVista Software, Inc.  (Nico, your code rocks!)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/sched.h>
#include <linux/cnt32_to_63.h>

#include <asm/div64.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/pxa-regs.h>
#include <asm/mach-types.h>

#define OST_C0		(1 << 0)
#define OST_C1		(1 << 1)
#define OST_C2		(1 << 2)
#define OST_C3		(1 << 3)
#define OST_C4		(1 << 4)
#define OST_C10		(1 << 10)
#define OST_C11		(1 << 11)

/*
 * This is PXA's sched_clock implementation. This has a resolution
 * of at least 308 ns and a maximum value of 208 days.
 *
 * The return value is guaranteed to be monotonic in that range as
 * long as there is always less than 582 seconds between successive
 * calls to sched_clock() which should always be the case in practice.
 */

#define OSCR2NS_SCALE_FACTOR 10

static unsigned long oscr2ns_scale;

static int timer32k_enabled = 0;

static void __init set_oscr2ns_scale(unsigned long oscr_rate)
{
	unsigned long long v = 1000000000ULL << OSCR2NS_SCALE_FACTOR;
	do_div(v, oscr_rate);
	oscr2ns_scale = v;
	/*
	 * We want an even value to automatically clear the top bit
	 * returned by cnt32_to_63() without an additional run time
	 * instruction. So if the LSB is 1 then round it up.
	 */
	if (oscr2ns_scale & 1)
		oscr2ns_scale++;
}

static cycle_t pxa_timer_read(void)
{
	if (timer32k_enabled)
		return (cycle_t)OSCR4;
	return (cycle_t)OSCR;
}

static unsigned long long pxa_32k_ticks_to_nsec(cycle_t cycles)
{
	return (unsigned long long)cycles * 1000 *5*5*5*5*5*5 >> 9;
}


unsigned long long sched_clock(void)
{
	unsigned long long v;
	if (timer32k_enabled) {
		return pxa_32k_ticks_to_nsec(pxa_timer_read());
	}
	v = cnt32_to_63(OSCR);
	return (v * oscr2ns_scale) >> OSCR2NS_SCALE_FACTOR;
}

/* Use OST channel 11 and 10 to calibrate RTC:
 * RTC will use 32K clock. 32K clock is provided by VCTCXO, which 
 * will become inaccurate with age. In order to ensure RTC accurate,
 * 1MHZ clock(fast clock) is connected to OST channel 10 and 32K clock 
 * (slow clock) is connected to the OST channel 11. OST channel 10 
 * and channel 11 will be started at same time. When OST channel 11 
 * stops at 1s match event occurs, the read the counts of channel 10 
 * and channel 11 at same time. By comparing the count of channel 11 
 * with its theoretic value, the deviation of 32K clock can be calculated, 
 * so the RTC can be calibrated by adjusting its divisor.
 */
#define FAST_TIMER_1SEC_VALUE	1000000
#define RTTR_DEFAULT_DIV_VALUE	0x00007FFF
#define FAST_TIMER_START_VALUE  1000000
#define ONE_SEC_TIME_INTERVAL	32769

static struct completion rtc_calib_complete;
int rtc_calib(void)
{
	int ret;

	/* disable OS timers IRQ (10 & 11) */
	OIER &= ~OST_C10;
	OIER &= ~OST_C11;

	/* run 32 Khz timer */
	OMCR11 = (1 << 7) | (1 << 0) | (1 << 9);

	/* run 1 Mhz timer */
	OMCR10 = (1 << 7) | (4 << 0);	  

	/* configure 32Khz timer */
	OSCR11 = 1;
	OSMR11 = ONE_SEC_TIME_INTERVAL;

	/* configure OS timer to 1 Mhz with 10 sec interval */
	OSCR10 = FAST_TIMER_START_VALUE;	

	/* Enable OSCR11 match interrupt for slow timer */
	OIER |= OST_C11;							  

	/* wait for RTC calibration done */
	ret = wait_for_completion_timeout(&rtc_calib_complete, 2*HZ);

	if (!ret) {
		INIT_COMPLETION(rtc_calib_complete);
		printk(KERN_ERR "%s: rtc calibration failed\n", __func__);
		return -EIO;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(rtc_calib);

static void set_rtc_rttr(int count)
{
	int divisor = 0;

	/* clear the lock bit */
	RTTR |= 0x7FFFFFFF; 

	/* calculate delta time */
	divisor = FAST_TIMER_1SEC_VALUE - count; 
	pr_debug("- %s:delta count:%d\n", __func__, divisor);

	/* convert fast clock ticks to slow ticks */
	divisor = divisor*32*1024/count; 
	pr_debug("- %s:delta divisor:%d\n", __func__, divisor);

	/* update CLK_DIV */
	divisor = RTTR_DEFAULT_DIV_VALUE + divisor;

	RTTR &= ~0x0000FFFF;
	RTTR |= divisor;
}

#define MIN_OSCR_DELTA 16
#define MIN_OSCR4_DELTA 16

static irqreturn_t
pxa_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;
	unsigned int oscr11;

	if (!(OSSR & OST_C0) && !(OSSR & OST_C4) && !(OSSR & OST_C11))
		return IRQ_NONE;

	/* Disarm the compare/match, signal the event. */
	if (OSSR & OST_C0) {
		OIER &= ~OST_C0;
		OSSR = OST_C0;
		if (!timer32k_enabled)
			c->event_handler(c);
	}

	if (OSSR & OST_C4) {
		OIER &= ~OST_C4;
		OSSR = OST_C4;
		if (timer32k_enabled)
			c->event_handler(c);
	}

	if (OSSR & OST_C11) {
		/* clear interrupt status */
		OSSR = OST_C11;
		
		/* OSC11 will be read out and OSCR10 will read into OSNR simutaneously */
		oscr11 = OSCR11; 

		/* Disable OST channel 10 (fast clock) */
		OIER &= ~OST_C10;
		OMCR10 = 0;

		/* stop channel 11 (slow clock) */
		OIER &= ~OST_C11;
		OMCR11 = 0;
		
		/* calibrate the RTC */
		set_rtc_rttr(OSNR - FAST_TIMER_START_VALUE);

		/* notify rtc calibration is done */
		complete(&rtc_calib_complete);
	}

	return IRQ_HANDLED;
}

static int
pxa_timer_set_next_event(unsigned long delta, struct clock_event_device *dev)
{
	unsigned long flags, next, oscr;

	if (timer32k_enabled) {
		raw_local_irq_save(flags);
		OIER |= OST_C4;
		next = OSCR4 + delta;
		OSMR4 = next;
		oscr = OSCR4;
		raw_local_irq_restore(flags);
		return (signed)(next - oscr) <= MIN_OSCR4_DELTA ? -ETIME : 0;
	} else {
		raw_local_irq_save(flags);
		OIER |= OST_C0;
		next = OSCR + delta;
		OSMR0 = next;
		oscr = OSCR;
		raw_local_irq_restore(flags);
		return (signed)(next - oscr) <= MIN_OSCR_DELTA ? -ETIME : 0;
	}
	return 0;

}

static void
pxa_timer_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	unsigned long irqflags;

	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
		raw_local_irq_save(irqflags);
		if (timer32k_enabled) {
			OIER &= ~OST_C4;
			OSSR = OST_C4;
		} else {
			OIER &= ~OST_C0;
			OSSR = OST_C0;
		}
		raw_local_irq_restore(irqflags);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* initializing, released, or preparing for suspend */
		raw_local_irq_save(irqflags);
		if (timer32k_enabled) {
			OIER &= ~OST_C4;
			OSSR = OST_C4;
		} else {
			OIER &= ~OST_C0;
			OSSR = OST_C0;
		}
		raw_local_irq_restore(irqflags);
		break;

	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

static struct clock_event_device ckevt_pxa_osmr0 = {
	.name		= "osmr0",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.rating		= 200,
	.cpumask	= CPU_MASK_CPU0,
	.set_next_event	= pxa_timer_set_next_event,
	.set_mode	= pxa_timer_set_mode,
};

static struct clock_event_device ckevt_32ktimer = {
	.name		= "32k timer",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 24,
	.rating		= 150,
	.cpumask	= CPU_MASK_CPU0,
	.set_next_event	= pxa_timer_set_next_event,
	.set_mode	= pxa_timer_set_mode,
};
 
static struct clocksource cksrc_pxa_oscr0 = {
	.name           = "oscr0",
	.rating         = 200,
	.read           = pxa_timer_read,
	.mask           = CLOCKSOURCE_MASK(32),
	.shift          = 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct clocksource cksrc_32ktimer = {
	.name		= "oscr4",
	.rating		= 150,
	.read		= pxa_timer_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 9,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction pxa_timer_irq = {
	.name		= "os timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL | IRQF_SHARED,
	.handler	= pxa_timer_interrupt,
};

static void __init pxa_timer_init(void)
{
	unsigned long clock_tick_rate;

	OIER = 0;
	OSSR = OST_C0 | OST_C1 | OST_C2 | OST_C3 | OST_C4 | OST_C10 | OST_C11;

	if (cpu_is_pxa21x() || cpu_is_pxa25x())
		clock_tick_rate = 3686400;
	else if (machine_is_mainstone())
		clock_tick_rate = 3249600;
	else {
#ifdef CONFIG_PXA_32KTIMER
		clock_tick_rate = 32768;
		timer32k_enabled = 1;	/* use 32K timer */
#else
 		clock_tick_rate = 3250000;
		timer32k_enabled = 0;	/* use timer0 */
#endif
	}

	if (timer32k_enabled && cpu_is_pxa3xx()) {
		OMCR4 = 0xC1;		/* 32.768KHz */

		pxa_timer_irq.dev_id = &ckevt_32ktimer;
		setup_irq(IRQ_OST_4_11, &pxa_timer_irq);

		OSMR4 = 0;
		OSCR4 = 1;		/* enable channel 4 interrupt */

		ckevt_32ktimer.mult =
			div_sc(clock_tick_rate, NSEC_PER_SEC,
			ckevt_32ktimer.shift);
		ckevt_32ktimer.max_delta_ns =
			clockevent_delta2ns(LLONG_MAX, &ckevt_32ktimer);
		ckevt_32ktimer.min_delta_ns =
			clockevent_delta2ns(2 * MIN_OSCR4_DELTA, &ckevt_32ktimer) + 1;

		cksrc_32ktimer.mult =
			clocksource_hz2mult(clock_tick_rate,
					cksrc_32ktimer.shift);

		clocksource_register(&cksrc_32ktimer);
		clockevents_register_device(&ckevt_32ktimer);
	} else if (!timer32k_enabled) {
		set_oscr2ns_scale(clock_tick_rate);

		ckevt_pxa_osmr0.mult =
			div_sc(clock_tick_rate, NSEC_PER_SEC,
			ckevt_pxa_osmr0.shift);
		ckevt_pxa_osmr0.max_delta_ns =
			clockevent_delta2ns(0x7fffffff, &ckevt_pxa_osmr0);
		ckevt_pxa_osmr0.min_delta_ns =
			clockevent_delta2ns(MIN_OSCR_DELTA, &ckevt_pxa_osmr0)
			+ 1;

		cksrc_pxa_oscr0.mult =
			clocksource_hz2mult(clock_tick_rate,
					cksrc_pxa_oscr0.shift);

		pxa_timer_irq.dev_id = &ckevt_pxa_osmr0;
		setup_irq(IRQ_OST0, &pxa_timer_irq);

		clocksource_register(&cksrc_pxa_oscr0);
		clockevents_register_device(&ckevt_pxa_osmr0);
	} else {
		printk(KERN_WARNING "Wrong timer configuration in pxa.\n");
		BUG();
	}

	/* enable interrupt for channel 10 and 11 for RTC calibration
	 * if it is not enabled
	 */
	init_completion(&rtc_calib_complete);
	if (!timer32k_enabled || !cpu_is_pxa3xx()) {
		pxa_timer_irq.dev_id = NULL;
		setup_irq(IRQ_OST_4_11, &pxa_timer_irq);
	}
}

int pxa_dyn_enable;      /* flag of dynamic tick status */

#ifdef CONFIG_PM
static unsigned long osmr[4], oier, oscr;

static void pxa_timer_suspend(void)
{
	osmr[0] = OSMR0;
	osmr[1] = OSMR1;
	osmr[2] = OSMR2;
	osmr[3] = OSMR3;
	oier = OIER;
	oscr = OSCR;
}

static void pxa_timer_resume(void)
{
	/*
	 * Ensure that we have at least MIN_OSCR_DELTA between match
	 * register 0 and the OSCR, to guarantee that we will receive
	 * the one-shot timer interrupt.  We adjust OSMR0 in preference
	 * to OSCR to guarantee that OSCR is monotonically incrementing.
	 */
	if (osmr[0] - oscr < MIN_OSCR_DELTA)
		osmr[0] += MIN_OSCR_DELTA;

	OSMR0 = osmr[0];
	OSMR1 = osmr[1];
	OSMR2 = osmr[2];
	OSMR3 = osmr[3];
	OIER = oier;
	OSCR = oscr;
}
#else
#define pxa_timer_suspend NULL
#define pxa_timer_resume NULL
#endif

struct sys_timer pxa_timer = {
	.init		= pxa_timer_init,
	.suspend	= pxa_timer_suspend,
	.resume		= pxa_timer_resume,
};
