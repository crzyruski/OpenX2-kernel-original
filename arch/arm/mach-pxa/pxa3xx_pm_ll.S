/*
 * Low-level Monahans suspend/resume support
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/pxa3xx_pm.h>

/* pxa3xx_cpu_resume()
 * Entry point for bootloader resume to kernel
 *
 * It will invoke pm_resume_from_sleep which use area_phy_address as parameter
 */
/* Note: The following code is located into the .data section.
 *       This is to allow area_phy_address to be accessed with a relative load
 *       while we can't rely on any MMU translation.  We could have put
 *       area_phy_address in the .text section as well, but some setups might
 *       insist on it to be truly read-only.
 */
	.data
	.align 5
ENTRY(pxa3xx_cpu_resume)
	ldr	r1, =0x80000004
	ldr	r0, [r1]
	bl	pm_resume_from_sleep
	cmp	r0, #1
	@ maybe turn on some lights for warning
error_ret:
	nop
	beq	error_ret

/* pxa3xx_cpu_standby()
 *
 * Entry point for entering standby(S0D2C2).
 */
	.text
	.align 5
ENTRY(pxa3xx_cpu_standby)
/* lr register will be the instruction just after invoke of pxa3xx_cpu_standby */
	b	pm_enter_standby

/* pxa3xx_cpu_lcdrefresh()
 *
 * Entry point for entering lcdrefresh(S0D1C2).
 */
	.text
	.align 5
ENTRY(pxa3xx_cpu_lcdrefresh)
/* lr register will be the instruction just after invoke of pxa3xx_cpu_lcdrefresh */
	b	pm_enter_lcd_refresh

/* pxa3xx_cpu_sleep(unsigned int a, unsigned int b)
 *
 * Entry point for entering sleep mode(S2D3C4).
 * a:
 *	vitual address of the data save area for Monahans content
 * b:
 *	physical address of the data save area for Monahans content
 *
 * The API pm_enter_sleep will use the first parameter "a". The "b" will
 * be stored in area_phy_adress which will be used by pm_resume_from_sleep.
 */

ENTRY(pxa3xx_cpu_sleep)
	b	pm_enter_sleep

/* pxa3xx_cpu_deepsleep(unsigned int a, unsigned int b)
 *
 * Entry point for entering sleep mode(S2D3C4).
 * a:
 *	vitual address of the data save area for Monahans content
 * b:
 *	physical address of the data save area for Monahans content
 *
 * The pm_enter_sleep will use the first parameter "a". The "b" will
 * be stored in area_phy_adress which will be used by pm_resume_from_sleep.
 */

ENTRY(pxa3xx_cpu_deepsleep)
	b	pm_enter_deep_sleep


@*****************************************************************************
@ pm_checksum_calculate
@
@ Calculate checksum
@
@ Inputs:
@	r0: the virutal address of the data area which will be calculated the
@		checksum
@	r1: the toltal word of the data area. Checksum is done on 4-byte word
@
@ Output:
@	checksum
@
@ Registers used
@	r0, r1, r2, r3
@

pm_checksum_calculate:

	@ pick a non-zero seed
	ldr	r2, =(0x5A72)
calculate:
	@ get value and increment pointer
	ldr	r3, [r0], #4
	add	r2, r2, r3
	@ rotate left by one bit position
	mov	r2, r2, ROR #31
	subs	r1, r1, #1
	bne	calculate
	mov	r0, r2
	mov	pc, lr



@******************************************************************************
@
@ pm_resume_from_deep_sleep
@
@ Restore saved content and return back
@
@ Inputs:
@	r0: The physical address of the saved data area
@
@ Outputs:
@	None
@
pm_resume_from_deep_sleep:

@******************************************************************************
@
@ pm_resume_from_sleep
@
@ Restore saved content and return back
@
@ Inputs:
@	r0: The physical address of the saved data area
@
@ Outputs:
@	None
@

pm_resume_from_sleep:
	@ make sure that we are in SVC mode with irq and fiq off
	mov	r1, #(CPSR_Mode_SVC | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r1

	@ Step 1
	@ validate checksum
	@ get the address of the first word that is checksumable
	mov	r9, r0
	ldr	r1, [r9, #SleepState_wordCount]
	mov	r8, lr
	add	r0, r9, #4
	bl	pm_checksum_calculate
	ldr	r3, [r9, #SleepState_checksum]
	subs	r1, r3, r0
	mov	r0, #0
	movne	r0, #1
	@ return if checksum is wrong
	movne	pc, r8
	mov	r0, r9

	@ Step 2
	ldr	r9, [r0, #SleepState_Cp15_ACR_MMU]
	ldr	r8, [r0, #SleepState_Cp15_AUXCR_MMU]
	ldr	r7, [r0, #SleepState_Cp15_TTBR_MMU]
	ldr	r6, [r0, #SleepState_Cp15_DACR_MMU]
	ldr	r5, [r0, #SleepState_Cp15_PID_MMU]
	ldr	r4, [r0, #SleepState_Cp15_CPAR]
	ldr	r0, [r0, #SleepState_areaAddress]

	@ invalidate I, D caches & BTB
	mcr	p15, 0, ip, c7, c7, 0
	@ Drain Write (& Fill) Buffer
	mcr	p15, 0, ip, c7, c10, 4
	@ Prefetch Flush
	mcr	p15, 0, ip, c7, c5, 4
	@ invalidate I, D TLBs
	mcr	p15, 0, ip, c8, c7, 0

	@ Step 3
	@ Rrestore MMU settings and turn on MMU
	mcr	p15, 0, r4, c15, c1, 0
	mcr	p15, 0, r5, c13, c0, 0
	mcr	p15, 0, r6, c3, c0, 0
	mcr	p15, 0, r7, c2, c0, 0
	mcr	p15, 0, r8, c1, c0, 1

	@ Get page table address
	mrc	p15, 0, r1, c2, c0, 0
	bic	r1, r1, #0xff
	bic	r1, r1, #0x3f00
	ldr	r2, =0x542e

	@ Mapping resume_turn_on_mmu in the pagetable
	adr	r3, resume_turn_on_mmu
	mov	r3, r3, lsr #20
	orr	r4, r2, r3, lsl #20
	ldr	r5, [r1, r3, lsl #2]
	str     r4, [r1, r3, lsl #2]

	@ Mapping page table address in the page table
	mov	r6, r1, lsr #20
	orr	r7, r2, r6, lsl #20
	ldr	r8, [r1, r6, lsl #2]
	str	r7, [r1, r6, lsl #2]

	ldr	r10, =resume_after_turn_on_mmu
	mov	r10, r10
	b	resume_turn_on_mmu

	.align 5

resume_turn_on_mmu:
	mcr	p15, 0, r9, c1, c0, 0

	@ cp_wait
	mrc	p15, 0, r2, c2, c0, 0
	mov	r2, r2
	mov	r2, r2
	mov	pc, r10
	nop
	nop
	nop
	nop

resume_after_turn_on_mmu:
	@ Restore the Mappings in page table
	str	r5, [r1, r3, lsl #2]
	str	r8, [r1, r6, lsl #2]

	@ Step 4
	@ r0 stores the virtual address of the content save area
	@ compare "modeSaveFlag" to decide which mode will be saved
	ldr	r6, [r0, #SleepState_modeSaveFlags]
1:
	@ restore SVC content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_SVC)
	beq	2f
	add	r7, r0, #SleepState_SVC_REGS
	ldmia	r7, {r2, sp, lr}
	msr	spsr, r2

2:
	@ restore UND mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_UND)
	beq	3f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_UND | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_UND_REGS
	ldmia	r7, {r2, sp, lr}
	msr	spsr, r2

3:
	@ restore ABT mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_ABT)
	beq	4f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_ABT | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_ABT_REGS
	ldmia	r7, {r2, sp, lr}
	msr	spsr, r2

4:
	@ restore IRQ mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_IRQ)
	beq	5f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_IRQ | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_IRQ_REGS
	ldmia	r7, {r2, sp, lr}
	msr	spsr, r2

5:
	@ restore FIQ mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_FIQ)
	beq	6f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_FIQ | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_FIQ_REGS
	ldmia	r7, {r2, r8-r12, sp, lr}
	msr	spsr, r2

6:
	@ restore SYS mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_SYS)
	beq	7f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_SYS | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r0, #SleepState_SYS_REGS
	ldmia	r7, {sp, lr}

7:
	@ Step 5
	@ Re-establish whatever mode was in use at the time pm_enter_sleep()
	@ was invoked and restore complete register context.  Before restoring
	@ the SPSR, make sure that the entry mode was not SYS mode, which has
	@ no SPSR.

	@  Load CPSR, sp and (if not SYS mode) SPSR
	ldr	r3, [r0, #SleepState_ENTRY_CPSR]
	msr	cpsr, r3
	ldr	r2, =CPSR_Mode_SYS
	and	r3, r3, r2
	cmp	r3, r2
	ldrne	r2, [r0, #SleepState_ENTRY_SPSR]
	msrne	spsr, r2
	add	r0, r0, #SleepState_ENTRY_R0
	@ use "increase after" to skip r0 register restore,
	ldmib	r0, {r1 - r12, sp, lr}
	@ restore r0 reigster
	ldr	r0, [r0]

	@ return to next instruction after pm_enter_sleep
	mov	pc, lr

@******************************************************************************
@
@ pxa3xx_init_standby
@
@ Copy standby code into ISRAM
@
@ Inputs:
@	r0 = Start address of relocated program
@
@ Outputs:
@	None
@

ENTRY(pxa3xx_init_standby)
	stmfd	sp!, {r0 - r12, lr}
	ldr	r3, =pm_enter_standby_start
	ldr	r4, =pm_enter_standby_end

rel_ram:
	ldmia	r3!, {r5 - r12}
	stmia	r0!, {r5 - r12}
	cmp	r3, r4
	ble	rel_ram

	ldmfd	sp!, {r0 - r12, pc}

@******************************************************************************
@
@ pm_enter_standby
@
@ Put the system into S0D2C2 state
@
@ Inputs:
@	r0 = Start address of relocated program
@	r1 = Start address of relocated stack
@	r2 = Address of storing OS Timer ticks of exiting standby
@
@ Outputs:
@	None
@

pm_enter_standby:

	@ save registers on stack
	stmfd	sp!, {r3 - r12, lr}
	mov	r8, lr			@ save lr into r8 because SP will be changed
	bl	pxa935_disable_cache

	ldr	r3, =pm_enter_standby_start
	ldr	r4, =pm_enter_standby_end

	mov	r12, r0			@ save start address of program in r12
	mov	r11, r1			@ save start address of stack in r11
	mov	r10, sp			@ save sp in r10
	mov	sp, r11			@ set up new stack address
	mov	lr, r8			@ restore lr from r8
	stmfd	sp!, {lr}		@ store the return address

#if 1
	mov	r5, r0			@ ISRAM start address
	@ copy standby routine to ISRAM
rel_sram:
	ldmia	r3!, {r6-r9}
	stmia	r5!, {r6-r9}
	cmp	r3, r4
	ble	rel_sram
#endif

#if 0
	ldr	r3, =pm_enter_standby_start
	ldr	r4, =pm_enter_standby_end
	sub	r5, r4, r3		@ size of relocated (code + data)
	add	r1, r12, r5		@ end address

	@ the total relocated code+data can't exceed two pages
	mcr	p15, 0, r12, c8, c5, 1	@ invalidate i-TLB entry
	mcr	p15, 0, r12, c10, c4, 0	@ lock pm_enter_standby_start
	mcr	p15, 0, r1, c8, c5, 1	@ invalidate i-TLB entry
	mcr	p15, 0, r1, c10, c4, 0	@ lock pm_enter_standby_end

	mcr	p15, 0, r12, c8, c6, 1	@ invalidate d-TLB entry
	mcr	p15, 0, r12, c10, c8, 0	@ lock pm_enter_standby_start in d-TLB
	mcr	p15, 0, r1, c8, c6, 1	@ invalidate d-TLB entry
	mcr	p15, 0, r1, c10, c8, 0	@ lock pm_enter_standby_end in d-TLB
#endif

	ldr	r3, =ost_membase	@ OS Timer BASE
	ldr	r3, [r3]
	add	r8, r3, #0x40		@ OSCR4
	ldr	r5, [r3]		@ load the address

	ldr	r4, =dmc_membase	@ DMEMC_REG_BASE (MDCNFG)
	ldr	r4, [r4]
	ldr	r5, [r4]

	mov	pc, r0

pm_enter_standby_start:
	b	1f

	.align  5

1:
	@ enter S0D2C2 state
	mov	r5, #PXA3xx_PM_S0D2C2
	mcr     p14, 0, r5, c7, c0, 0

	@ wait for standby
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	@ r2 -- storing OSCR4 ticks
	@ r4 -- DMEMC Base Address
	@ r8 -- OSCR4 address
	ldr	r8, [r8]		@ Load OSCR4 and save it in r8

ddr_reinit:
	@ Re-enable I & D cache
	bl	pxa935_enable_cache

	@ 1) disable DDR_HCAL[HCEN]
	ldr	r5, [r4, #0x60]		@ DDR_HCAL offset 0x60
        bic     r5, r5, #0x80000000     @ clear HCEN
        str     r5, [r4, #0x60]
2:
        ldr     r5, [r4, #0x60]
        tst     r5, #0x80000000
        bne     2b

	@ 2) reset calibration and update DDR_HCAL
	@	set DDR_HCAL[HCPW]
	@	set DDR_HCAL[HCPORG]
	@	set DDR_HCAL[HCOD]
	ldr	r5, [r4, #0x60]		@ DDR_HCAL offset 0x60
	bic	r5, r5, #0x000000df
	bic	r5, r5, #0x00003f00
	bic	r5, r5, #0x3f000000
	orr	r5, r5, #0x38000000
	str	r5, [r4, #0x60]

	@ 3) clear DMCISR[RCI], DMCISR[ORF], DMCISR[DLP]
	mov	r3, #0xe0000000
	str	r3, [r4, #0x78]

	@ 4) enable DMCIER[EDLP]
	ldr	r5, [r4, #0x70]
	orr	r5, r5, #0x20000000
	str	r5, [r4, #0x70]

	@ 5) enable DDR_HCAL
	ldr	r3, [r4, #0x60]
	orr	r3, r3, #0x80000000
	str	r3, [r4, #0x60]

	@ 6) RCOMP cycle
	mov	r0, r4
	bl	xlli_Rcomp

	@ 7) Polling DMCISR[DLP]
	mrc	p15, 0, r5, c0, c0, 0		@ get cpu id
	mov	r6, #0xf0			@ cpu id mask: 0xfff0
	orr	r6, r6, #0xff00
	and	r5, r5, r6			@ cpuid & 0xfff0
	mov	r6, #0x30			@ PXA935/PXA940 cpu id & 0xfff0
	orr	r6, r6, #0x6900			@ == 0x6930
	teq	r5, r6				@ is it PXA935/PXA940?
	beq	4f				@ yes, then do not poll DLP bit
3:
	ldr	r5, [r4, #0x78]
	tst	r5, #0x20000000
	beq	3b
4:
	@ 8) disable DMCIER[EDLP] and strobe calibration finished
	ldr	r3, [r4, #0x70]
	bic	r3, r3, #0x20000000
	str	r3, [r4, #0x70]

	@ 9) enable MDCNFG[DMCEN]
	ldr	r5, [r4]
	orr	r5, r5, #0x40000000
	str	r5, [r4]

	@ 10) tune the calibration range
	ldr	r3, [r4, #0x60]
	orr	r3, r3, #0x00000003
	str	r3, [r4, #0x60]

	str	r8, [r2]			@ save OST tick
	ldmfd	sp!, {lr}
	mov	sp, r10				@ restore stack address

#if 0
	@ unlock TLB
	ldr	r4, =pm_enter_standby_start
	ldr	r3, =pm_enter_standby_end
	sub	r5, r3, r4
	add	r6, r12, r5

	mcr	p15, 0, r12, c10, c4, 0		@ unlock i-TLB
	mcr	p15, 0, r6, c10, c4, 0		@ unlock i-TLB
	mcr	p15, 0, r12, c8, c5, 1		@ invalidate i-TLB
	mcr	p15, 0, r6, c8, c5, 1		@ invalidate i-TLB
	mcr	p15, 0, r12, c8, c6, 1		@ invalidate d-TLB
	mcr	p15, 0, r6, c8, c6, 1		@ invalidate d-TLB
#endif
        ldmfd   sp!, {r3 - r12, pc}


@******************************************************************************
@
@ pxa935_disable_cache
@
@ Disable cache before entering Low power mode
@
@ Inputs:
@	None
@
@ Outputs:
@	None
@

ENTRY(pxa935_disable_cache)
	@ save registers on stack
	stmfd	sp!, {r0 - r12, lr}

	@ Check whether CPU is PXA935
	mrc	p15, 0, r3, c0, c0, 0
	mov	r1, #0xf0
	orr	r1, r1, #0xff00
	and	r2, r3, r1
	mov	r5, #0x30
	orr	r5, r5, #0x6900
	cmp	r2, r5
	bne	disable_cache_end

	@ Check whether CPU is PXA935 Ax & B0 stepping
	mov	r1, #0xff
	orr	r1, r1, #0xff00
	and	r2, r3, r1
	mov	r5, #0x37
	orr	r5, r5, #0x6900
	subs	r2, r2, r5
	bcs	disable_cache_end

	@ clean & invalidate D cache
	mov	r2, #0x1f00
	orr	r2, r2, #0x00e0
1:
	mcr	p15, 0, r2, c7, c14, 2	@ clean/invalidate L1 D line
	adds	r2, r2, #0x40000000
	bcc	1b
	subs	r2, r2, #0x20
	bpl	1b

	@ invalidate I cache & BTB
	mcr	p15, 0, r2, c7, c5, 0	@ invalidate I cache & BTB

	mcr	p15, 0, r2, c7, c10, 4	@ data writer barrier
	mcr	p15, 0, r2, c7, c5, 4	@ prefetch flush

	mov	r1, #0x1000		@ I cache flag
	orr	r1, r1, #0x4		@ D cache flag
	mvn	r3, r1
	@ Disable I & D cache
	mrc	p15, 0, r2, c1, c0, 0
	and	r0, r2, r3
	mcr	p15, 0, r0, c1, c0, 0

disable_cache_end:
	ldmfd	sp!, {r0 - r12, pc}

ENTRY(pxa935_enable_cache)
	@ save registers on stack
	stmfd	sp!, {r0 - r12, lr}

	@ Check whether CPU is PXA935
	mrc	p15, 0, r3, c0, c0, 0
	mov	r1, #0xf0
	orr	r1, r1, #0xff00
	and	r2, r3, r1
	mov	r5, #0x30
	orr	r5, r5, #0x6900
	cmp	r2, r5
	bne	enable_cache_end

	@ Check whether CPU is PXA935 Ax & B0 stepping
	mov	r1, #0xff
	orr	r1, r1, #0xff00
	and	r2, r3, r1
	mov	r5, #0x37
	orr	r5, r5, #0x6900
	subs	r2, r2, r5
	bcs	enable_cache_end

	@ Unlock I Cache
	mcr	p15, 0, r2, c9, c5, 1
	@ Unlock D Cache
	mcr	p15, 0, r2, c9, c6, 1
	@ Invalidate I & D Cache, BTB
	mcr	p15, 0, r2, c7, c7, 0
	@ Enable I & D Cache
	mrc	p15, 0, r2, c1, c0, 0
	mov	r1, #0x1000
	orr	r1, r1, #0x4
	orr	r2, r2, r1
	mcr	p15, 0, r2, c1, c0, 0
enable_cache_end:
	ldmfd	sp!, {r0 - r12, pc}


	@ Rcomp
	@ Input Parameter Required
	@	R0 = Dynamic Memory Control Register Base Address
	@
xlli_Rcomp:
	stmfd	sp!, {r0 - r12, lr}		@ save r0 - r12 and link on stack
	mov	r4, r0				@ Copy Dynamic Memory Control Register Base

	@ 1) Perform Resistive Compensation calibration cycle (RCOMP)
	mov	r1, #4				@ Set up RCOMP retry loop count
xlli_m3:
	ldr	r3, [r4, #0x100]		@ Write initial values to RCOMP
	orr	r3, r3, #0x80000000		@ Set bit to perform the RCOMP
	str	r3, [r4, #0x100]		@ Start the RCOMP evaluation
	@
	@ Wait for Rcomp evalution cycle to complete - RCOMP[SWEVAL] clears
	@ A "time out" counter is in this loop - just in case the eval bit doesn't clear
	@ so there is an exit path
	@
	mov	r0, #0x10000			@ Set up arbitrary time out counter
xlli_m4:
	ldr	r2, [r4, #0x100]		@ Fetch status
	subs	r0, r0, #1			@ Decrement loop count
	beq	xlli_m9				@ Branch if timed out
	ands	r2, r2, #0x80000000		@ Is the evaluation complete?
	bne	xlli_m4				@ No - loop until done
	mrc	p15, 0, r2, c0, c0, 0		@ get cpu id
	mov	r3, #0xf0			@ cpu id mask: 0xfff0
	orr	r3, r3, #0xff00
	and	r5, r2, r3			@ cpuid & 0xfff0
	mov	r3, #0x30			@ PXA935/PXA940 cpu id & 0xfff0
	orr	r3, r3, #0x6900			@ == 0x6930
	teq	r5, r3				@ is it PXA935/PXA940?
	bne	continue_pxa3xx_rcomp		@ no
	mov	r3, #0x0f			@ yes
	add	r2, r2, r3
	subs	r2, r2, #0x06			@ 6 means PXA935 B0 stepping
	bcc	continue_pxa935_ax_rcomp
	b	continue_pxa935_bx_rcomp

xlli_m9:
	subs	r1, r1, #1			@ Decrement re-try count
	bne	xlli_m3				@ Try again if count is not zero

	@ if we time out, we have an issue. We will just continue in this case.

continue_pxa3xx_rcomp:
	ldr	r2, [r4, #0x78]			@ Get results of the evaluation from DMCISR
	and	r6, r2, #0x003f8000		@ Extract NCODE
	mov	r6, r6, lsr #15			@ and right justify
	and	r5, r2, #0x1fc00000		@ Extract PCODE
	mov	r5, r5, lsr #22			@ and right justify

@	At this point: r5 = PCODE, r6 = NCODE
@       The PSLEW and NSLEW values are calculated as follows:
@
@       PSLEW = 1.9473 - 0.051(PCODE) + 0.1914(NCODE)
@       NSLEW = - 2.0786 + 0.2739(PCODE) + 0.0279(NCODE)
@       To gain the desired accuracy, the constant values above are multiplied
@       by 10,000 (so 0.051 will become 510) and the math is done as integer math
@       rather than floating point. When the calculations are done, the result
@       is divided by 10,000 and rounded up/down to the closest integer value
@       for PSLEW and NSLEW.

	mov	r3, #0x7A			@ load 0.1914 * 10,000 into r3
	orr	r3, r3, #0x0700			@ 0x77A == 1914
	mul	r2, r3, r6			@ r2 = 1914(NCODE)
	mov	r3, #0xFE			@ load 0.051 * 10,000 into r3
	orr	r3, r3, #0x0100			@ 0x1FE == 510
	mul	r1, r3, r5			@ r1 = 510(PCODE)
	sub	r2, r2, r1			@ subtract the PCODE from the NCODE
	mov	r3, #0x11			@ load 1.9473 * 10000 into r3
	orr	r3, r3, #0x4C00			@ 0x4C11 == 19473
	add	r0, r2, r3			@ r0 = PSLEW * 10,000 at this point
	mov	r3, #0x88			@ Set up the midpoint of 10,000
	orr	r3, r3, #0x1300			@ 0x1388 == 5000
	add	r0, r0, r3			@ Add to PSLEW (will provide 
						@ rounded PSLEW after divide)
	mov	r1, #0x10			@ Set up the divisor
	orr	r1, r1, #0x27			@ 0x2710 == 10000
	bl	xlli_U32Divide			@ Divide
	mov	r7, r2				@ Save the calculated PSLEW value in r7

	@
	@ Do the NSLEW calculation. 5000 is added to NSLEW before it is divided
	@ by 10,000 so the result is rounded to the closest whole number.
	@
	mov	r3, #0xB3			@ load 0.2739 * 10,000 into r3
	orr	r3, r3, #0xA00			@ 0xAB3 == 2739
	mul	r1, r3, r5			@ r1 = 2739(PCODE)
	mov	r3, #0x17			@ load 0.0279 * 10,000 into r3
	orr	r3, r3, #0x100			@ 0x117 == 279
	mul	r2, r3, r6			@ r2 = 279(NCODE)
	add	r1, r1, r2			@ Add the NCODE to the PCODE
	mov	r3, #0x32			@ load 2.0786 * 10,000 into r3
	orr	r3, r3, #0x5100			@ 0x5132 == 20786
	sub	r0, r1, r3			@ r0 = NSLEW * 10,000 at this point
	mov	r3, #0x88			@ Set up the midpoint of 10,000
	orr	r3, r3, #0x1300			@ 0x1388 == 5000
	add	r0, r0, r3			@ Add to NSLEW (will provide 
						@ rounded NSLEW after divide)
	mov	r1, #0x10			@ Set up the divisor
	orr	r1, r1, #0x27			@ 0x2710 == 10000
	bl	xlli_U32Divide			@ Divide
	mov	r8, r2				@ Save the calculated NSLEW value in r8

	@
	@ At this point the registers we care about contain the following:
	@
	@	r4 = Dynamic Memory Control Register Base Address
	@	r5 = PCODE value
	@	r6 = NCODE value
	@	r7 = PSLEW value
	@	r8 = NSLEW value
	@
	@ Next Step is to combine the NCODE, PCODE, NSLEW, PSLEW values and program
	@ all the pad registers to the same value.
	@
	mov	r5, r5, lsl #24			@ Move the PCODE into position
	mov	r6, r6, lsl #16			@ Move the NCODE into position
	mov	r7, r7, lsl #8			@ Move the PSLEW into position
	orr	r8, r8, r7			@ OR the NSLEW and PSLEW values together
	orr	r8, r8, r6
	orr	r8, r8, r5
	@
	@ The PAD register data has been assembled into r3. Now the code needs
	@ to send this data to the selected PAD registers.
	@
	str	r8, [r4, #0x110]	@ Load the PAD MA register
	str	r8, [r4, #0x114]	@ Load the PAD MDMSB register
	str	r8, [r4, #0x118]	@ Load the PAD MDLSB register
	str	r8, [r4, #0x11c]	@ Load the PAD DMEM register
	str	r8, [r4, #0x120]	@ Load the PAD SDCLK register
	str	r8, [r4, #0x124]	@ Load the PAD SDCS register
	str	r8, [r4, #0x128]	@ Load the PAD SMEM register
	str	r8, [r4, #0x12c]	@ Load the PAD SCLK register
	@
	@ Set the UPDATE bit in the Rcomp register
	@
	ldr	r3, [r4, #0x100]		@ Fetch RCOMP register value
	orr	r3, r3, #0x40000000		@ Set the UPDATE bit
	str	r3, [r4, #0x100]		@ Write back

	ldmfd	sp!, {r0 - r12, pc}

continue_pxa935_ax_rcomp:
	mov	r5, #0x10			@ Hardcoded the PCODE
	mov	r6, #0x10			@ Hardcoded the NCODE
	mov	r7, #0x06			@ Hardcoded the XCODE
	mov	r8, #0x01			@ Hardcoded the SR(Slew Rate)

	@ At this point the registers we care about contain the following:
	@
	@	r4 = Dynamic Memory Control Register Base Address
	@	r5 = PCODE value
	@	r6 = NCODE value
	@	r7 = XCODE value
	@	r8 = SR value
	@
	@ Next Step is to combine the NCODE, PCODE, XCODE, SR values and program
	@ all the pad registers to the same value.
	@
	mov	r5, r5, lsl #24			@ Move the PCODE into position
	mov	r6, r6, lsl #16			@ Move the NCODE into position
	mov	r7, r7, lsl #8			@ Move the XCODE into position
	orr	r8, r8, r7			@ OR the XCODE and SR values together
	orr	r8, r8, r6			@ OR in the NCODE value
	orr	r8, r8, r5			@ OR in the PCODE value
	@
	@ The PAD register data has been assembled into r3. Now the code needs
	@ to send this data to the selected PAD registers.
	@
	str	r8, [r4, #0x110]	@ Load the PAD CMD register
	str	r8, [r4, #0x118]	@ Load the PAD QUAD register
	str	r8, [r4, #0x120]	@ Load the PAD SDCLK register

	@
	@ Set the UPDATE bit in the Rcomp register
	@
	ldr	r3, [r4, #0x100]		@ Fetch RCOMP register value
	orr	r3, r3, #0x40000000		@ Set the UPDATE bit
	str	r3, [r4, #0x100]		@ Write back

	ldmfd	sp!, {r0 - r12, pc}

continue_pxa935_bx_rcomp:
	ldr	r2, [r4, #0x78]			@ Get results of the evaluation from DMCISR
	and	r5, r2, #0x1fc00000		@ Extract PCODE
	cmp	r5, #0
	orreq	r5, r5, #0x04000000		@ Hardcode PCODE if we can't get PCODE
	mov	r5, r5, lsr #22			@ and right justify
	mov	r6, r5				@ NCODE use same value as PCODE
	ldr	r2, [r4, #0x7c]			@ Get results of the evaluation from DMCISR2
	and	r7, r2, #0x0f			@ Extract XCODE
	cmp	r7, #0
	moveq	r7, #6				@ Hardcode XCODE if we can't get XCODE
	mov	r8, #1				@ Hardcode SR

	@ At this point the registers we care about contain the following:
	@
	@	r4 = Dynamic Memory Control Register Base Address
	@	r5 = PCODE value
	@	r6 = NCODE value
	@	r7 = XCODE value
	@	r8 = SR value
	@
	@ Next Step is to combine the NCODE, PCODE, XCODE, SR values and program
	@ all the pad registers to the same value.
	@
	and	r5, r5, #0x1f
	and	r6, r6, #0x1f
	and	r7, r7, #0x0f
	mov	r5, r5, lsl #24			@ Move the PCODE into position
	mov	r6, r6, lsl #16			@ Move the NCODE into position
	mov	r7, r7, lsl #8			@ Move the XCODE into position
	orr	r8, r8, r7			@ OR the XCODE and SR values together
	orr	r8, r8, r6			@ OR in the NCODE value
	orr	r8, r8, r5			@ OR in the PCODE value

	@
	@ The PAD register data has been assembled into r3. Now the code needs
	@ to send this data to the selected PAD registers.
	@
	str	r8, [r4, #0x110]	@ Load the PAD CMD register
	str	r8, [r4, #0x118]	@ Load the PAD QUAD register
	str	r8, [r4, #0x120]	@ Load the PAD SDCLK register

	@
	@ Set the UPDATE bit in the Rcomp register
	@
	ldr	r3, [r4, #0x100]		@ Fetch RCOMP register value
	orr	r3, r3, #0x40000000		@ Set the UPDATE bit
	str	r3, [r4, #0x100]		@ Write back

	ldmfd	sp!, {r0 - r12, pc}
	
	
	@ xlli_U32Divide
	@ Parameter Passing
	@	r0 contains the dividend (preserved)
	@	r1 contains the divisor (preserved)
	@	r2 will contain the result on subroutine exit
	@	r3 will contain the remainder on subroutine exit

xlli_U32Divide:
	stmfd	sp!, {r0, r1, r4, r5, lr}	@ Save used registers

	mov	r2, #0				@ Clear result
	mov	r3, #0				@ Clear remainder
	mov	r5, #32				@ load r5 with loop count
	mrs	r4, cpsr			@ Get CPSR
	bic	r4, r4, #0x20000000		@ Clear carry bit

	@
	@ Main division loop begins here
	@
xlli_D1:
	msr	cpsr_f, r4			@ Write flags back
	adcs	r2, r2, r2			@ Shift C bit into result
	adcs	r0, r0, r0			@ Rotate r0 left 1 bit through C bit
	adcs	r3, r3, r3			@ Rotate data left into r3
	subs	r3, r3, r1			@ Subtract dividend
	blt	xlli_D2				@ Branch if negative
	mrs	r4, cpsr			@ Get CPSR
	orr	r4, r4, #0x20000000		@ Set carry bit
	msr	cpsr_f, r4			@ Write flags back
	b	xlli_D3				@ Check loop count

xlli_D2:
	add	r3, r3, r1			@ Restore r3
	mrs	r4, cpsr			@ Get CPSR
	bic	r4, r4, #0x20000000		@ Clear carry bit
	msr	cpsr_f, r4			@ Write flags back

xlli_D3:
	mrs	r4, CPSR			@ Get CPSR
	subs	r5, r5, #1			@ Decrement loop count
	bne	xlli_D1				@ Loop until done

	@
	@ Shift the last bit into the result and return to caller
	@
	msr	cpsr_f, r4			@ Write flags back
	adcs	r2, r2, r2			@ Shift C bit into result
	adcs	r0, r0, r0			@ Rotate r0 left 1 bit through C bit

	ldmfd	sp!, {r0, r1, r4, r5, pc}	@ Return to caller

pm_enter_standby_end:
	nop


@******************************************************************************
@
@ pm_enter_lcd_refresh
@
@ Put the system into S0D1C2 state
@
@ Inputs:
@	r0 = Start address of relocated program
@	r1 = Start address of relocated stack
@	r2 = Address of storing OS Timer ticks of exiting standby
@
@ Outputs:
@	None
@

pm_enter_lcd_refresh:

	@ save registers on stack
	stmfd	sp!, {r3 - r12, lr}
	ldr	r3, =pm_enter_lcd_refresh_start
	ldr	r4, =pm_enter_lcd_refresh_end

	mov	r12, r0			@ save start address of program in r12
	mov	r11, r1			@ save start address of stack in r11
	mov	r10, sp			@ save sp in r10
	mov	sp, r11			@ set up new stack address
	stmfd	sp!, {lr}		@ store the return address

#if 1
	mov	r5, r0			@ ISRAM start address
	@ copy standby routine to ISRAM
lcd_refresh_rel_sram:
	ldmia	r3!, {r6-r9}
	stmia	r5!, {r6-r9}
	cmp	r3, r4
	ble	lcd_refresh_rel_sram
#endif

#if 0
	ldr	r3, =pm_enter_standby_start
	ldr	r4, =pm_enter_standby_end
	sub	r5, r4, r3		@ size of relocated (code + data)
	add	r1, r12, r5		@ end address

	@ the total relocated code+data can't exceed two pages
	mcr	p15, 0, r12, c8, c5, 1	@ invalidate i-TLB entry
	mcr	p15, 0, r12, c10, c4, 0	@ lock pm_enter_standby_start
	mcr	p15, 0, r1, c8, c5, 1	@ invalidate i-TLB entry
	mcr	p15, 0, r1, c10, c4, 0	@ lock pm_enter_standby_end

	mcr	p15, 0, r12, c8, c6, 1	@ invalidate d-TLB entry
	mcr	p15, 0, r12, c10, c8, 0	@ lock pm_enter_standby_start in d-TLB
	mcr	p15, 0, r1, c8, c6, 1	@ invalidate d-TLB entry
	mcr	p15, 0, r1, c10, c8, 0	@ lock pm_enter_standby_end in d-TLB
#endif

	ldr	r3, =ost_membase	@ OS Timer BASE
	ldr	r3, [r3]
	add	r8, r3, #0x40		@ OSCR4
	ldr	r5, [r3]		@ load the address

	ldr	r4, =dmc_membase	@ DMEMC_REG_BASE (MDCNFG)
	ldr	r4, [r4]
	ldr	r5, [r4]

	mov	pc, r0

pm_enter_lcd_refresh_start:
	b	1f

	.align  5

1:
	@ enter S0D1C2 state
	mov	r5, #PXA3xx_PM_S0D1C2
	mcr     p14, 0, r5, c7, c0, 0

	@ wait for lcd refresh
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	ldr	r8, [r8]		@ Load OSCR4 and save it in r8

lcd_refresh_ddr_reinit:
	@ r4 stores MDCNFG address
	@ 1) disable MDCNFG[DMCEN]
	ldr	r3, [r4]
	bic	r3, r3, #0x40000000
	str	r3, [r4]
1:
	ldr	r3, [r4]
	tst	r3, #0x40000000
	bne	1b

	@ 2) disable DDR_HCAL[HCEN]
	ldr	r5, [r4, #0x60]		@ DDR_HCAL offset 0x60
        bic     r5, r5, #0x80000000     @ clear HCEN
        str     r5, [r4, #0x60]
2:
        ldr     r5, [r4, #0x60]
        tst     r5, #0x80000000
        bne     2b

	@ 2) reset calibration and update DDR_HCAL
	@	set DDR_HCAL[HCPW]
	@	set DDR_HCAL[HCPORG]
	@	set DDR_HCAL[HCOD]
	ldr	r5, [r4, #0x60]		@ DDR_HCAL offset 0x60
	bic	r5, r5, #0x000000df
	bic	r5, r5, #0x00003f00
	bic	r5, r5, #0x3f000000
	orr	r5, r5, #0x38000000
	str	r5, [r4, #0x60]

	@ 3) clear DMCISR[ORF], DMCISR[DLP]
	mov	r3, #0x60000000
	str	r3, [r4, #0x78]

	@ 4) enable DMCIER[EDLP]
	ldr	r5, [r4, #0x70]
	orr	r5, r5, #0x20000000
	str	r5, [r4, #0x70]

	@ 5) enable DDR_HCAL
	ldr	r3, [r4, #0x60]
	orr	r3, r3, #0x80000000
	str	r3, [r4, #0x60]

	@ 6) RCOMP cycle
	mov	r0, r4
	bl	lcd_refresh_xlli_Rcomp

	@ 7) Polling DMCISR[DLP]
	mrc	p15, 0, r5, c0, c0		@ get cpu id
	mov	r6, #0xf0			@ cpu id mask: 0xfff0
	orr	r6, r6, #0xff00
	and	r5, r5, r6			@ cpuid & 0xfff0
	mov	r6, #0x30			@ PXA935/PXA940 cpu id & 0xfff0
	orr	r6, r6, #0x6900			@ == 0x6930
	teq	r5, r6				@ is it PXA935/PXA940?
	beq	4f				@ yes, then do not poll DLP bit
3:
	ldr	r5, [r4, #0x78]
	tst	r5, #0x20000000
	beq	3b
4:
	@ 8) disable DMCIER[EDLP] and strobe calibration finished
	ldr	r3, [r4, #0x70]
	bic	r3, r3, #0x20000000
	str	r3, [r4, #0x70]

	@ 9) enable MDCNFG[DMCEN]
	ldr	r5, [r4]
	orr	r5, r5, #0x40000000
	str	r5, [r4]

	@ 10) tune the calibration range
	ldr	r3, [r4, #0x60]
	orr	r3, r3, #0x00000003
	str	r3, [r4, #0x60]

	str	r8, [r2]			@ save OST tick
	ldmfd	sp!, {lr}
	mov	sp, r10				@ restore stack address

#if 0
	@ unlock TLB
	ldr	r4, =pm_enter_standby_start
	ldr	r3, =pm_enter_standby_end
	sub	r5, r3, r4
	add	r6, r12, r5

	mcr	p15, 0, r12, c10, c4, 0		@ unlock i-TLB
	mcr	p15, 0, r6, c10, c4, 0		@ unlock i-TLB
	mcr	p15, 0, r12, c8, c5, 1		@ invalidate i-TLB
	mcr	p15, 0, r6, c8, c5, 1		@ invalidate i-TLB
	mcr	p15, 0, r12, c8, c6, 1		@ invalidate d-TLB
	mcr	p15, 0, r6, c8, c6, 1		@ invalidate d-TLB
#endif
        ldmfd   sp!, {r3 - r12, pc}


	@ Rcomp
	@ Input Parameter Required
	@	R0 = Dynamic Memory Control Register Base Address
	@
lcd_refresh_xlli_Rcomp:
	stmfd	sp!, {r0 - r12, lr}		@ save r0 - r12 and link on stack
	mov	r4, r0				@ Copy Dynamic Memory Control Register Base

	@ 1) Perform Resistive Compensation calibration cycle (RCOMP)
	mov	r1, #4				@ Set up RCOMP retry loop count
lcd_refresh_xlli_m3:
	ldr	r3, [r4, #0x100]		@ Write initial values to RCOMP
	orr	r3, r3, #0x80000000		@ Set bit to perform the RCOMP
	str	r3, [r4, #0x100]		@ Start the RCOMP evaluation
	@
	@ Wait for Rcomp evalution cycle to complete - RCOMP[SWEVAL] clears
	@ A "time out" counter is in this loop - just in case the eval bit doesn't clear
	@ so there is an exit path
	@
	mov	r0, #0x10000			@ Set up arbitrary time out counter
lcd_refresh_xlli_m4:
	ldr	r2, [r4, #0x100]		@ Fetch status
	subs	r0, r0, #1			@ Decrement loop count
	beq	lcd_refresh_xlli_m9				@ Branch if timed out
	ands	r2, r2, #0x80000000		@ Is the evaluation complete?
	bne	lcd_refresh_xlli_m4				@ No - loop until done
	mrc	p15, 0, r2, c0, c0		@ get cpu id
	mov	r3, #0xf0			@ cpu id mask: 0xfff0
	orr	r3, r3, #0xff00
	and	r2, r2, r3			@ cpuid & 0xfff0
	mov	r3, #0x30			@ PXA935/PXA940 cpu id & 0xfff0
	orr	r3, r3, #0x6900			@ == 0x6930
	teq	r2, r3				@ is it PXA935/PXA940?
	bne	lcd_refresh_continue_pxa3xx_rcomp		@ no
	beq	lcd_refresh_continue_pxa935_rcomp		@ yes

lcd_refresh_xlli_m9:
	subs	r1, r1, #1			@ Decrement re-try count
	bne	lcd_refresh_xlli_m3				@ Try again if count is not zero

	@ if we time out, we have an issue. We will just continue in this case.

lcd_refresh_continue_pxa3xx_rcomp:
	ldr	r2, [r4, #0x78]			@ Get results of the evaluation from DMCISR
	and	r6, r2, #0x003f8000		@ Extract NCODE
	mov	r6, r6, lsr #15			@ and right justify
	and	r5, r2, #0x1fc00000		@ Extract PCODE
	mov	r5, r5, lsr #22			@ and right justify

@	At this point: r5 = PCODE, r6 = NCODE
@       The PSLEW and NSLEW values are calculated as follows:
@
@       PSLEW = 1.9473 - 0.051(PCODE) + 0.1914(NCODE)
@       NSLEW = - 2.0786 + 0.2739(PCODE) + 0.0279(NCODE)
@       To gain the desired accuracy, the constant values above are multiplied
@       by 10,000 (so 0.051 will become 510) and the math is done as integer math
@       rather than floating point. When the calculations are done, the result
@       is divided by 10,000 and rounded up/down to the closest integer value
@       for PSLEW and NSLEW.

	mov	r3, #0x7A			@ load 0.1914 * 10,000 into r3
	orr	r3, r3, #0x0700			@ 0x77A == 1914
	mul	r2, r3, r6			@ r2 = 1914(NCODE)
	mov	r3, #0xFE			@ load 0.051 * 10,000 into r3
	orr	r3, r3, #0x0100			@ 0x1FE == 510
	mul	r1, r3, r5			@ r1 = 510(PCODE)
	sub	r2, r2, r1			@ subtract the PCODE from the NCODE
	mov	r3, #0x11			@ load 1.9473 * 10000 into r3
	orr	r3, r3, #0x4C00			@ 0x4C11 == 19473
	add	r0, r2, r3			@ r0 = PSLEW * 10,000 at this point
	mov	r3, #0x88			@ Set up the midpoint of 10,000
	orr	r3, r3, #0x1300			@ 0x1388 == 5000
	add	r0, r0, r3			@ Add to PSLEW (will provide 
						@ rounded PSLEW after divide)
	mov	r1, #0x10			@ Set up the divisor
	orr	r1, r1, #0x27			@ 0x2710 == 10000
	bl	lcd_refresh_xlli_U32Divide			@ Divide
	mov	r7, r2				@ Save the calculated PSLEW value in r7

	@
	@ Do the NSLEW calculation. 5000 is added to NSLEW before it is divided
	@ by 10,000 so the result is rounded to the closest whole number.
	@
	mov	r3, #0xB3			@ load 0.2739 * 10,000 into r3
	orr	r3, r3, #0xA00			@ 0xAB3 == 2739
	mul	r1, r3, r5			@ r1 = 2739(PCODE)
	mov	r3, #0x17			@ load 0.0279 * 10,000 into r3
	orr	r3, r3, #0x100			@ 0x117 == 279
	mul	r2, r3, r6			@ r2 = 279(NCODE)
	add	r1, r1, r2			@ Add the NCODE to the PCODE
	mov	r3, #0x32			@ load 2.0786 * 10,000 into r3
	orr	r3, r3, #0x5100			@ 0x5132 == 20786
	sub	r0, r1, r3			@ r0 = NSLEW * 10,000 at this point
	mov	r3, #0x88			@ Set up the midpoint of 10,000
	orr	r3, r3, #0x1300			@ 0x1388 == 5000
	add	r0, r0, r3			@ Add to NSLEW (will provide 
						@ rounded NSLEW after divide)
	mov	r1, #0x10			@ Set up the divisor
	orr	r1, r1, #0x27			@ 0x2710 == 10000
	bl	lcd_refresh_xlli_U32Divide			@ Divide
	mov	r8, r2				@ Save the calculated NSLEW value in r8

	@
	@ At this point the registers we care about contain the following:
	@
	@	r4 = Dynamic Memory Control Register Base Address
	@	r5 = PCODE value
	@	r6 = NCODE value
	@	r7 = PSLEW value
	@	r8 = NSLEW value
	@
	@ Next Step is to combine the NCODE, PCODE, NSLEW, PSLEW values and program
	@ all the pad registers to the same value.
	@
	mov	r5, r5, lsl #24			@ Move the PCODE into position
	mov	r6, r6, lsl #16			@ Move the NCODE into position
	mov	r7, r7, lsl #8			@ Move the PSLEW into position
	orr	r8, r8, r7			@ OR the NSLEW and PSLEW values together
	orr	r8, r8, r6
	orr	r8, r8, r5
	@
	@ The PAD register data has been assembled into r3. Now the code needs
	@ to send this data to the selected PAD registers.
	@
	str	r8, [r4, #0x110]	@ Load the PAD MA register
	str	r8, [r4, #0x114]	@ Load the PAD MDMSB register
	str	r8, [r4, #0x118]	@ Load the PAD MDLSB register
	str	r8, [r4, #0x11c]	@ Load the PAD DMEM register
	str	r8, [r4, #0x120]	@ Load the PAD SDCLK register
	str	r8, [r4, #0x124]	@ Load the PAD SDCS register
	str	r8, [r4, #0x128]	@ Load the PAD SMEM register
	str	r8, [r4, #0x12c]	@ Load the PAD SCLK register
	@
	@ Set the UPDATE bit in the Rcomp register
	@
	ldr	r3, [r4, #0x100]		@ Fetch RCOMP register value
	orr	r3, r3, #0x40000000		@ Set the UPDATE bit
	str	r3, [r4, #0x100]		@ Write back

	ldmfd	sp!, {r0 - r12, pc}

lcd_refresh_continue_pxa935_rcomp:
	mov	r5, #0x10			@ Hardcoded the PCODE
	mov	r6, #0x10			@ Hardcoded the NCODE
	mov	r7, #0x06			@ Hardcoded the XCODE
	mov	r8, #0x01			@ Hardcoded the SR(Slew Rate)

	@ At this point the registers we care about contain the following:
	@
	@	r4 = Dynamic Memory Control Register Base Address
	@	r5 = PCODE value
	@	r6 = NCODE value
	@	r7 = XCODE value
	@	r8 = SR value
	@
	@ Next Step is to combine the NCODE, PCODE, XCODE, SR values and program
	@ all the pad registers to the same value.
	@
	mov	r5, r5, lsl #24			@ Move the PCODE into position
	mov	r6, r6, lsl #16			@ Move the NCODE into position
	mov	r7, r7, lsl #8			@ Move the XCODE into position
	orr	r8, r8, r7			@ OR the XCODE and SR values together
	orr	r8, r8, r6			@ OR in the NCODE value
	orr	r8, r8, r5			@ OR in the PCODE value
	@
	@ The PAD register data has been assembled into r3. Now the code needs
	@ to send this data to the selected PAD registers.
	@
	str	r8, [r4, #0x110]	@ Load the PAD CMD register
	str	r8, [r4, #0x118]	@ Load the PAD QUAD register
	str	r8, [r4, #0x120]	@ Load the PAD SDCLK register

	@
	@ Set the UPDATE bit in the Rcomp register
	@
	ldr	r3, [r4, #0x100]		@ Fetch RCOMP register value
	orr	r3, r3, #0x40000000		@ Set the UPDATE bit
	str	r3, [r4, #0x100]		@ Write back

	ldmfd	sp!, {r0 - r12, pc}

	
	@ xlli_U32Divide
	@ Parameter Passing
	@	r0 contains the dividend (preserved)
	@	r1 contains the divisor (preserved)
	@	r2 will contain the result on subroutine exit
	@	r3 will contain the remainder on subroutine exit

lcd_refresh_xlli_U32Divide:
	stmfd	sp!, {r0, r1, r4, r5, lr}	@ Save used registers

	mov	r2, #0				@ Clear result
	mov	r3, #0				@ Clear remainder
	mov	r5, #32				@ load r5 with loop count
	mrs	r4, cpsr			@ Get CPSR
	bic	r4, r4, #0x20000000		@ Clear carry bit

	@
	@ Main division loop begins here
	@
lcd_refresh_xlli_D1:
	msr	cpsr_f, r4			@ Write flags back
	adcs	r2, r2, r2			@ Shift C bit into result
	adcs	r0, r0, r0			@ Rotate r0 left 1 bit through C bit
	adcs	r3, r3, r3			@ Rotate data left into r3
	subs	r3, r3, r1			@ Subtract dividend
	blt	lcd_refresh_xlli_D2				@ Branch if negative
	mrs	r4, cpsr			@ Get CPSR
	orr	r4, r4, #0x20000000		@ Set carry bit
	msr	cpsr_f, r4			@ Write flags back
	b	lcd_refresh_xlli_D3				@ Check loop count

lcd_refresh_xlli_D2:
	add	r3, r3, r1			@ Restore r3
	mrs	r4, cpsr			@ Get CPSR
	bic	r4, r4, #0x20000000		@ Clear carry bit
	msr	cpsr_f, r4			@ Write flags back

lcd_refresh_xlli_D3:
	mrs	r4, CPSR			@ Get CPSR
	subs	r5, r5, #1			@ Decrement loop count
	bne	lcd_refresh_xlli_D1				@ Loop until done

	@
	@ Shift the last bit into the result and return to caller
	@
	msr	cpsr_f, r4			@ Write flags back
	adcs	r2, r2, r2			@ Shift C bit into result
	adcs	r0, r0, r0			@ Rotate r0 left 1 bit through C bit

	ldmfd	sp!, {r0, r1, r4, r5, pc}	@ Return to caller

pm_enter_lcd_refresh_end:
	nop



@*****************************************************************************
@ pm_enter_sleep_or_deep_sleep
@
@ Put the system into S2D3C4 or S3D4C4 state
@
@ Inputs:
@	r0: the virutal address of the data area to save the content of core
@	r1: sleep type, 6(sleep), 7(deep sleep)
@
@ Outputs:
@	None
@
@ Notes:
@      r1 should be saved previously
@

pm_enter_sleep_or_deep_sleep:

	@ Step 1
	@ store registers(r0-r12), sp, lr of current mode in the data array
	@ ENTRY_REGS
	@ the r0 changes to be virutal address of ENTRY_REGS
	add	r0, r0, #SleepState_ENTRY_R0
	@ skip r0 and r1 save
	add	r0, r0, #4
	stmib	r0, {r2 - r12, sp, lr}
	sub	r0, r0, #4
	mov	r11, r1 @save the sleep type
	@ save r0 register
	sub	r5, r0, #SleepState_ENTRY_R0
	str	r5, [r0]

	@ store cpsr of current mode in the data array ENTRY_REGS.
	mrs	r3, cpsr
	str	r3, [r5, #SleepState_ENTRY_CPSR]

	@ store spsr(if not SYS mode) of current mode in the content area
	ldr	r2, =CPSR_Mode_SYS
	and	r1, r3, r2
	cmp	r1, r2
	mrsne	r2, spsr
	strne	r2, [r5, #SleepState_ENTRY_SPSR]

	@ Step 2
	@ compare "modeSaveFlag" to decide which mode will be saved
	@ the private registers are saved in an array. the consequence should
	@ be "spsr", "r8-r12", sp, lr
	@ the data array stores registers from low address to high address.
	ldr	r6, [r5, #SleepState_modeSaveFlags]

1:
	@ save SYS mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_SYS)
	beq	2f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_SYS | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	add	r7, r5, #SleepState_SYS_REGS
	stmia	r7, {sp, lr}

2:
	@ save FIQ mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_FIQ)
	beq	3f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_FIQ | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	mrs	r2, spsr
	add	r7, r5, #SleepState_FIQ_REGS
	stmia	r7, {r2, r8 - r12, sp, lr}

3:
	@ save IRQ mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_IRQ)
	beq	4f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_IRQ | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	mrs	r2, spsr
	add	r7, r5, #SleepState_IRQ_REGS
	stmia	r7, {r2, sp, lr}

4:
	@ save ABT mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_ABT)
	beq	5f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_ABT | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	mrs	r2, spsr
	add	r7, r5, #SleepState_ABT_REGS
	stmia	r7, {r2, sp, lr}

5:
	@ save UND mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_UND)
	beq	6f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_UND | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	mrs	r2, spsr
	add	r7, r5, #SleepState_UND_REGS
	stmia	r7, {r2, sp, lr}

6:
	@ save SVC mode content?
	ands	r1, r6, #(PM_MODE_SAVE_FLAG_SVC)
	beq	7f
	bic	r3, r3, #(CPSR_Mode_MASK)
	orr	r3, r3, #(CPSR_Mode_SVC | CPSR_I_Bit | CPSR_F_Bit)
	msr	cpsr_c, r3
	mrs	r2, spsr
	add	r7, r5, #SleepState_SVC_REGS
	stmia	r7, {r2, sp, lr}

7:
	@ Step 3
	@ save MMU settings
	@ r5 is pointer to sleep save data area

	@ Cp15_ACR_MMU
	mrc	p15, 0, r0, c1, c0, 0
	str	r0, [r5, #SleepState_Cp15_ACR_MMU]

	@ Cp15_AUXCR_MMU;
	mrc	p15, 0, r0, c1, c0, 1
	str	r0, [r5, #SleepState_Cp15_AUXCR_MMU]

	@ Cp15_TTBR_MMU;
	mrc	p15, 0, r0, c2, c0, 0
	str	r0, [r5, #SleepState_Cp15_TTBR_MMU]

	@ Cp15_DACR_MMU;
	mrc	p15, 0, r0, c3, c0, 0
	str	r0, [r5, #SleepState_Cp15_DACR_MMU]

	@ Cp15_PID_MMU;
	mrc	p15, 0, r0, c13, c0, 0
	str	r0, [r5, #SleepState_Cp15_PID_MMU]

	@ Cp15_CPAR;
	mrc	p15, 0, r0, c15, c1, 0
	str	r0, [r5, #SleepState_Cp15_CPAR]

	@ Now enable access to all valid coprocessors
	mcr	p15, 0, r1, c15, c1, 0

	@ cp_wait
	mrc	p15, 0, r0, c2, c0, 0
	mov	r0, r0
	sub	pc, pc, #4

	@ Step 4
	@ The block 0 of nand flash should be copied to SRAM 0x5c014000
	@ The OS should save the resume back address and the content save area address
	@ load current pspr to r12 register
	ldr	r12, [r5, #SleepState_psprAddress]
	@ Store 0x5c014000 to PSPR
	ldr	r1, =0x5c014000
	str	r1, [r12]

	@ Step 5
	@ calculate checksum
	@ get total word count for ckecksum and should not include "checksum"
	mov	r1, #SleepState_size - 4
	ldr	r2, [r5, #SleepState_extendedChecksumByteCount]
	add	r1, r1, r2
	@ get the word count by /4
	mov	r1, r1, lsr #2
	mov	r0, r5
	str	r1, [r0, #SleepState_wordCount]!
	bl	pm_checksum_calculate
	str	r0, [r5, #SleepState_checksum]
	@ Step 6
	@ invoke user flush function
	ldr	r0, [r5, #SleepState_flushFunc]
	cmp	r0, #0
	movne	lr, pc
	movne	pc, r0

	b	1f
	.align 5
1:
	@ Step 7
	@ enter sleep or deep sleep
	mcr	p14, 0, r11, c7, c0, 0

	@ wait for sleep
20:
	nop
	b	20b


@*****************************************************************************
@ pm_enter_sleep
@
@ Put the system into S2D3C4 state
@
@ Inputs:
@	r0: the virutal address of the data area to save the content of core
@
@ Outputs:
@	None
@

pm_enter_sleep:
	str	r1, [r0, #SleepState_ENTRY_R1]
	mov	r1, #PXA3xx_PM_S2D3C4
	b	pm_enter_sleep_or_deep_sleep

@*****************************************************************************
@ pm_enter_deep_sleep
@
@ Put the system into S3D4C4 state
@
@ Inputs:
@	r0: the virutal address of the data area to save the content of core
@
@ Outputs:
@	None
@

pm_enter_deep_sleep:
	str	r1, [r0, #SleepState_ENTRY_R1]
	mov	r1, #PXA3xx_PM_S3D4C4
	b	pm_enter_sleep_or_deep_sleep


