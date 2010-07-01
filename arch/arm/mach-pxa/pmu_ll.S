@ "This software program is available to you under a choice of one of two
@ licenses.  You may choose to be licensed under either the GNU General Public
@ License (GPL) Version 2, June 1991, available at
@ http://www.fsf.org/copyleft/gpl.html, or the BSD License, the text of
@ which follows:
@
@ Copyright (c) 1996-2005, Intel Corporation. All rights reserved.
@
@ Redistribution and use in source and binary forms, with or without
@ modification, are permitted provided that the following conditions are met:
@
@ Redistributions of source code must retain the above copyright notice, this
@ list of conditions and the following disclaimer.
@
@ Redistributions in binary form must reproduce the above copyright notice, this
@ list of conditions and the following disclaimer in the documentation and/or
@ other materials provided with the distribution.
@
@ Neither the name of the Intel Corporation ("Intel") nor the names of its
@ contributors may be used to endorse or promote products derived from this
@ software without specific prior written permission.
@
@ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
@ AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
@ IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
@ DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
@ FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
@ DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
@ SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
@ CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
@ OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
@
@  FILENAME:       pmu_ll.S
@
@  PURPOSE:        Provides low level PMU primitive functions written specifically for
@                  the Bulverde/Mainstone processor/platform.  Specially design to fit
@                  into Intel VTUNE Architecture
@
@
@  LAST MODIFIED:  10/31/02
@******************************************************************************
@
@
@ List of primitive functions in this source code include:
@
	.global pmu_read_reg
	.global pmu_write_reg

	.text

@
@ pmu_read_reg - Read the PMU Register
@
@ Description:
@	This routine reads the designated PMU register via CoProcesser 14.
@
@ Input Parameters:
@	r0 - arg1, PMU register number to read.  Number between 0 to 8
@	if r0 contains:
@		0 -> PMNC,	PMU Control Register
@		1 -> CCNT,	PMU Clock Counter
@		2 -> PMN0,	PMU Count Register 0
@		3 -> PMN1,	PMU Count Register 1
@		4 -> PMN2,	PMU Count Register 2
@		5 -> PMN3,	PMU Count Register 3
@		6 -> INTEN,	PMU Interupt Enable Register
@		7 -> FLAG,	PMU Overflow Flag Status Register
@		8 -> EVTSEL	PMU Event Select Register
@
@ Returns:
@	r0 - 32-bit value read from CoProcessor
@
@ Registers Modified:
@	CoProcessor Register Modified: None
@	General Purpose Registers Modified: r0
@
@ NOTE:
@	Currently not support THUMB mode
@	Error checking not included

pmu_read_reg:

	cmp	r0, #8
	addls	pc, pc, r0, lsl #2
	b	RRet
	b	RdPMNC
	b	RdCCNT
	b	RdPMN0
	b	RdPMN1
	b	RdPMN2
	b	RdPMN3
	b	RdINTEN
	b	RdFLAG
	b	RdEVTSEL

RdPMNC:
	mrc	p14, 0, r0, c0, c1, 0	@ Read PMNC
	b	RRet
RdCCNT:
	mrc	p14, 0, r0, c1, c1, 0	@ Read CCNT
	b	RRet
RdPMN0:
	mrc	p14, 0, r0, c0, c2, 0	@ Read PMN0
	b	RRet
RdPMN1:
	mrc	p14, 0, r0, c1, c2, 0	@ Read PMN1
	b	RRet
RdPMN2:
	mrc	p14, 0, r0, c2, c2, 0	@ Read PMN2
	b	RRet
RdPMN3:
	mrc	p14, 0, r0, c3, c2, 0	@ Read PMN3
	b	RRet
RdINTEN:
	mrc	p14, 0, r0, c4, c1, 0	@ Read INTEN
	b	RRet
RdFLAG:
	mrc	p14, 0, r0, c5, c1, 0	@ Read FLAG
	b	RRet
RdEVTSEL:
	mrc	p14, 0, r0, c8, c1, 0	@ Read EVTSEL

RRet:
	mov	pc, lr			@ return


@
@ pmu_write_reg - Writes to the PMU Register
@
@ Description:
@	This routine writes to the designated PMU register via CoProcesser 14.
@
@ Input Parameters:
@	r0 - arg1 - PMU register number to write
@	r1 - arg2 - Value to write to PMU register
@
@	if r0 contains:
@		0 -> PMNC,	PMU Control Register
@		1 -> CCNT,	PMU Clock Counter
@		2 -> PMN0,	PMU Count Register 0
@		3 -> PMN1,	PMU Count Register 1
@		4 -> PMN2,	PMU Count Register 2
@		5 -> PMN3,	PMU Count Register 3
@		6 -> INTEN,	PMU Interupt Enable Register
@		7 -> FLAG,	PMU Overflow Flag Status Register
@		8 -> EVTSEL	PMU Event Select Register
@
@ Returns:
@	None
@
@ Registers Modified:
@	CoProcessor Register Modified: PMU Register
@	General Purpose Registers Modified: None
@
@NOTE:
@	Currently not support THUMB mode
@	Error checking not included

pmu_write_reg:

	cmp	r0, #8
	addls	pc, pc, r0, lsl #2
	b	WRet
	b	WrPMNC
	b	WrCCNT
	b	WrPMN0
	b	WrPMN1
	b	WrPMN2
	b	WrPMN3
	b	WrINTEN
	b	WrFLAG
	b	WrEVTSEL

WrPMNC:
	mcr	p14, 0, r1, c0, c1, 0	@ Write PMNC
	b	WRet
WrCCNT:
	mcr	p14, 0, r1, c1, c1, 0	@ Write CCNT
	b	WRet
WrPMN0:
	mcr	p14, 0, r1, c0, c2, 0	@ Write PMN0
	b	WRet
WrPMN1:
	mcr	p14, 0, r1, c1, c2, 0	@ Write PMN1
	b	WRet
WrPMN2:
	mcr	p14, 0, r1, c2, c2, 0	@ Write PMN2
	b	WRet
WrPMN3:
	mcr	p14, 0, r1, c3, c2, 0	@ Write PMN3
	b	WRet
WrINTEN:
	mcr	p14, 0, r1, c4, c1, 0	@ Write INTEN
	b	WRet
WrFLAG:
	mcr	p14, 0, r1, c5, c1, 0	@ Write FLAG
	b	WRet
WrEVTSEL:
	mcr	p14, 0, r1, c8, c1, 0	@ Write EVTSEL

WRet:
	mov	pc, lr			@ return

