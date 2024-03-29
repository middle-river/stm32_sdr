/*
  Signal processing library by assembly language.
  2020-11-03  T. Nakagawa
*/

	.altmacro
	.text
	.align 1
	.p2align 2,,3
	.syntax unified
	.thumb

// int32_t offset2(int32_t ofst, int size, const int16_t *inp, int16_t *out)
// r0=sum, r1=size, r2=inp, r3=out, r4-11=work, r12=bias, r14=0x00010001
.macro	m_off0		reg end
	smlad		r0, r\reg, r14, r0
	ssub16		r\reg, r\reg, r12
	.if		\reg != \end
	m_off0		%(reg + 1) \end
	.endif
.endm
.global	offset2
.type	offset2, %function
.thumb_func
offset2:
	push		{r4-r11, lr}
	asr		r1, r1, #4		@ size /= 16
	asr		r12, r0, #16		@ bias = ofst >> 16
	orr		r12, r12, r12, lsl #16	@ bias = bias | (bias << 16)
	mov		r0, #0			@ sum = 0
	mov		r14, #65537		@ constant (0x00010001)
1:
	@ Process 16 elements.
	ldmia		r2!, {r4-r11}
	m_off0		4 11
	stmia		r3!, {r4-r11}
	subs		r1, r1, #1
	bne		1b
	pop		{r4-r11, pc}

// void decimator_cic2(const int16_t *tbl, int32_t *state, int size, const int16_t *inp, float *out)
// r0=tbl, r1=inp, r2-r6=work, r7-r9=intg, r10-r12=comb, r14=out, s0=state, s1=size, s2=tmp, s3=1/2^30
.macro	m_cic0		inst x y
	\inst		r6, r\x, r\y
	add		r7, r7, r6, asr #11
	add		r8, r8, r7
	add		r9, r9, r8
.endm
.global	decimator_cic2
.type	decimator_cic2, %function
.thumb_func
decimator_cic2:
	push		{r4-r11, lr}
	ldr		r14, [sp, #36]
	ldmia		r1, {r7-r12}		@ Load intg and comb.
	vmov		s0, r1
	ldr		r1, d_cic0
	vmov		s3, r1
	mov		r1, r3
	asr		r2, r2, #5		@ size /= 32
1:
	vmov		s1, r2
	.rept		8
	@ Process 4 samples.
	ldmia		r0!, {r2-r3}
	ldmia		r1!, {r4-r5}
	m_cic0		smulbb 2 4
	m_cic0		smultt 2 4
	m_cic0		smulbb 3 5
	m_cic0		smultt 3 5
	.endr
	@ 1/4 decimation.
	sub		r2, r9, r10
	mov		r10, r9
	sub		r3, r2, r11
	mov		r11, r2
	sub		r4, r3, r12
	mov		r12, r3
	vmov		s2, r4
	vcvt.f32.s32	s2, s2
	vdiv.f32	s2, s2, s3
	vstmia		r14!, {s2}
	vmov		r2, s1
	subs		r2, #1
	bne		1b
	vmov		r1, s0
	stmia		r1, {r7-r12}		@ Store intg and comb.
	pop		{r4-r11, pc}
d_cic0:
	.float		9.313225746154785e-10	@ 1.0 / (1 << 30)

// void decimator_fir2(const float *coeff, float *state, int size, const float *inp, float *out)
// r0=coeff, r1=state, r2=size, r3=inp, r4-r6=bak, r14=out, s0-s31=work
.macro	m_fir1		cnt end acc coe sum top btm flt
	.if		\cnt == 0		@ The top/bottom/second-top elements in the filter stack are preserved.
	vadd.f32	s\acc, s\top, s\btm
	vldr		s\coe, [r0, #\flt]
	vmul.f32	s\acc, s\coe, s\acc
	.else
	vadd.f32	s\sum, s\top, s\btm
	vldr		s\coe, [r0, #\flt]
	vfma.f32	s\acc, s\coe, s\sum
	.endif
	.if		\cnt != \end
	m_fir1		%(\cnt + 1), \end, \acc, \coe, \sum, %((\top + 31) % 32), %((\btm + 1) % 32), %(\flt + 4)
	.endif
.endm
.macro	m_fir0		cnt end bas top btm stp
	vldmia		r3!, {s\bas-s\top}
	vmov		r4, s\top		@ Preserve the top/bottom/second-top elements which are used as acc/coe/sum.
	vmov		r5, s\btm
	vmov		r6, s\stp
	m_fir1		0, 15, \top, \btm, \stp, \top, \btm, 0	@ s\top <- FIR result.
	vstmia		r14!, {s\top}
	vmov		s\top, r4		@ Restore the top/bottom/second-top elements.
	vmov		s\btm, r5
	vmov		s\stp, r6
	.if		\cnt != \end
	m_fir0		%(\cnt + 1), \end, %(\bas + 4), %(\top + 4), %((\btm + 4) % 32), %(\stp + 4)
	.endif
.endm
.global		decimator_fir2
.type		decimator_fir2, %function
.thumb_func
decimator_fir2:
	push		{r4-r6, lr}
	ldr		r14, [sp, #20]
	vpush		{s16-s31}
	asr		r2, r2, #5		@ size /= 32
	vldmia		r1, {s0-s31}		@ Load the filter states.
1:
	m_fir0		0, 7, 0, 3, 4, 2	@ Process 32 samples (1/4 decimation x 8)
	subs		r2, #1
	bne		1b
	vstmia		r1, {s0-s31}		@ Save the filter states.
	vpop		{s16-s31}
	pop		{r4-r6, pc}

// void bpf_iir2(const float *coeff, float *state, int size, const float *inp, float *out)
// r0=coeff, r1=state, r2=size, r3=inp, r14=out, s0-s15=buffer, s16-21=coeff_values, s22-s27=state_values, s28=acc
.macro	m_bpf1		cnt x y
	vmov		s28, s\cnt
	vfms.f32	s28, s20, s\x
	vfms.f32	s28, s21, s\y
	vmul.f32	s\cnt, s16, s28
	vfma.f32	s\cnt, s17, s\x
	vfma.f32	s\cnt, s18, s\y
	vmov		s\y, s\x
	vmov		s\x, s28
.endm
.macro	m_bpf0		cnt end
	m_bpf1		\cnt, 22, 23		@ 1st stage.
	m_bpf1		\cnt, 24, 25		@ 2nd stage.
	m_bpf1		\cnt, 26, 27		@ 3rd stage.
	.if		\cnt != \end
	m_bpf0		%(\cnt + 1), \end
	.endif
.endm
.global	bpf_iir2
.type	bpf_iir2, %function
.thumb_func
bpf_iir2:
	push		{lr}
	ldr		r14, [sp, #4]
	vpush		{s16-s28}
	vldmia		r0, {s16-s21}
	vldmia		r1, {s22-s27}		@ Load the filter states.
	asr		r2, r2, #4		@ size /= 16
1:
	vldmia		r3!, {s0-s15}
	m_bpf0		0 15
	vstmia		r14!, {s0-s15}
	subs		r2, #1
	bne		1b
	vstmia		r1, {s22-s27}		@ Save the filter states.
	vpop		{s16-s28}
	pop		{pc}

// float demodulator_amplitude2(float alpha, float state, int size, const float *inp0, const float *inp1, float *out)
// r0=inp1, r1=out, r2=size, r3=inp0, s0-s15=buffer, s16=1-alpha, s17=state, s18=tmp
.macro	m_demamp0	cnt end reg
	vmul.f32	s\cnt, s\cnt, s\cnt
	vfma.f32	s\cnt, s\reg, s\reg
	vsqrt.f32	s\cnt, s\cnt
	vmul.f32	s18, s16, s17
	vadd.f32	s\cnt, s\cnt, s18
	vadd.f32	s17, s17, s\cnt
	.if		\cnt != \end
	m_demamp0	%(\cnt + 1), \end, %(\reg + 1)
	.endif
.endm
.global	demodulator_amplitude2
.type	demodulator_amplitude2, %function
.thumb_func
demodulator_amplitude2:
	push		{lr}
	vpush		{s16-s18}
	vmov		s17, r1
	movw		r1, #1
	vmov		s16, r1
	vcvt.f32.s32	s16, s16
	vmov		s0, r0
	vsub.f32	s16, s16, s0
	ldr		r0, [sp, #12]
	ldr		r1, [sp, #14]
	asr		r2, r2, #3		@ size /= 8
1:
	vldmia		r3!, {s0-s7}
	vldmia		r0!, {s8-s15}
	m_demamp0	0, 7, 8
	vstmia		r1!, {s0-s7}
	subs		r2, #1
	bne		1b
	vmov		r0, s17
	vpop		{s16-s18}
	pop		{pc}

// void demodulator_mixer2(const float *tbl0, const float *tbl1, int size, const float *inp0, const float *inp1, float *out)
// r0=tbl0, r1=tbl1, r2=size, r3=inp0, r4=inp1, r14=out, s0-s31=buffer
.macro	m_demmix0	cnt end x y z
	vmul.f32	s\cnt, s\cnt, s\x
	vfms.f32	s\cnt, s\y, s\z
	vadd.f32	s\cnt, s\cnt, s\cnt
	.if		\cnt != \end
	m_demmix0	%(\cnt + 1), \end, %(\x + 1), %(\y + 1), %(\z + 1)
	.endif
.endm
.global	demodulator_mixer2
.type	demodulator_mixer2, %function
.thumb_func
demodulator_mixer2:
	push		{r4, lr}
	vpush		{s16-s31}
	ldr		r4, [sp, #72]
	ldr		r14, [sp, #76]
	asr		r2, r2, #3		@ size /= 8
1:
	vldmia		r0!, {s0-s7}
	vldmia		r3!, {s8-s15}
	vldmia		r1!, {s16-s23}
	vldmia		r4!, {s24-s31}
	m_demmix0	0, 7, 8, 16, 24
	vstmia		r14!, {s0-s7}
	subs		r2, #1
	bne		1b
	vpop		{s16-s31}
	pop		{r4, pc}

// void demodulator_hilbert2(const float *coeff, float *state, int size, const float *inp0, const float *inp1, float *out)
// r0=coeff, r1=state, r2=size, r3=inp0, r4=inp1, r5-r6=bak, r14=out, s0-s31=state_values
.macro	m_demhlb2	fst cnt res off
	.if		\fst != 0 && \cnt < 14
	vldr		s\res, [r1, #\off]
	.else
	vldmia		r4!, {s\res}
	.endif
.endm
.macro	m_demhlb1	cnt end acc coe sum top btm flt
	vsub.f32	s\sum, s\top, s\btm
	vldr		s\coe, [r0, #\flt]
	.if		\cnt == 0
	vmul.f32	s\acc, s\coe, s\sum
	.else
	vfma.f32	s\acc, s\coe, s\sum
	.endif
	.if		\cnt != \end
	m_demhlb1	%(\cnt + 1), \end, \acc, \coe, \sum, %((\top + 31) % 32), %((\btm + 1) % 32), %(\flt + 4)
	.endif
.endm
.macro	m_demhlb0	fst cnt end top btm tmp
	vldmia		r3!, {s\top}		@ Input an I-signal.
	vmov		r5, s\top		@ Preserve the top/bottom element which is used as acc/coe.
	vmov		r6, s\btm
	m_demhlb1	0, 14, \top, \btm, %((\top + 1) % 32), \top, \btm, 0	@ s\top <- result of FIR.
	m_demhlb2	\fst, \cnt, \btm, %((32 + \cnt) * 4)	@ \btm <- delayed Q-signal.
	vsub.f32	s\top, s\top, s\btm
	vstmia		r14!, {s\top}
	vmov		s\top, r5		@ Restore the top/botoom element.
	vmov		s\btm, r6
	.if		\cnt != \end
	m_demhlb0	\fst, %(\cnt + 1), \end, %(\top + 1), %((\btm + 1) % 32)
	.endif
.endm
.global	demodulator_hilbert2
.type	demodulator_hilbert2, %function
.thumb_func
demodulator_hilbert2:
	push		{r4-r6, lr}
	ldr		r4, [sp, #16]
	ldr		r14, [sp, #20]
	vpush		{s16-s31}
	asr		r2, r2, #5		@ size /= 32
	sub		r2, r2, #1		@ size -= 1
	vldmia		r1, {s0-s31}		@ Load the filter states.
	m_demhlb0	1, 0, 31, 0, 2, 1	@ Process the first 32 examples.
1:
	m_demhlb0	0, 0, 31, 0, 2, 1	@ Process 32 examples.
	subs		r2, #1
	bne		1b
	vstmia		r1!, {s0-s31}		@ Save the filter states.
	vldmia		r4!, {s0-s13}		@ Save the delay buffer.
	vstmia		r1!, {s0-s13}
	vpop		{s16-s31}
	pop		{r4-r6, pc}
