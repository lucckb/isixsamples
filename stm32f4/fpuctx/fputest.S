/*
 * fputest.S
 *
 *  Created on: 3 gru 2013
 *      Author: lucck
 */

// -------------------------------------------------

.text

// -------------------------------------------------
.global fputest_fill_and_add
fputest_fill_and_add:
	vldr s0, fone
	vmov s1, r0
	vcvt.f32.s32 s1, s1
	vadd.f32  s1,s1,s0
	vadd.f32  s2,s1,s0
	vadd.f32  s3,s2,s0
	vadd.f32  s4,s3,s0
	vadd.f32  s5,s4,s0
	vadd.f32  s6,s5,s0
	vadd.f32  s7,s6,s0
	vadd.f32  s8,s7,s0
	vadd.f32  s9,s8,s0
	vadd.f32  s10,s9,s0
	vadd.f32  s11,s10,s0
	vadd.f32  s12,s11,s0
	vadd.f32  s13,s12,s0
	vadd.f32  s14,s13,s0
	vadd.f32  s15,s14,s0
	vadd.f32  s16,s15,s0
	vadd.f32  s17,s16,s0
	vadd.f32  s18,s17,s0
	vadd.f32  s19,s18,s0
	vadd.f32  s20,s19,s0
	vadd.f32  s17,s16,s0
	vadd.f32  s18,s17,s0
	vadd.f32  s19,s18,s0
	vadd.f32  s20,s19,s0
	vadd.f32  s21,s20,s0
	vadd.f32  s22,s21,s0
	vadd.f32  s23,s22,s0
	vadd.f32  s24,s23,s0
	vadd.f32  s25,s24,s0
	vadd.f32  s26,s25,s0
	vadd.f32  s27,s26,s0
	vadd.f32  s28,s27,s0
	vadd.f32  s29,s28,s0
	vadd.f32  s30,s29,s0
	vadd.f32  s31,s30,s0
	vadd.f32  s0,s31,s0
	bx lr
// -------------------------------------------------
.global fputest_fill_and_add_check
fputest_fill_and_add_check:
	bx lr

// -------------------------------------------------
.align 4
fone: .float 1.0


.end

