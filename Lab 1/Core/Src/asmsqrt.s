/*
 * asmsqrt.s
 *
 *  Created on: Jan 23, 2026
 *      Author: victor
 */

.syntax unified
.thumb

.global asmSqrt
.thumb_func
.type asmSqrt, %function

// void asmSqrt(const float *x, float *out)
// R0 = &x, R1 = out
asmSqrt:
    // out == NULL -> just return
    cbz     r1, .ret

    // x == NULL -> write 0.0
    cbz     r0, .write_zero

    // Load input: s0 = *x
    vldr.f32 s0, [r0]

    // If x < 0 => out = 0.0
    vcmpe.f32 s0, #0.0
    vmrs    apsr_nzcv, fpscr
    bmi     .write_zero

    // Hardware sqrt
    vsqrt.f32 s1, s0
    vstr.f32 s1, [r1]
    bx      lr

.write_zero:
    movs    r2, #0      // r2 = 0
    vmov    s1, r2      // s1 = 0.0f
    vstr.f32 s1, [r1]   // *out = 0.0f
    bx      lr

.ret:
    bx      lr

.size asmSqrt, .-asmSqrt


