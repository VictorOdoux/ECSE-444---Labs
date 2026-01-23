/*
 * transcendental_asm.s
 *
 * Newton-Raphson solver for x^2 = cos(omega*x + phi).
 * Hard-float ABI: omega, phi, x0, eps in s0-s3; maxIter in r0;
 * iters* in r1; status* in r2; return x in s0.
 */

.syntax unified
.thumb

.extern arm_cos_f32
.extern arm_sin_f32

.global nr_transcendental_asm
.type nr_transcendental_asm, %function

nr_transcendental_asm:
    push {r4-r8, lr}
    vpush {s16-s23}

    mov  r5, r0        // maxIter
    mov  r6, r1        // iters*
    mov  r7, r2        // status*

    vmov s16, s0       // omega
    vmov s17, s1       // phi
    vmov s20, s2       // x
    vmov s18, s3       // eps
    vldr s19, =1.0e-6  // derivative threshold

    movs r4, #0        // i = 0
    movs r8, #1        // status = NR_MAXITER

loop_check:
    cmp  r4, r5        
    bcs  done          // when i > maxIter do to the done section

    // theta = omega * x + phi
    // s23 will hold theta
    vmov.f32 s23, s17      // copy phi into s23
    vmla.f32 s23, s16, s20 // s23 = s23 + (s16 * s20)

    // s21 will hold cos(theta)
    vmov s0, s23
    bl   arm_cos_f32
    vmov s21, s0

    // s22 will hold sin(theta)
    vmov s0, s23
    bl   arm_sin_f32
    vmov s22, s0

    // f = x*x - cos(theta)
    vmul.f32 s4, s20, s20  // s4 holds x*x 
    vsub.f32 s5, s4, s21   // s5 holds f

    // f' = 2*x + omega*sin(theta)
    vadd.f32 s6, s20, s20
    vmla.f32 s6, s16, s22

    // if |f'| < 1e-6 -> derivative too small
    vabs.f32 s8, s6
    vcmp.f32 s8, s19
    vmrs APSR_nzcv, FPSCR
    blt  deriv_zero 

    // x_new = x - f / f'
    vdiv.f32 s7, s5, s6
    vsub.f32 s7, s20, s7

    // |x_new - x| < eps ?
    vsub.f32 s4, s7, s20
    vabs.f32 s4, s4
    vcmp.f32 s4, s18
    vmrs APSR_nzcv, FPSCR
    blt  converged

    // |f| < eps ?
    vabs.f32 s5, s5
    vcmp.f32 s5, s18
    vmrs APSR_nzcv, FPSCR
    blt  converged

    // continue
    vmov s20, s7
    adds r4, r4, #1
    b    loop_check

deriv_zero:
    movs r8, #2        // NR_DERIVZERO
    adds r4, r4, #1
    b    done

converged:
    vmov s20, s7
    movs r8, #0        // NR_OK
    adds r4, r4, #1
    b    done

done:
    // store *iters if non-null
    cmp  r6, #0
    beq  skip_iters
    str  r4, [r6]
skip_iters:
    // store *status if non-null
    cmp  r7, #0
    beq  skip_status
    str  r8, [r7]
skip_status:
    vmov s0, s20       // return x
    vpop {s16-s23}
    pop  {r4-r8, pc}

    .ltorg
