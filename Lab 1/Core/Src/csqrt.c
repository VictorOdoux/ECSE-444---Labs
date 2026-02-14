/*
 * csqrt.c
 *
 * Square Root Implementations for Lab 1
 * Comparing FPU, CMSIS-DSP, and Newton-Raphson methods
 *
 */
#include <stdint.h>
#include <math.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "lab1math.h"

/*
 * ============================================================================
 * Method 1: Cortex-M4 FPU (Native Hardware Square Root)
 * ============================================================================
 * The Cortex-M4 FPU has a dedicated VSQRT.F32 instruction that computes
 * square root directly in hardware. This is the fastest method.
 *
 * We use inline assembly to directly invoke the VSQRT.F32 instruction.
 */
void sqrtFPU(float32_t x, float32_t *pResult) {
    // Use inline assembly to directly call the FPU's VSQRT instruction
    // VSQRT.F32 Sd, Sm - computes square root of Sm and stores in Sd
    __ASM volatile ("VSQRT.F32 %0, %1" : "=t"(*pResult) : "t"(x));
}

/*
 * ============================================================================
 * Method 3: Newton-Raphson Method (Iterative Software Implementation)
 * ============================================================================
 * Newton-Raphson is an iterative method for finding roots of equations.
 * To find sqrt(x), we solve f(y) = y^2 - x = 0
 *
 * Newton-Raphson formula: y_{n+1} = y_n - f(y_n) / f'(y_n)
 *
 * For sqrt: y_{n+1} = y_n - (y_n^2 - x) / (2*y_n)
 *                   = y_n - y_n/2 + x/(2*y_n)
 *                   = (y_n + x/y_n) / 2
 *
 * This is also known as Heron's method or the Babylonian method.
 *
 * Convergence: Newton-Raphson has quadratic convergence, meaning the number
 * of correct digits roughly doubles with each iteration.
 *
 * Initial guess: We use x/2 as a simple initial guess. Better initial
 * guesses can reduce iterations needed.
 */
#define NEWTON_ITERATIONS 10      // Number of iterations (10 gives good precision)
#define NEWTON_TOLERANCE  1e-7f   // Convergence tolerance (optional early exit)

void sqrtNewtonRaphson(float32_t x, float32_t *pResult) {
    // Handle edge cases
    if (x <= 0.0f) {
        *pResult = 0.0f;
        return;
    }

    // Initial guess: start with x/2 (simple but effective for most cases)
    // Alternative: could use bit manipulation for better initial guess
    float32_t guess = x * 0.5f;

    // Iterate using Newton-Raphson formula: y = (y + x/y) / 2
    for (int i = 0; i < NEWTON_ITERATIONS; i++) {
        // Newton-Raphson iteration
        guess = 0.5f * (guess + x / guess);

        // Optional: early termination if converged
        // This check adds overhead but can save iterations for some inputs
        // float32_t error = guess * guess - x;
        // if (error < NEWTON_TOLERANCE && error > -NEWTON_TOLERANCE) break;
    }

    *pResult = guess;
}
