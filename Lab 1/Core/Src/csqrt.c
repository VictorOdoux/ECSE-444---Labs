/*
 * csqrt.c
 *
 *  Created on: Jan 22, 2026
 *      Author: victo
 */
#include <stdint.h>
#include <math.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "lab1math.h"

/* Newton-Raphson is an iterative method for finding roots of equations.
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

    }

    *pResult = guess;
}
