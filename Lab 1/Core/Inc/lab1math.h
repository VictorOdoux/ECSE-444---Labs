/*
 * lab1math.h
 *
 *  Created on: Jan 20, 2026
 *      Author: victor
 */

#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_
#include "main.h"

/* ============================================================================
 * Max Functions (from previous lab parts)
 * ============================================================================ */
void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

/* ============================================================================
 * Square Root Functions
 * ============================================================================
 * Three implementations for performance comparison:
 * 1. sqrtFPU          - Uses Cortex-M4 FPU VSQRT.F32 instruction (fastest)
 * 2. arm_sqrt_f32     - CMSIS-DSP square root (called directly in main)
 * 3. sqrtNewtonRaphson - Software implementation using Newton-Raphson method
 * ============================================================================ */

// Method 1: Cortex-M4 FPU hardware square root (VSQRT.F32 instruction)
void sqrtFPU(float x, float *pResult);

// Method 3: Newton-Raphson iterative method (software)
void sqrtNewtonRaphson(float x, float *pResult);

#endif /* INC_LAB1MATH_H_ */
