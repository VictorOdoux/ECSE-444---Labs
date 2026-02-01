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
 */

extern void asmSqrt(const float *x, float *out);

void sqrtNewtonRaphson(float x, float *pResult);

#endif /* INC_LAB1MATH_H_ */
