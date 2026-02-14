/*
 * transcendental.c
 *
 * Newton-Raphson solver for x^2 = cos(omega*x + phi).
 */

#include "transcendental.h"

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#include "arm_math.h"

#include <math.h>
#include <stddef.h>

float nr_transcendental_c(float omega, float phi, float x0,
                          uint32_t maxIter, float eps,
                          uint32_t *iters, NRStatus *status) {
  const float deriv_eps = 1.0e-6f;
  float x = x0;
  uint32_t i = 0;
  NRStatus st = NR_MAXITER;

  // Newton-Raphson update: x_{n+1} = x_n - f(x_n)/f'(x_n)
  for (i = 0; i < maxIter; ++i) {
    float theta = omega * x + phi;
    float c = arm_cos_f32(theta);
    float s = arm_sin_f32(theta);
    // f(x) = x^2 - cos(theta)
    float f = (x * x) - c;
    // f'(x) = 2x + omega*sin(theta)
    float fp = (2.0f * x) + (omega * s);

    // Guard against division by a near-zero derivative.
    // fabsf returns the absolute value of a float
    if (fabsf(fp) < deriv_eps) {
      st = NR_DERIVZERO;
      ++i;
      break;
    }

    float x_new = x - (f / fp);
    // stop if 
    // |x_new - x| < eps : the update step is tiny, so the estimate isn’t changing meaningfully anymore.
    // or 
    // |f| < eps : the current estimate already satisfies the equation to within the desired tolerance.
    if (fabsf(x_new - x) < eps || fabsf(f) < eps) {
      x = x_new;
      st = NR_OK;
      ++i;
      break;
    }

    x = x_new;
  }

  if (iters != NULL) {
    // Iterations actually executed (1-based on early exit).
    *iters = i;
  }
  if (status != NULL) {
    *status = st;
  }

  return x;
}
