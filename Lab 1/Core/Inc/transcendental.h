#ifndef INC_TRANSCENDENTAL_H_
#define INC_TRANSCENDENTAL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  NR_OK = 0,
  NR_MAXITER = 1,
  NR_DERIVZERO = 2
} NRStatus;

// Inputs:
// - omega: angular coefficient in cos(omega*x + phi)
// - phi: phase offset in cos(omega*x + phi)
// - x0: initial guess for the root
// - maxIter: maximum Newton iterations to run
// - eps: convergence tolerance for |x_{n+1}-x_n| or |f(x)|
// - iters: optional output pointer for iterations executed (may be NULL)
// - status: optional output pointer for NRStatus (may be NULL)

float nr_transcendental_c(float omega, float phi, float x0,
                          uint32_t maxIter, float eps,
                          uint32_t *iters, NRStatus *status);

float nr_transcendental_asm(float omega, float phi, float x0,
                            uint32_t maxIter, float eps,
                            uint32_t *iters, NRStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* INC_TRANSCENDENTAL_H_ */
