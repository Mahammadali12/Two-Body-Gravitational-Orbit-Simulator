    #ifndef SOLVER_H
#define SOLVER_H

#include "physics.h"   // STATE_DIM

// Derivative function signature
typedef int (*DerivFn)(double t, const double state[STATE_DIM],
                       double out[STATE_DIM]);

// Single RK4 step.
// Advances `state` by timestep `dt` using derivative function `f`.
// Writes result into `out`.
// Returns 0 on success, -1 if any derivative evaluation signals a collision.
int rk4_step(DerivFn f, double t, const double state[STATE_DIM],
             double dt, double out[STATE_DIM]);

#endif // SOLVER_H