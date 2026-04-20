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

// Derivative function signature for double body
typedef int (*DerivFnDouble)(double t, const double state_obj1[STATE_DIM], 
                        const double state_obj2[STATE_DIM], double out1[STATE_DIM], double out2[STATE_DIM]);
// Single RK4 step.
// Advances `state` by timestep `dt` using derivative function `f`.
// Writes result into `out`.
// Returns 0 on success, -1 if any derivative evaluation signals a collision.
int rk4_step_double_body(DerivFnDouble f, double t, const double state_obj1[STATE_DIM],const double state_obj2[STATE_DIM],
             double dt, double out_obj1[STATE_DIM],double out_obj2[STATE_DIM]);


typedef int (*DerivFnNBody)(double t, const Planet states[N_BODY_MAX], int n, Planet out[N_BODY_MAX]);

int rk4_step_n_body(DerivFnNBody f, double t,int n, const Planet states[N_BODY_MAX],double dt, const Planet out[N_BODY_MAX]);

#endif // SOLVER_H