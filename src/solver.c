#include "solver.h"
#include <string.h>

int rk4_step(DerivFn f, double t, const double state[STATE_DIM],
             double dt, double out[STATE_DIM])
{
    double k1[STATE_DIM], k2[STATE_DIM], k3[STATE_DIM], k4[STATE_DIM];
    double tmp[STATE_DIM];
    int i;

    // k1 = f(t, y)
    if (f(t, state, k1) < 0) return -1;

    // k2 = f(t + dt/2, y + dt/2 * k1)
    for (i = 0; i < STATE_DIM; i++) tmp[i] = state[i] + 0.5 * dt * k1[i];
    if (f(t + 0.5*dt, tmp, k2) < 0) return -1;

    // k3 = f(t + dt/2, y + dt/2 * k2)
    for (i = 0; i < STATE_DIM; i++) tmp[i] = state[i] + 0.5 * dt * k2[i];
    if (f(t + 0.5*dt, tmp, k3) < 0) return -1;

    // k4 = f(t + dt, y + dt * k3)
    for (i = 0; i < STATE_DIM; i++) tmp[i] = state[i] + dt * k3[i];
    if (f(t + dt, tmp, k4) < 0) return -1;

    // y_{n+1} = y_n + (dt/6) * (k1 + 2k2 + 2k3 + k4)
    for (i = 0; i < STATE_DIM; i++)
        out[i] = state[i] + (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);

    return 0;
}

int rk4_step_double_body(DerivFnDouble f, double t, const double state_obj1[STATE_DIM],const double state_obj2[STATE_DIM],
             double dt, double out_obj1[STATE_DIM],double out_obj2[STATE_DIM])
{
    double k1_1[STATE_DIM], k2_1[STATE_DIM], k3_1[STATE_DIM], k4_1[STATE_DIM]; // obj1
    double k1_2[STATE_DIM], k2_2[STATE_DIM], k3_2[STATE_DIM], k4_2[STATE_DIM]; // obj2
    double tmp1[STATE_DIM], tmp2[STATE_DIM];
    int i;
    // ── k1 ───────────────────────────────────────────────────
    if(f(t, state_obj1, state_obj2, k1_1, k1_2) <0) return -1;

    // ── k2 ───────────────────────────────────────────────────
    // Both tmp's built from k1 of their respective object
    for (i = 0; i < STATE_DIM; i++) tmp1[i] = state_obj1[i] + 0.5*dt*k1_1[i];
    for (i = 0; i < STATE_DIM; i++) tmp2[i] = state_obj2[i] + 0.5*dt*k1_2[i];
    if(f(t + 0.5*dt, tmp1, tmp2, k2_1, k2_2) <0) return -1;

    // ── k3 ───────────────────────────────────────────────────
    for (i = 0; i < STATE_DIM; i++) tmp1[i] = state_obj1[i] + 0.5*dt*k2_1[i];
    for (i = 0; i < STATE_DIM; i++) tmp2[i] = state_obj2[i] + 0.5*dt*k2_2[i];
    if(f(t + 0.5*dt, tmp1, tmp2, k3_1, k3_2) <0) return -1;

    // ── k4 ───────────────────────────────────────────────────
    for (i = 0; i < STATE_DIM; i++) tmp1[i] = state_obj1[i] + dt*k3_1[i];
    for (i = 0; i < STATE_DIM; i++) tmp2[i] = state_obj2[i] + dt*k3_2[i];
    if(f(t + dt, tmp1, tmp2, k4_1, k4_2) <0) return -1;

    // ── Final update ─────────────────────────────────────────
    for (i = 0; i < STATE_DIM; i++)
        out_obj1[i] = state_obj1[i] + (dt/6.0)*(k1_1[i] + 2*k2_1[i] + 2*k3_1[i] + k4_1[i]);
    for (i = 0; i < STATE_DIM; i++)
        out_obj2[i] = state_obj2[i] + (dt/6.0)*(k1_2[i] + 2*k2_2[i] + 2*k3_2[i] + k4_2[i]);

    return 0;
}