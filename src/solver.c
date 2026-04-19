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