#include "physics.h"
#include "../config.h"
#include <math.h>
#include <string.h>

int gravity_derivatives(double t, const double state[STATE_DIM],
                        double out[STATE_DIM])
{
    (void)t; // autonomous system — time unused

    double x  = state[0];
    double y  = state[1];
    double vx = state[2];
    double vy = state[3];

    double r  = sqrt(x*x + y*y);

    if (r < R_EARTH) return -1;     // collision with Earth

    double r3 = r * r * r;
    double a  = -GM / r3;           // scalar: -GM/r³

    out[0] = vx;                    // dx/dt  = vx
    out[1] = vy;                    // dy/dt  = vy
    out[2] = a * x;                 // dvx/dt = ax
    out[3] = a * y;                 // dvy/dt = ay

    return 0;
}

double orbital_radius(const double state[STATE_DIM])
{
    double x = state[0], y = state[1];
    return sqrt(x*x + y*y);
}

double orbital_speed(const double state[STATE_DIM])
{
    double vx = state[2], vy = state[3];
    return sqrt(vx*vx + vy*vy);
}

double specific_energy(const double state[STATE_DIM])
{
    double v = orbital_speed(state);
    double r = orbital_radius(state);
    return 0.5 * v*v - GM / r;     // kinetic + potential (per unit mass)
}

double specific_angular_momentum(const double state[STATE_DIM])
{
    // h = r × v  (z-component in 2D)
    return state[0] * state[3] - state[1] * state[2];  // x*vy - y*vx
}

double eccentricity(const double state[STATE_DIM])
{
    double eps = specific_energy(state);
    double h   = specific_angular_momentum(state);
    double val = 1.0 + 2.0 * eps * h*h / (GM * GM);
    if (val < 0.0) val = 0.0;      // clamp numerical noise for nearly-circular orbits
    return sqrt(val);
}

const char *orbit_type(const double state[STATE_DIM])
{
    double e = eccentricity(state);
    double eps = specific_energy(state);

    if (eps >= 0.0)          return "Hyperbolic (Escape)";
    if (e < 0.01)            return "Circular";
    if (e < 0.5)             return "Elliptical (low e)";
    return                          "Elliptical (high e)";
}