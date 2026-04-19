#ifndef PHYSICS_H
#define PHYSICS_H

// State vector layout: [x, y, vx, vy]  (SI units: metres, m/s)
#define STATE_DIM 4

// Compute derivatives of the state vector under Newtonian gravity.
// out[0] = vx,   out[1] = vy
// out[2] = ax,   out[3] = ay
// Returns 0 on success, -1 if satellite has hit Earth.
int gravity_derivatives(double t, const double state[STATE_DIM],
                        double out[STATE_DIM]);


// gravity derivative for two moving objects, two calculations are made
int gravity_derivatives_double_body(double t, const double state_obj1[STATE_DIM], 
                        const double state_obj2[STATE_DIM], double out1[STATE_DIM], double out2[STATE_DIM]);

// Specific orbital energy (J/kg):  E = 0.5*v² - GM/r
double specific_energy(const double state[STATE_DIM]);

// Specific angular momentum (m²/s):  h = x*vy - y*vx
double specific_angular_momentum(const double state[STATE_DIM]);

// Orbital eccentricity magnitude derived from energy and angular momentum.
// e < 1  → elliptical (e ≈ 0 → circular)
// e = 1  → parabolic
// e > 1  → hyperbolic
double eccentricity(const double state[STATE_DIM]);

// Distance from Earth centre (m)
double orbital_radius(const double state[STATE_DIM]);

// Orbital speed (m/s)
double orbital_speed(const double state[STATE_DIM]);

// Classify orbit as a string label
const char *orbit_type(const double state[STATE_DIM]);

#endif // PHYSICS_H