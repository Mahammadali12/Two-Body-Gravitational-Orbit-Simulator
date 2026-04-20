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


int gravity_derivatives_double_body(double t, const double state_obj1[STATE_DIM], 
                        const double state_obj2[STATE_DIM], double out1[STATE_DIM], double out2[STATE_DIM])
{
    (void)t; // autonomous system — time unused

    double x1  = state_obj1[0];
    double y1  = state_obj1[1];
    double vx1 = state_obj1[2];
    double vy1 = state_obj1[3];

    double x2  = state_obj2[0];
    double y2  = state_obj2[1];
    double vx2 = state_obj2[2];
    double vy2 = state_obj2[3];


    double dx = x2 - x1;
    double dy = y2 - y1;

    // double r  = sqrt(x*x + y*y);
    double distance = sqrt(dx*dx + dy*dy);

    if (distance < R_OBJ1 + R_OBJ2) return -1;     // collision with each other
    // if()

    double distance3 = distance*distance*distance;
    
    // double a  = -GM / r3;           // scalar: -GM/r³
    double a1  =  GM2 / distance3; // the acceleration of obj1
    double a2  =  GM1 / distance3; // the acceleration of obj2



    
    out1[0] = vx1;
    out1[1] = vy1;
    out1[2] = a1 * dx;
    out1[3] = a1 * dy;


    out2[0] = vx2;
    out2[1] = vy2;
    out2[2] = a2 * (-dx);
    out2[3] = a2 * (-dy);   
    
    return 0;
    
}


// Returns -1 if any collision detected
int gravity_derivatives_n_body(double t, const Planet states[N_BODY_MAX], int n, Planet out[N_BODY_MAX])
{

        (void)t; // autonomous system — time unused
    for (int i = 0; i < n; i++)
    {
        out[i].x = states[i].vx;
        out[i].y = states[i].vy;
        out[i].vx = 0;
        out[i].vy = 0;
        
        for (int j = 0; j < n; j++)
        {
            if(j==i) continue;
            double dx = states[j].x - states[i].x;
            double dy = states[j].y - states[i].y;
            double dist  = sqrt(dx*dx + dy*dy);

            // if(dist < states[i].radius + states[j].radius) return -1;

            double dist3 = dist*dist*dist;

            
            out[i].vx += G * states[j].mass * dx / dist3;
            out[i].vy += G * states[j].mass * dy / dist3;
        }   
    }
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