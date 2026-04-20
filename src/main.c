#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>

#include "../config.h"
#include "physics.h"
#include "solver.h"
#include "trail.h"

// ── Unit conversion ───────────────────────────────────────────────────────────

static inline float to_px(double metres) { return (float)(metres / SCALE); }

static inline Vector2 world_to_screen(double x, double y)
{
    return (Vector2){
        SCREEN_W * 0.5f + to_px(x),
        SCREEN_H * 0.5f - to_px(y)
    };
}

// Multi-body: origin follows centre of mass, uses TWO_BODY_SCALE
static inline Vector2 world_to_screen_tb(double x, double y,
                                          double cm_x, double cm_y)
{
    return (Vector2){
        SCREEN_W * 0.5f + (float)((x - cm_x) / TWO_BODY_SCALE),
        SCREEN_H * 0.5f - (float)((y - cm_y) / TWO_BODY_SCALE)
    };
}

// ── Helpers ───────────────────────────────────────────────────────────────────

// Compute centre of mass of a Planet array
static void compute_cm(const Planet states[], int n,
                        double *cm_x, double *cm_y)
{
    double total_mass = 0.0;
    *cm_x = 0.0;
    *cm_y = 0.0;
    for (int i = 0; i < n; i++) {
        *cm_x      += states[i].mass * states[i].x;
        *cm_y      += states[i].mass * states[i].y;
        total_mass += states[i].mass;
    }
    *cm_x /= total_mass;
    *cm_y /= total_mass;
}

// ── Single-body preset loader ─────────────────────────────────────────────────

static void load_preset(int idx, double state[STATE_DIM])
{
    double r, vc, ve;
    switch (idx) {
    case 0:
        r = PRESET_LEO_R;   vc = sqrt(GM / r);
        state[0] = r;   state[1] = 0.0;
        state[2] = 0.0; state[3] = vc;
        break;
    case 1:
        r = PRESET_ELLIP_R; vc = sqrt(GM / r);
        state[0] = r;   state[1] = 0.0;
        state[2] = 0.0; state[3] = vc * 1.6;
        break;
    case 2:
        r = PRESET_GEO_R;   vc = sqrt(GM / r);
        state[0] = r;   state[1] = 0.0;
        state[2] = 0.0; state[3] = vc;
        break;
    case 3:
        r = PRESET_LEO_R;   ve = sqrt(2.0 * GM / r);
        state[0] = r;   state[1] = 0.0;
        state[2] = 0.0; state[3] = ve * 1.05;
        break;
    default:
        load_preset(0, state);
    }
}

// ── Two-body preset loader ────────────────────────────────────────────────────

static void load_two_body(double s1[STATE_DIM], double s2[STATE_DIM])
{
    double d = 1e8;                       // each object 1e11 m from origin
    double v = sqrt(GM2 / (4.0 * d));     // circular orbit speed

    // Object 1: left, moving up
    s1[0] = -d;   s1[1] = 0.0;
    s1[2] =  0.0; s1[3] =  v;

    // Object 2: right, moving down (total momentum = 0)
    s2[0] =  d;   s2[1] = 0.0;
    s2[2] =  0.0; s2[3] = -0;
}

// ── N-body preset loader ──────────────────────────────────────────────────────

static void load_n_body(Planet states[N_BODY_MAX], int n)
{
    SetRandomSeed(time(0));
    // srand(time(0))

    double total_momentum_x = 0.0;
    double total_momentum_y = 0.0;
    double total_CM_x       = 0.0;
    double total_CM_y       = 0.0;

    double spread  = (1e8);
    // double vspread = 1e3;
    double vspread = 5e2;

    for (int i = 0; i < n - 1; i++) {
        states[i].mass = MASS_POOL[GetRandomValue(0, MASS_POOL_SIZE - 1)];
        states[i].x    = (GetRandomValue(-1000, 1000) / 1000.0) * spread;
        states[i].y    = (GetRandomValue(-1000, 1000) / 1000.0) * spread;
        states[i].vx   = (GetRandomValue(-1000, 1000) / 1000.0) * vspread;
        states[i].vy   = (GetRandomValue(-1000, 1000) / 1000.0) * vspread;

        total_CM_x       += states[i].mass * states[i].x;
        total_CM_y       += states[i].mass * states[i].y;
        total_momentum_x += states[i].mass * states[i].vx;
        total_momentum_y += states[i].mass * states[i].vy;
    }

    // Last planet satisfies CM-at-origin and zero total momentum
    states[n-1].mass = MASS_POOL[GetRandomValue(0, MASS_POOL_SIZE - 1)];
    states[n-1].x    = -total_CM_x       / states[n-1].mass;
    states[n-1].y    = -total_CM_y       / states[n-1].mass;
    states[n-1].vx   = -total_momentum_x / states[n-1].mass;
    states[n-1].vy   = -total_momentum_y / states[n-1].mass;
}

// ── Preset metadata ───────────────────────────────────────────────────────────

#define PRESET_COUNT 6
static const char *preset_names[PRESET_COUNT] = {
    "[1] Circular LEO",
    "[2] Elliptical",
    "[3] GEO",
    "[4] Escape",
    "[5] Two-Body",
    "[6] N-Body"
};

// Colour palette for N-body planets (up to N_BODY_MAX)
static const Color PLANET_COLORS[] = {
    {80,  200, 255, 255},   // cyan
    {255, 160,  40, 255},   // orange
    {120, 255, 120, 255},   // green
    {255,  80, 120, 255},   // red-pink
    {200, 120, 255, 255},   // purple
    {255, 240,  80, 255},   // yellow
    {80,  200, 180, 255},   // teal
    {255, 180, 100, 255},   // peach
};

// ── Main ──────────────────────────────────────────────────────────────────────

int main(void)
{
    InitWindow(SCREEN_W, SCREEN_H, "Orbital Mechanics Simulator");
    SetTargetFPS(TARGET_FPS);

    // ── State ─────────────────────────────────────────────────────────────────
    double state[STATE_DIM];
    double next[STATE_DIM];

    double state1[STATE_DIM];
    double state2[STATE_DIM];
    double next1[STATE_DIM];
    double next2[STATE_DIM];

    Planet nb_states[N_BODY_MAX];   // n-body current state
    Planet nb_next[N_BODY_MAX];     // n-body next state

    int    preset_idx = 0;
    int    two_body   = 0;
    int    n_body     = 0;
    double time_scale = TIME_SCALE_INIT;
    double sim_time   = 0.0;
    int    crashed    = 0;
    int    paused     = 0;

    Trail trail;
    Trail trail1, trail2;

    load_preset(preset_idx, state);
    trail_init(&trail);
    trail_init(&trail1);
    trail_init(&trail2);

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (!WindowShouldClose())
    {
        float dt_wall = GetFrameTime();

        // ── Input ─────────────────────────────────────────────────────────────

        for (int i = 0; i < PRESET_COUNT; i++) {
            if (IsKeyPressed(KEY_ONE + i) && i != preset_idx) {
                preset_idx = i;
                sim_time   = 0.0;
                crashed    = 0;
                two_body   = (i == 4);
                n_body     = (i == 5);

                if (n_body) {
                    load_n_body(nb_states, N_BODY_COUNT);
                    time_scale = TIME_SCALE_INIT_NB;
                } else if (two_body) {
                    load_two_body(state1, state2);
                    time_scale = TIME_SCALE_INIT_TB;
                    trail_init(&trail1);
                    trail_init(&trail2);
                } else {
                    load_preset(i, state);
                    time_scale = TIME_SCALE_INIT;
                    trail_init(&trail);
                }
            }
        }

        if (IsKeyPressed(KEY_SPACE)) paused = !paused;

        if (IsKeyPressed(KEY_R)) {
            sim_time = 0.0;
            crashed  = 0;
            if (n_body) {
                load_n_body(nb_states, N_BODY_COUNT);
                // time_scale = TIME_SCALE_INIT_TB;
            } else if (two_body) {
                load_two_body(state1, state2);
                // time_scale = TIME_SCALE_INIT_TB;
                trail_init(&trail1);
                trail_init(&trail2);
            } else {
                load_preset(preset_idx, state);
                time_scale = TIME_SCALE_INIT;
                trail_init(&trail);
            }
        }

        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD))
            time_scale = fmin(time_scale * TIME_SCALE_STEP, TIME_SCALE_MAX);
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT))
            time_scale = fmax(time_scale / TIME_SCALE_STEP, TIME_SCALE_MIN);

        // ── Physics update ────────────────────────────────────────────────────
        if (!paused && !crashed) {
            double physics_seconds = dt_wall * time_scale;
            double elapsed = 0.0;
            while (elapsed < physics_seconds) {
                double step = fmin(DT, physics_seconds - elapsed);

                if (n_body) {
                    if (rk4_step_n_body(gravity_derivatives_n_body,
                                        sim_time, N_BODY_COUNT,
                                        nb_states, step, nb_next) < 0) {
                        crashed = 1;
                        break;
                    }
                    // memcpy(nb_states, nb_next, sizeof(Planet) * N_BODY_COUNT);
                    *nb_next= *nb_states;

                } else if (two_body) {
                    if (rk4_step_double_body(gravity_derivatives_double_body,
                                             sim_time, state1, state2,
                                             step, next1, next2) < 0) {
                        crashed = 1;
                        break;
                    }
                    memcpy(state1, next1, sizeof(state1));
                    memcpy(state2, next2, sizeof(state2));

                } else {
                    if (rk4_step(gravity_derivatives, sim_time,
                                 state, step, next) < 0) {
                        crashed = 1;
                        break;
                    }
                    memcpy(state, next, sizeof(state));
                }

                sim_time += step;
                elapsed  += step;
            }

            // // Trails (single and two-body only)
            // if (two_body) {
            //     double cm_x, cm_y;
            //     // manual CM for two fixed-mass objects
            //     cm_x = (M_OBJ1*state1[0] + M_OBJ2*state2[0]) / (M_OBJ1+M_OBJ2);
            //     cm_y = (M_OBJ1*state1[1] + M_OBJ2*state2[1]) / (M_OBJ1+M_OBJ2);
            //     Vector2 sp1 = world_to_screen_tb(state1[0], state1[1], cm_x, cm_y);
            //     Vector2 sp2 = world_to_screen_tb(state2[0], state2[1], cm_x, cm_y);
            //     trail_push(&trail1, sp1.x, sp1.y);
            //     trail_push(&trail2, sp2.x, sp2.y);
            // } else if (!n_body) {
            //     Vector2 sp = world_to_screen(state[0], state[1]);
            //     trail_push(&trail, sp.x, sp.y);
            // }
        }

        // ── Render ────────────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground((Color){10, 10, 20, 255});

        // // Stars
        // {
        //     static int stars_ready = 0;
        //     static Vector2 stars[200];
        //     if (!stars_ready) {
        //         SetRandomSeed(42);
        //         for (int i = 0; i < 200; i++)
        //             stars[i] = (Vector2){ (float)GetRandomValue(0, SCREEN_W),
        //                                  (float)GetRandomValue(0, SCREEN_H) };
        //         stars_ready = 1;
        //     }
        //     for (int i = 0; i < 200; i++)
        //         DrawPixelV(stars[i], (Color){200, 200, 220, 180});
        // }

        if (n_body) {
            // ── N-body rendering ──────────────────────────────────────────────
            double cm_x, cm_y;
            compute_cm(nb_states, N_BODY_COUNT, &cm_x, &cm_y);
            
            // CM marker
            DrawCircleV((Vector2){SCREEN_W*0.5f, SCREEN_H*0.5f},
                        4.0f, (Color){255, 255, 100, 200});

            for (int i = 0; i < N_BODY_COUNT; i++) {
                Vector2 p = world_to_screen_tb(nb_states[i].x, nb_states[i].y,
                                               cm_x, cm_y);
                Color c = PLANET_COLORS[i % 8];

                // Size scales with mass relative to pool max
                float r_px = 6.0f + (float)(nb_states[i].mass / 2.2e26) * 10.0f;

                DrawCircleV(p, r_px, c);
                DrawCircleLines((int)p.x, (int)p.y, r_px * 1.4f,
                                (Color){c.r, c.g, c.b, 120});
            }

            if (crashed)
                DrawText("COLLISION", SCREEN_W/2 - 50, SCREEN_H/2, 24, RED);

        } else if (two_body) {
            // ── Two-body rendering ────────────────────────────────────────────
            double cm_x = (M_OBJ1*state1[0] + M_OBJ2*state2[0]) / (M_OBJ1+M_OBJ2);
            double cm_y = (M_OBJ1*state1[1] + M_OBJ2*state2[1]) / (M_OBJ1+M_OBJ2);

            DrawCircleV((Vector2){SCREEN_W*0.5f, SCREEN_H*0.5f},
                        3.0f, (Color){255, 255, 100, 180});

            int tc = trail1.count;
            for (int i = 1; i < tc; i++) {
                float x0, y0, x1, y1;
                trail_get(&trail1, i-1, &x0, &y0);
                trail_get(&trail1, i,   &x1, &y1);
                unsigned char a = (unsigned char)(((float)i/tc)*220.0f);
                DrawLineV((Vector2){x0,y0},(Vector2){x1,y1},(Color){80,200,255,a});
            }
            tc = trail2.count;
            for (int i = 1; i < tc; i++) {
                float x0, y0, x1, y1;
                trail_get(&trail2, i-1, &x0, &y0);
                trail_get(&trail2, i,   &x1, &y1);
                unsigned char a = (unsigned char)(((float)i/tc)*220.0f);
                DrawLineV((Vector2){x0,y0},(Vector2){x1,y1},(Color){255,160,40,a});
            }

            float r1_px = (float)(R_OBJ1 / TWO_BODY_SCALE);
            float r2_px = (float)(R_OBJ2 / TWO_BODY_SCALE);

            Vector2 p1 = world_to_screen_tb(state1[0], state1[1], cm_x, cm_y);
            DrawCircleV(p1, r1_px, (Color){80, 200, 255, 255});
            DrawCircleLines((int)p1.x, (int)p1.y, r1_px*1.3f,
                            (Color){180,240,255,160});

            Vector2 p2 = world_to_screen_tb(state2[0], state2[1], cm_x, cm_y);
            DrawCircleV(p2, r2_px, (Color){255, 160, 40, 255});
            DrawCircleLines((int)p2.x, (int)p2.y, r2_px*1.3f,
                            (Color){255,210,140,160});

            if (crashed)
                DrawText("COLLISION", SCREEN_W/2-50, SCREEN_H/2-20, 24, RED);

        } else {
            // ── Single-body rendering ─────────────────────────────────────────
            Vector2 centre = { SCREEN_W*0.5f, SCREEN_H*0.5f };
            float earth_r  = to_px(R_EARTH);

            DrawCircleV(centre, earth_r*1.15f, (Color){30, 80,160, 60});
            DrawCircleV(centre, earth_r*1.05f, (Color){30, 80,160,100});
            DrawCircleV(centre, earth_r,        (Color){30,100,200,255});
            DrawCircleLines((int)centre.x, (int)centre.y, earth_r*1.06f,
                            (Color){100,180,255,80});

            int tc = trail.count;
            for (int i = 1; i < tc; i++) {
                float x0, y0, x1, y1;
                trail_get(&trail, i-1, &x0, &y0);
                trail_get(&trail, i,   &x1, &y1);
                unsigned char a = (unsigned char)(((float)i/tc)*220.0f);
                DrawLineV((Vector2){x0,y0},(Vector2){x1,y1},
                          (Color){80,200,120,a});
            }

            Vector2 sat = world_to_screen(state[0], state[1]);
            if (!crashed) {
                DrawCircleV(sat, 5.0f, WHITE);
                DrawCircleLines((int)sat.x, (int)sat.y, 8.0f,
                                (Color){200,255,200,180});
            } else {
                DrawText("IMPACT",(int)sat.x-30,(int)sat.y-20,20,RED);
            }
        }

        // ── HUD ───────────────────────────────────────────────────────────────
        {
            DrawRectangle(10, 10, 300, 100, (Color){0,0,0,160});
            DrawRectangleLines(10, 10, 300, 100, (Color){80,120,80,200});

            Color cg = (Color){100,255,120,255};
            Color cy = (Color){255,220, 60,255};
            Color cw = WHITE;
            char buf[128];

            DrawText("ORBITAL MECHANICS SIM", 20, 20, 14, cg);

            snprintf(buf, sizeof(buf), "Preset : %s", preset_names[preset_idx]);
            DrawText(buf, 20, 42, 13, cy);

            snprintf(buf, sizeof(buf), "Speed  : %.0fx", time_scale);
            DrawText(buf, 20, 60, 13, cw);

            snprintf(buf, sizeof(buf), "Time   : %.2f hrs", sim_time/3600.0);
            DrawText(buf, 20, 78, 13, cw);

            if (paused)  DrawText("[ PAUSED ]",  150, 78, 13, cy);
            if (crashed) DrawText("[ CRASHED ]", 150, 78, 13, RED);
        }

        // ── Key legend ────────────────────────────────────────────────────────
        {
            int bx = SCREEN_W-255, by = SCREEN_H-160;
            DrawRectangle(bx-5, by-5, 250, 155, (Color){0,0,0,140});
            DrawRectangleLines(bx-5, by-5, 250, 155, (Color){80,80,80,200});
            Color cl = (Color){160,160,160,255};
            DrawText("1-4  : Single-body presets", bx, by,      12, cl);
            DrawText("5    : Two-body sim",        bx, by+18,   12, cl);
            DrawText("6    : N-body sim",          bx, by+36,   12, cl);
            DrawText("SPACE: Pause / Resume",      bx, by+54,   12, cl);
            DrawText("R    : Reset",               bx, by+72,   12, cl);
            DrawText("+/-  : Time scale",          bx, by+90,   12, cl);
            DrawText("ESC  : Quit",                bx, by+108,  12, cl);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}