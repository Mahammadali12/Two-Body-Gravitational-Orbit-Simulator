#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "../config.h"
#include "physics.h"
#include "solver.h"
#include "trail.h"

// ── Helpers: unit conversion ──────────────────────────────────────────────────

// Physics metres → screen pixels (origin = screen centre)
static inline float to_px(double metres) { return (float)(metres / SCALE); }

static inline Vector2 world_to_screen(double x, double y)
{
    return (Vector2){
        SCREEN_W * 0.5f + to_px(x),
        SCREEN_H * 0.5f - to_px(y)   // y-axis flipped (screen y grows down)
    };
}

// ── Preset loader ─────────────────────────────────────────────────────────────

typedef struct { const char *name; double state[STATE_DIM]; } Preset;

static void load_preset(int idx, double state[STATE_DIM])
{
    double r, vc, ve;

    switch (idx) {

    case 0: // ── Circular LEO (400 km) ────────────────────────────────────────
        r  = PRESET_LEO_R;
        vc = sqrt(GM / r);
        state[0] =  r;   state[1] = 0.0;
        state[2] =  0.0; state[3] = vc;   // tangential velocity → circular orbit
        break;

    case 1: // ── Elliptical (high eccentricity) ─────────────────────────────
        r  = PRESET_ELLIP_R;
        vc = sqrt(GM / r);
        state[0] =  r;    state[1] = 0.0;
        state[2] =  0.0;  state[3] = vc * 1.6;  // faster → high apogee
        break;

    case 2: // ── GEO (35 786 km) ──────────────────────────────────────────────
        r  = PRESET_GEO_R;
        vc = sqrt(GM / r);
        state[0] =  r;   state[1] = 0.0;
        state[2] =  0.0; state[3] = vc;
        break;

    case 3: // ── Escape trajectory ────────────────────────────────────────────
        r  = PRESET_LEO_R;
        ve = sqrt(2.0 * GM / r);
        state[0] =  r;    state[1] = 0.0;
        state[2] =  0.0;  state[3] = ve * 1.05;  // 5% above escape velocity
        break;

    default:
        load_preset(0, state);
    }
}

static const char *preset_names[4] = {
    "[1] Circular LEO",
    "[2] Elliptical",
    "[3] GEO",
    "[4] Escape"
};

// ── Conservation diagnostics ──────────────────────────────────────────────────
// We track initial energy and angular momentum and display percentage drift.

// ── Main ──────────────────────────────────────────────────────────────────────

int main(void)
{
    InitWindow(SCREEN_W, SCREEN_H, "Orbital Mechanics Simulator");
    SetTargetFPS(TARGET_FPS);

    // Simulation state
    double state[STATE_DIM];
    double next[STATE_DIM];
    int    preset_idx  = 0;
    double time_scale  = TIME_SCALE_INIT;
    double sim_time    = 0.0;
    int    crashed     = 0;
    int    paused      = 0;

    // Conservation baselines
    double E0 = 0.0, H0 = 0.0;

    // Trail
    Trail trail;

    // Load initial preset
    load_preset(preset_idx, state);
    trail_init(&trail);
    E0 = specific_energy(state);
    H0 = specific_angular_momentum(state);
    int i=0;
    int j=0;
    // ── Main loop ─────────────────────────────────────────────────────────────
    while (!WindowShouldClose())
    {
        float dt_wall = GetFrameTime();

        // ── Input ─────────────────────────────────────────────────────────────

        // Preset selection
        for (int i = 0; i < 4; i++) {
            if (IsKeyPressed(KEY_ONE + i) && i != preset_idx) {
                preset_idx = i;
                load_preset(preset_idx, state);
                trail_init(&trail);
                E0 = specific_energy(state);
                H0 = specific_angular_momentum(state);
                sim_time = 0.0;
                crashed  = 0;
            }
        }

        // Pause / resume
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;

        // Reset current preset
        if (IsKeyPressed(KEY_R)) {
            load_preset(preset_idx, state);
            trail_init(&trail);
            E0 = specific_energy(state);
            H0 = specific_angular_momentum(state);
            sim_time = 0.0;
            crashed  = 0;
        }

        // Time scale
        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD))
            time_scale = fmin(time_scale * TIME_SCALE_STEP, TIME_SCALE_MAX);
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT))
            time_scale = fmax(time_scale / TIME_SCALE_STEP, TIME_SCALE_MIN);

        // ── Physics update ────────────────────────────────────────────────────
        if (!paused && !crashed) {
            // How many physics seconds to advance this frame
            double physics_seconds = dt_wall * time_scale;
            double elapsed = 0.0;

            while (elapsed < physics_seconds) {
                double step = fmin(DT, physics_seconds - elapsed);
                
                if (rk4_step(gravity_derivatives, sim_time, state, step, next) < 0) {
                    crashed = 1;
                    break;
                }

                memcpy(state, next, sizeof(state));
                sim_time += step;
                elapsed  += step;
            }

            // Push current position to trail (screen coords stored as floats)
            Vector2 sp = world_to_screen(state[0], state[1]);
            trail_push(&trail, sp.x, sp.y);
        }

        // ── Render ────────────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground((Color){10, 10, 20, 255});   // deep space background

        // Stars (static — seeded pattern)
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

        Vector2 centre = { SCREEN_W * 0.5f, SCREEN_H * 0.5f };
        float earth_r  = to_px(R_EARTH);

        // Earth glow
        // DrawCircleV(centre, earth_r * 1.15f, (Color){30, 80, 160, 60});
        // DrawCircleV(centre, earth_r * 1.05f, (Color){30, 80, 160, 100});

        // Earth
        DrawCircleV(centre, earth_r, (Color){30, 100, 200, 255});

        // Atmosphere ring
        DrawCircleLines((int)centre.x, (int)centre.y, earth_r * 1.06f,
                        (Color){100, 180, 255, 80});

        // ── Trail ─────────────────────────────────────────────────────────────
        int tc = trail.count;
        for (int i = 1; i < tc; i++) {
            float x0, y0, x1, y1;
            trail_get(&trail, i-1, &x0, &y0);
            trail_get(&trail, i,   &x1, &y1);

            float alpha = (float)i / tc;        // older points more transparent
            unsigned char a = (unsigned char)(alpha * 220.0f);
            Color c = { 80, 200, 120, a };

            DrawLineV((Vector2){x0, y0}, (Vector2){x1, y1}, c);
        }

        // ── Satellite ─────────────────────────────────────────────────────────
        Vector2 sat_screen = world_to_screen(state[0], state[1]);

        if (!crashed) {
            DrawCircleV(sat_screen, 5.0f, WHITE);
            DrawCircleLines((int)sat_screen.x, (int)sat_screen.y, 8.0f,
                            (Color){200, 255, 200, 180});
        } else {
            // Explosion marker
            DrawText("IMPACT", (int)sat_screen.x - 30, (int)sat_screen.y - 20,
                     20, RED);
        }

        // ── HUD ───────────────────────────────────────────────────────────────
        {
            double r       = orbital_radius(state);
            double alt_km  = (r - R_EARTH) / 1000.0;
            double spd_kms = orbital_speed(state) / 1000.0;
            double E_now   = specific_energy(state);
            double H_now   = specific_angular_momentum(state);
            double dE_pct  = fabs((E_now - E0) / (fabs(E0) + 1e-30)) * 100.0;
            double dH_pct  = fabs((H_now - H0) / (fabs(H0) + 1e-30)) * 100.0;

            // HUD background panel
            DrawRectangle(10, 10, 340, 260, (Color){0, 0, 0, 160});
            DrawRectangleLines(10, 10, 340, 260, (Color){80, 120, 80, 200});

            int y  = 20;
            int dy = 24;
            int x  = 20;
            Color cw  = WHITE;
            Color cg  = (Color){100, 255, 120, 255};
            Color cy  = (Color){255, 220, 60,  255};
            Color cr  = (Color){255, 80,  80,  255};

            DrawText("ORBITAL MECHANICS SIM",    x, y, 16, cg);  y += dy + 4;

            char buf[128];

            // Preset name
            snprintf(buf, sizeof(buf), "Preset : %s", preset_names[preset_idx]);
            DrawText(buf, x, y, 14, cy); y += dy;

            // Orbit type
            snprintf(buf, sizeof(buf), "Orbit  : %s", orbit_type(state));
            DrawText(buf, x, y, 14, cw); y += dy;

            // Altitude
            snprintf(buf, sizeof(buf), "Alt    : %.1f km", alt_km);
            DrawText(buf, x, y, 14, cw); y += dy;

            // Speed
            snprintf(buf, sizeof(buf), "Speed  : %.3f km/s", spd_kms);
            DrawText(buf, x, y, 14, cw); y += dy;

            // Specific energy
            snprintf(buf, sizeof(buf), "Energy : %.4e J/kg", E_now);
            DrawText(buf, x, y, 14, cw); y += dy;

            // Energy drift (conservation check)
            Color drift_col = (dE_pct < 0.01) ? cg : (dE_pct < 0.1) ? cy : cr;
            snprintf(buf, sizeof(buf), "E drift: %.6f %%", dE_pct);
            DrawText(buf, x, y, 14, drift_col); y += dy;

            // Angular momentum drift
            Color h_col = (dH_pct < 0.01) ? cg : (dH_pct < 0.1) ? cy : cr;
            snprintf(buf, sizeof(buf), "h drift: %.6f %%", dH_pct);
            DrawText(buf, x, y, 14, h_col); y += dy;

            // Time scale
            snprintf(buf, sizeof(buf), "Speed  : %.0fx  (+/- to change)", time_scale);
            DrawText(buf, x, y, 14, cy); y += dy;

            // Sim time
            double hrs = sim_time / 3600.0;
            snprintf(buf, sizeof(buf), "Time   : %.2f hrs", hrs);
            DrawText(buf, x, y, 14, cw);

            if (crashed) {
                DrawText("CRASHED INTO EARTH", x, y + dy, 16, cr);
            }
            if (paused) {
                DrawText("[ PAUSED ]", x, y + dy * 2, 16, cy);
            }
        }

        // ── Key legend ────────────────────────────────────────────────────────
        {
            int bx = SCREEN_W - 240, by = SCREEN_H - 130;
            DrawRectangle(bx - 5, by - 5, 235, 120, (Color){0,0,0,140});
            DrawRectangleLines(bx - 5, by - 5, 235, 120, (Color){80,80,80,200});
            Color cl = (Color){160, 160, 160, 255};
            DrawText("1-4  : Load preset",     bx, by,      13, cl);
            DrawText("SPACE: Pause / Resume",  bx, by + 18, 13, cl);
            DrawText("R    : Reset",           bx, by + 36, 13, cl);
            DrawText("+/-  : Time scale",      bx, by + 54, 13, cl);
            DrawText("ESC  : Quit",            bx, by + 72, 13, cl);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}