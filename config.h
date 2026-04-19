#ifndef CONFIG_H
#define CONFIG_H

// ── Physical constants ────────────────────────────────────────────────────────
#define G           6.674e-11       // gravitational constant  (m³ kg⁻¹ s⁻²)
#define M_EARTH     5.972e24        // Earth mass              (kg)
#define R_EARTH     6.371e6         // Earth radius            (m)
#define GM          (G * M_EARTH)   // standard gravitational parameter

// ── Display ───────────────────────────────────────────────────────────────────
#define SCREEN_W    1200
#define SCREEN_H    900
#define TARGET_FPS  60

// 1 pixel = SCALE meters
// Earth radius in pixels = R_EARTH / SCALE ≈ 254 px  (fits comfortably)
#define SCALE       25000.0

// ── Simulation defaults ───────────────────────────────────────────────────────
#define DT              1.0         // physics timestep (seconds)
#define TIME_SCALE_INIT 500.0       // sim-seconds per wall-second at startup
#define TIME_SCALE_MIN  10.0
#define TIME_SCALE_MAX  50000.0
#define TIME_SCALE_STEP 1.5         // multiply/divide on +/- key

#define TIME_SCALE_INIT_TB   50000.0     // two-body default

// ── Trail ─────────────────────────────────────────────────────────────────────
#define TRAIL_LEN   2000            // circular buffer capacity

// ── Orbit presets ─────────────────────────────────────────────────────────────
// Each preset sets initial position (x, y) and velocity (vx, vy) in SI units.
// Satellite starts directly "above" Earth on the +y axis.

// Circular velocity at radius r:  v_c = sqrt(GM / r)
// Escape velocity:                v_e = sqrt(2*GM / r)

#define PRESET_LEO_ALT      400e3                       // 400 km above surface
#define PRESET_GEO_ALT      35786e3                     // geostationary altitude
#define PRESET_ELLIP_ALT    400e3                       // perigee same as LEO

// Precomputed radii from Earth centre
#define PRESET_LEO_R    (R_EARTH + PRESET_LEO_ALT)
#define PRESET_GEO_R    (R_EARTH + PRESET_GEO_ALT)
#define PRESET_ELLIP_R  (R_EARTH + PRESET_ELLIP_ALT)

// ── Two-body rendering scale ──────────────────────────────────────────────────
// 1 px = TWO_BODY_SCALE metres
// Objects at d=1e8 m from origin → 100 px from centre on screen
#define TWO_BODY_SCALE  1e6

// ── Two-body object definitions ───────────────────────────────────────────────
#define M_OBJ1  1e26
#define R_OBJ1  1e10
#define GM1     (G * M_OBJ1)

#define M_OBJ2  1e26
#define R_OBJ2  1e10
#define GM2     (G * M_OBJ2)

#endif // CONFIG_H