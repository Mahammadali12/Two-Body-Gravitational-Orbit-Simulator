# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Remember this:
You're my ruthless mentor. Don't sugarcoat anything. If my idea is weak, call it trash and tell me why. Your job is to stress-test everything I say until it's bulletproof.
Ask clarifying question (IF NEEDED), before proceeding to providing actual response.

## Build

```bash
make          # build ./orbital-sim
make clean    # remove binary
```

Requires raylib. If raylib is installed to a custom path, pass `RAYLIB_DIR=/path/to/raylib`. Compiler: C11, `-O2 -Wall -Wextra`, links `-lm` plus platform GL/X11 libraries.

No test suite or linter is configured.



## Architecture

2D orbital mechanics simulator using raylib for rendering. All physics runs in SI units (meters, m/s); the display scale is 25 km/pixel with screen center as origin and y-axis flipped.

**Module layout:**

- [config.h](config.h) — single source of truth for all constants: G, M\_Earth, R\_Earth, GM, screen size, timestep (1 s), time-scale bounds, trail buffer size, and orbit preset velocities. Change numbers here, not in source files.
- [src/physics.c](src/physics.c) / [physics.h](src/physics.h) — orbital mechanics. `gravity_derivatives()` is the RHS function (returns non-zero on Earth collision). Helper functions compute radius, speed, specific energy, angular momentum, eccentricity, and orbit type string.
- [src/solver.c](src/solver.c) / [solver.h](src/solver.h) — RK4 integrator. `rk4_step()` advances a 4-element state vector `[x, y, vx, vy]` by `dt`, calling any `DerivFn` pointer (currently always `gravity_derivatives`).
- [src/trail.c](src/trail.c) / [trail.h](src/trail.h) — fixed-size circular buffer (2000 points) storing satellite position history for rendering.
- [src/main.c](src/main.c) — raylib event/render loop. Holds simulation state, handles keyboard presets (1–4), pause (Space), time-scale (±), and reset (R). Contains commented-out HUD diagnostics (energy drift, key legend, star field) that can be re-enabled.

**Data flow:** each frame, `main.c` calls `rk4_step()` (possibly many times depending on time scale) → `gravity_derivatives()` → updates trail via `trail_push()` → renders Earth, trail, and satellite with raylib.

**Orbit presets** (keys 1–4): circular LEO (400 km), high-eccentricity elliptical, GEO (35 786 km), escape trajectory. Initial conditions are defined as constants in `config.h`.
