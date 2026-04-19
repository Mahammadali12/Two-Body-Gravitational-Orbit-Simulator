#include "trail.h"
#include <string.h>

void trail_init(Trail *t)
{
    memset(t, 0, sizeof(*t));
}

void trail_push(Trail *t, float x, float y)
{
    t->x[t->head] = x;
    t->y[t->head] = y;
    t->head = (t->head + 1) % TRAIL_LEN;
    if (t->count < TRAIL_LEN) t->count++;
}

void trail_get(const Trail *t, int i, float *x, float *y)
{
    // oldest entry sits at (head - count + i) mod TRAIL_LEN
    int idx = (t->head - t->count + i + TRAIL_LEN * 2) % TRAIL_LEN;
    *x = t->x[idx];
    *y = t->y[idx];
}