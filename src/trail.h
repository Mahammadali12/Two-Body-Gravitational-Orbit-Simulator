#ifndef TRAIL_H
#define TRAIL_H

#include "../config.h"

void trail_init(Trail *t);
void trail_push(Trail *t, float x, float y);

// Get i-th oldest point (0 = oldest, count-1 = newest)
void trail_get(const Trail *t, int i, float *x, float *y);

#endif // TRAIL_H