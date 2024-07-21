#ifndef UTILS_H
#define UTILS_H

#include "main.h"
#include "stdbool.h"

// Pulled from math.h since it's not included for whatever reason
#define M_PI		3.14159265358979323846

// Returns the minimum of 3 provided int16s
int16_t int16_min3(int16_t a, int16_t b, int16_t c);

// Returns an integer bounded by a min and max
int bound(int value, int min, int max);

// Returns a float bounded by a min and max
float fbound(float value, float min, float max);

// Returns a float bounded by a symmetric max
float fbound_sym(float value, float max);

// Bounds an integer between a min and max, returns true if bounded
bool enforce_bound(int *value, int min, int max);

#endif