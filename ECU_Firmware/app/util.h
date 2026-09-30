/* Small maths helpers shared by the app files. */
#ifndef UTIL_H
#define UTIL_H

static inline float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

#endif
