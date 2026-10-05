/* The cones the ECU is given and the short centre line it builds from them. All
   positions are in the car frame: x ahead, y to the left, metres. */
#ifndef LOCAL_MAP_H
#define LOCAL_MAP_H

#include <stdint.h>

#define AUTO_MAX_CONES 16 // nearest cones only, so a scan fits on the bus

enum { CONE_BLUE, CONE_YELLOW }; // blue marks the left edge, yellow the right

typedef struct {
    float range_m;
    float bearing_rad; // positive to the left of the car's heading
    uint8_t colour;
} Cone;

typedef struct {
    Cone cone[AUTO_MAX_CONES];
    int count;
} ConeScan;

typedef struct {
    float x;
    float y;
} MapPoint;

typedef struct {
    MapPoint points[AUTO_MAX_CONES + 1]; // the car itself, then one point per gate
    int count;
    MapPoint left[AUTO_MAX_CONES];
    MapPoint right[AUTO_MAX_CONES];
    int left_count;
    int right_count;
} LocalMap;

#endif
