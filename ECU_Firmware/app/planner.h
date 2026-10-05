/* Builds a short centre line ahead of the car from the cones in one scan. */
#ifndef PLANNER_H
#define PLANNER_H

#include "local_map.h"

void planner_step(const ConeScan *scan, LocalMap *path);

#endif
