/* Pairs each blue cone with its nearest yellow cone and takes the midpoints. If
   no pair is in view it follows one edge at a fixed offset instead. */
#include "planner.h"
#include "config.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

static void collect_cones(const ConeScan *scan, LocalMap *path)
{
    for (int i = 0; i < scan->count; i++) {
        const Cone *cone = &scan->cone[i];
        if (!isfinite(cone->range_m) || !isfinite(cone->bearing_rad) || cone->range_m <= 0.0f) {
            continue;
        }
        MapPoint point
            = { cone->range_m * cosf(cone->bearing_rad), cone->range_m * sinf(cone->bearing_rad) };
        if (point.x < 0.0f) continue; // behind the car
        if (cone->colour == CONE_BLUE) {
            path->left[path->left_count++] = point;
        } else {
            path->right[path->right_count++] = point;
        }
    }
}

static int gate_midpoints(const LocalMap *path, MapPoint midpoints[AUTO_MAX_CONES])
{
    int count = 0;
    for (int left = 0; left < path->left_count; left++) {
        int nearest      = -1;
        float shortest_m = PLAN_MAX_GATE_M;
        for (int right = 0; right < path->right_count; right++) {
            float distance = hypotf(path->left[left].x - path->right[right].x,
                path->left[left].y - path->right[right].y);
            if (distance < shortest_m) {
                shortest_m = distance;
                nearest    = right;
            }
        }
        if (nearest < 0) continue;
        midpoints[count].x = 0.5f * (path->left[left].x + path->right[nearest].x);
        midpoints[count].y = 0.5f * (path->left[left].y + path->right[nearest].y);
        count++;
    }
    return count;
}

// Nearest first, starting from the car, so the cones run in driving order.
static void order_from_car(MapPoint *wall, int count)
{
    MapPoint previous = { 0.0f, 0.0f };
    for (int position = 0; position < count; position++) {
        int nearest = position;
        for (int i = position + 1; i < count; i++) {
            float distance = hypotf(wall[i].x - previous.x, wall[i].y - previous.y);
            float best     = hypotf(wall[nearest].x - previous.x, wall[nearest].y - previous.y);
            if (distance < best) nearest = i;
        }
        MapPoint swap  = wall[position];
        wall[position] = wall[nearest];
        wall[nearest]  = swap;
        previous       = wall[position];
    }
}

static int follow_one_edge(LocalMap *path, MapPoint midpoints[AUTO_MAX_CONES])
{
    bool use_left  = path->left_count >= 2;
    MapPoint *wall = use_left ? path->left : path->right;
    int count      = use_left ? path->left_count : path->right_count;
    order_from_car(wall, count);

    int found = 0;
    for (int i = 0; i < count - 1; i++) {
        float dx     = wall[i + 1].x - wall[i].x;
        float dy     = wall[i + 1].y - wall[i].y;
        float length = hypotf(dx, dy);
        if (length < PLAN_MIN_SEGMENT_M) continue;
        // step sideways from the edge, into the track
        float offset       = use_left ? PLAN_EDGE_OFFSET_M : -PLAN_EDGE_OFFSET_M;
        midpoints[found].x = wall[i].x + offset * dy / length;
        midpoints[found].y = wall[i].y - offset * dx / length;
        found++;
    }
    return found;
}

// Starts at the car and always moves to the nearest unused midpoint.
static void join_midpoints(LocalMap *path, const MapPoint *midpoints, int count)
{
    bool used[AUTO_MAX_CONES] = { false };
    path->points[0]           = (MapPoint) { 0.0f, 0.0f };
    path->count               = 1;
    for (int step = 0; step < count; step++) {
        MapPoint previous = path->points[path->count - 1];
        int nearest       = -1;
        float shortest_m  = INFINITY;
        for (int i = 0; i < count; i++) {
            if (used[i]) continue;
            float distance = hypotf(midpoints[i].x - previous.x, midpoints[i].y - previous.y);
            if (distance < shortest_m) {
                shortest_m = distance;
                nearest    = i;
            }
        }
        if (nearest < 0) break;
        used[nearest] = true;
        if (shortest_m >= PLAN_MIN_POINT_GAP_M) path->points[path->count++] = midpoints[nearest];
    }
}

void planner_step(const ConeScan *scan, LocalMap *path)
{
    memset(path, 0, sizeof *path);
    if (scan->count < 0 || scan->count > AUTO_MAX_CONES) return;

    collect_cones(scan, path);
    MapPoint midpoints[AUTO_MAX_CONES];
    int count = gate_midpoints(path, midpoints);
    if (count == 0) count = follow_one_edge(path, midpoints);
    join_midpoints(path, midpoints, count);
}
