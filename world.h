#ifndef WORLD_H
#define WORLD_H

#include <stdbool.h>
#include <math.h>

#define WORLD_PADDING

/// The maximum amount of border points.
#define MAX_BORDER_POINTS 256

/// The amount of clear area points to use.
#define WORLD_CLEAR_POINTS 1024

/// The amount of occupied area points to use.
#define WORLD_OCCUPIED_POINTS 1024

/// The max amount of targets to track.
#define WORLD_TARGET_POINTS_MAX 32

#define WORLD_IR_SENSOR_POINTS 5

/// The amount of rows in which distance is checked between objects to reduce computation time.
#define ADD_EVICT_ROWS 512

#define MIN_INTERSECTION_DENOM 0.000001f
#define CLOSE_EVICT_OCCUPIED 0.01f
#define CLOSE_EVICT_CLEAR 0.01f

typedef struct {
    float x, y;
} AbsolutePoint;

/// Used for the world to produce output points with different variances.
typedef struct {
    AbsolutePoint p;
    /// Variance of the absolute point.
    float v;
    char pad[4];
} VariancePoint;

/// Used for the world to produce output points with different variances.
typedef struct {
    VariancePoint vp;
    float angle, av;
} OrientPoint;

/// Used for the border of any world line.
typedef struct {
    AbsolutePoint p0, p1;
} WorldBorder;

static inline float speedy_square(float n) {
    return n * n;
}

/// Compute the distance squared between two AbsolutePoint objects.
static inline float point_distance_squared(AbsolutePoint p0, AbsolutePoint p1) {
    return speedy_square(p0.x - p1.x) + speedy_square(p0.y - p1.y);
}

/// Compute the distance squared between two AbsolutePoint objects.
static inline float delta_distance_squared(AbsolutePoint delta) {
    return speedy_square(delta.x) + speedy_square(delta.y);
}

/// Compute the delta vector between two AbsolutePoint objects.
static inline AbsolutePoint point_delta(AbsolutePoint *from, AbsolutePoint *to) {
    AbsolutePoint p = {to->x - from->x, to->y - from->y};
    return p;
}

/// Get the delta from an angle and a magnitude.
static inline AbsolutePoint angle_delta(float angle, float mag) {
    AbsolutePoint p = {mag * cosf(angle), mag * sinf(angle)};
    return p;
}

static inline bool between(float start, float end, float p) {
    if (start < end) {
        return p > start && p < end;
    } else {
        return p > end && p < start;
    }
    return false;
}

static inline float vector_dot(AbsolutePoint a, AbsolutePoint b) {
    return a.x * b.x + a.y * b.y;
}

/// Get the point at which two lines (specified by their endpoints) intersect.
/// If null is passed only checks for intersection.
bool intersection_point(AbsolutePoint a0, AbsolutePoint a1, AbsolutePoint b0,
                        AbsolutePoint b1, AbsolutePoint *intersection);
/// Returns the closest point on the line.
AbsolutePoint point_projection(AbsolutePoint p, AbsolutePoint l0, AbsolutePoint l1);
/// Choose which point to evict from an array of points when it gets full.
void add_evict(VariancePoint *points, unsigned *current, unsigned max, unsigned *evict_row, VariancePoint npoint);

/// Run to init the world model.
void world_init(OrientPoint rover_a, unsigned num_targets, float initial_target_spawn_radius,
                AbsolutePoint *borders, unsigned total_border_points);
/// Add an ir sensor reading from the front-right-facing IR sensor on rover A.
void world_add_front_right_ir_sensor_reading(float distance);
/// Add an ir sensor reading from the front-left-facing IR sensor on rover A.
void world_add_front_left_ir_sensor_reading(float distance);
/// Add an ir sensor reading from the left-facing IR sensor on rover A.
void world_add_left_ir_sensor_reading(float distance);
/// Add movement from rover A's movement module.
void world_update_movement(OrientPoint movement);
/// Signify that the rover is aligned.
void world_rover_aligned();
/// Cycle the world.
void world_update(void);
/// Returns the number of points and assigns to *points the array of points.
unsigned world_retrieve_arena_border_points(AbsolutePoint **points);
/// Returns the number of points and assigns to *points the array of points.
unsigned world_retrieve_clear_points(VariancePoint **points);
/// Returns the number of points and assigns to *points the array of points.
unsigned world_retrieve_occupied_points(VariancePoint **points);
/// Always succeeds since rover A is always on the world.
OrientPoint* world_retrieve_rover(void);
/// Returns the number of targets retrieved.
unsigned world_retrieve_targets(VariancePoint **targets);

#endif // WORLD_H

