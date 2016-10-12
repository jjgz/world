#ifndef WORLD_H
#define WORLD_H

#include <stdbool.h>

#define WORLD_PADDING

/// The maximum amount of arena borders.
#define MAX_ARENA_BORDERS 32

/// The maximum amount of visibility borders.
#define MAX_VISIBILITY_BORDERS 256

/// The maximum amount of object borders.
#define MAX_OBJECT_BORDERS 128

/// The amount of border points to use.
#define WORLD_ARENA_BORDER_POINTS 128

/// The amount of clear area points to use.
#define WORLD_CLEAR_POINTS 1024

/// The amount of occupied area points to use.
#define WORLD_OCCUPIED_POINTS 1024

/// The ratio of the closest line's varriance the point be within to be considered part of that line.
/// If the point is outside of this distance, then the point must have a line placed on top of it.
#define WORLD_POINT_MAX_LINE_VARIANCE_RATIO 4.0f

/// The maximum variance that a line can have before another line is created adjacent to it.
#define WORLD_LINE_MAX_VARIANCE 1.0f

/// Used for the world to produce output points with different variances.
typedef struct {
    float x, y;
    /// Variance of the point.
    float v;
} WorldPoint;

/// Used for the world to produce output points which are ends of a line.
typedef struct {
    WorldPoint point;
    bool open;
#ifdef WORLD_PADDING
    char pad[3];
#endif
} EndPoint;

/// Used for the world to produce output points which have orientation.
/// An angle of 0 points in the normal direction <1.0, 0.0>.
typedef struct {
    WorldPoint point;
    float angle;
    /// The angle variance.
    float av;
} OrientPoint;

/// Used for the border of any world line.
typedef struct {
    EndPoint p0, p1;
} WorldBorder;

/// Compute the distance squared between two WorldPoint objects.
float point_distance_squared(WorldPoint *p0, WorldPoint *p1);
/// Choose which point to evict from an array of points when it gets full.
void add_evict(WorldPoint *points, unsigned *current, unsigned max, WorldPoint npoint);

/// Run to init the world model.
void world_init(void);
/// Add an arena border reading at the correct location on rover A.
void world_add_arena_border_reading(float variance);
/// Add an ir sensor reading from the front-facing IR sensor on rover A.
void world_add_front_ir_sensor_reading(float distance, float variance);
/// Add an ir sensor reading from the left-facing IR sensor on rover A.
void world_add_left_ir_sensor_reading(float distance, float variance);
/// Add an ir sensor reading from the right-facing IR sensor on rover A.
void world_add_right_ir_sensor_reading(float distance, float variance);
/// Add an ultrasonic sensor reading from the front-facing ultrasonic sensor on rover A.
void world_add_front_ultrasonic_reading(float distance, float variance);
/// Add an ultrasonic sensor reading from the left-facing ultrasonic sensor on rover A.
void world_add_left_ultrasonic_reading(float distance, float variance);
/// Add an ultrasonic sensor reading from the right-facing ultrasonic sensor on rover A.
void world_add_right_ultrasonic_reading(float distance, float variance);
/// Add movement from rover A's movement module.
void world_add_movement_a(WorldPoint from, WorldPoint to);
/// Add movement from rover B's movement module.
void world_add_movement_b(WorldPoint from, WorldPoint to);
/// Cycle the world.
void world_update(void);
/// Returns the number of borders and assigns to *borders the array of borders.
unsigned world_retrieve_arena_borders(WorldBorder **borders);
/// Returns the number of borders and assigns to *borders the array of borders.
unsigned world_retrieve_visibility_borders(WorldBorder **borders);
/// Returns the number of borders and assigns to *borders the array of borders.
unsigned world_retrieve_object_borders(WorldBorder **borders);
/// Always succeeds since rover A is always on the world.
void world_retrieve_rover_a(OrientPoint *rover);
/// Returns true on success.
bool world_retrieve_rover_b(OrientPoint *rover);
/// Returns the number of targets retrieved.
unsigned world_retrieve_targets(OrientPoint **targets);

#endif // WORLD_H

