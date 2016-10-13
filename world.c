#include "world.h"
#include <stdlib.h>
#include <math.h>

// Lines which map to borders of the arena.
static WorldBorder arena_borders[MAX_ARENA_BORDERS];
static unsigned num_arena_borders;

// Lines which map to borders of the edge of visibility.
static WorldBorder visibility_borders[MAX_VISIBILITY_BORDERS];
static unsigned num_visibility_borders;

// Lines which map to borders of objects.
static WorldBorder object_borders[MAX_OBJECT_BORDERS];
static unsigned num_object_borders;

// Points at which we discovered an arena border.
static WorldPoint arena_border_points[WORLD_ARENA_BORDER_POINTS];
static unsigned num_arena_border_points;

// Points at which we know the area is clear.
static WorldPoint clear_points[WORLD_CLEAR_POINTS];
static unsigned num_clear_points;

// Points at which we know the area is occupied.
static WorldPoint occupied_points[WORLD_OCCUPIED_POINTS];
static unsigned num_occupied_points;

static OrientPoint rover_a;
static OrientPoint rover_b;
static bool rover_b_found;

float point_distance_squared(WorldPoint *p0, WorldPoint *p1) {
    return powf(p0->x - p1->x, 2) + powf(p0->y - p1->y, 2);
}

void add_evict(WorldPoint *points, unsigned *current, unsigned max, WorldPoint npoint) {
    if (*current < max) {
        points[(*current)++] = npoint;
        return;
    } else {
        float smallest_distance = point_distance_squared(points + 0, points + 1);
        unsigned evict_index = 0;

        unsigned i, j;
        for (i = 0; i < max; i++) {
            for (j = i + 1; j < max; j++) {
                float current_distance = point_distance_squared(points + i, points + j);
                if (current_distance < smallest_distance) {
                    smallest_distance = current_distance;
                    evict_index = i;
                }
            }
        }

        points[evict_index] = npoint;
    }
}

void world_init() {
    num_arena_borders = 0;
    num_visibility_borders = 0;
    num_object_borders = 0;
    num_arena_border_points = 0;
    num_clear_points = 0;
    num_occupied_points = 0;

    rover_a.angle = 0.0;
    rover_a.av = 0.0;
    rover_a.point.x = 0.0;
    rover_a.point.y = 0.0;

    rover_b_found = false;
}

void world_add_arena_border_reading() {
    WorldPoint p = {ROVER_HLENGTH * cosf(rover_a.angle) + ROVER_HWIDTH * sinf(rover_a.angle) + rover_a.point.x,
        -ROVER_HWIDTH * cosf(rover_a.angle) + ROVER_HLENGTH * sinf(rover_a.angle) + rover_a.point.y,
        BORDER_READING_VARIANCE};
    add_evict(arena_border_points, &num_arena_border_points, WORLD_ARENA_BORDER_POINTS, p);
}

void world_add_front_ir_sensor_reading(float distance) {
}

void world_add_left_ir_sensor_reading(float distance) {
}

void world_add_right_ir_sensor_reading(float distance) {
}

void world_add_front_ultrasonic_reading(float distance) {
}

void world_add_left_ultrasonic_reading(float distance) {
}

void world_add_right_ultrasonic_reading(float distance) {
}

void world_add_movement_a(OrientPoint from, OrientPoint to) {
    rover_a = to;
}

void world_add_movement_b(OrientPoint from, OrientPoint to) {
    rover_b = to;
    rover_b_found = true;
}

void world_update() {
}

unsigned world_retrieve_arena_border_points(WorldPoint **points) {
    *points = arena_border_points;
    return num_arena_border_points;
}

unsigned world_retrieve_clear_points(WorldPoint **points) {
    *points = clear_points;
    return num_clear_points;
}

unsigned world_retrieve_occupied_points(WorldPoint **points) {
    *points = occupied_points;
    return num_occupied_points;
}

unsigned world_retrieve_arena_borders(WorldBorder **borders) {
    return 0;
}

unsigned world_retrieve_visibility_borders(WorldBorder **borders) {
    return 0;
}

unsigned world_retrieve_object_borders(WorldBorder **borders) {
    return 0;
}

void world_retrieve_rover_a(OrientPoint *rover) {
}

bool world_retrieve_rover_b(OrientPoint *rover) {
    return false;
}

unsigned world_retrieve_targets(OrientPoint **targets) {
    return 0;
}
