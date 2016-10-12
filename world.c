#include "world.h"

// Lines which map to borders of the arena.
WorldBorder arena_borders[MAX_ARENA_BORDERS];
unsigned num_arena_borders;

// Lines which map to borders of the edge of visibility.
WorldBorder visibility_borders[MAX_VISIBILITY_BORDERS];
unsigned num_visibility_borders;

// Lines which map to borders of objects.
WorldBorder object_borders[MAX_OBJECT_BORDERS];
unsigned num_object_borders;

// Points at which we discovered an arena border.
WorldPoint arena_border_points[WORLD_ARENA_BORDER_POINTS];
unsigned num_arena_border_points;

// Points at which we know the area is clear.
WorldPoint clear_points[WORLD_CLEAR_POINTS];
unsigned num_clear_points;

// Points at which we know the area is occupied.
WorldPoint occupied_points[WORLD_OCCUPIED_POINTS];
unsigned num_occupied_points;

OrientPoint rover_a;
OrientPoint rover_b;
bool rover_b_found;

float point_distance_squared(WorldPoint *p0, WorldPoint *p1) {
    return p0->x * p1->x + p0->y * p1->y;
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

void world_add_arena_border_reading(float variance) {
}

void world_add_front_ir_sensor_reading(float distance, float variance) {
}

void world_add_left_ir_sensor_reading(float distance, float variance) {
}

void world_add_right_ir_sensor_reading(float distance, float variance) {
}

void world_add_front_ultrasonic_reading(float distance, float variance) {
}

void world_add_left_ultrasonic_reading(float distance, float variance) {
}

void world_add_right_ultrasonic_reading(float distance, float variance) {
}

void world_add_movement_a(WorldPoint from, WorldPoint to) {
}

void world_add_movement_b(WorldPoint from, WorldPoint to) {
}

void world_update() {
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
