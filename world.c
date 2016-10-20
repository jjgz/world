#include "world.h"
#include <stdlib.h>
#include <math.h>

typedef struct {
    AbsolutePoint pos;
    AbsolutePoint vel;
} VelocityPoint;

// Points at which we discovered an arena border.
static AbsolutePoint arena_border_points[MAX_BORDER_POINTS];
static unsigned num_arena_border_points;

// Points at which we know the area is clear.
static VariancePoint clear_points[WORLD_CLEAR_POINTS];
static unsigned num_clear_points;

// Points at which we know the area is occupied.
static VariancePoint occupied_points[WORLD_OCCUPIED_POINTS];
static unsigned num_occupied_points;

static OrientPoint rover;

// Points at which we think there is a target.
static VelocityPoint targets[WORLD_TARGET_POINTS_MAX];
static VariancePoint target_stat_points[WORLD_TARGET_POINTS_MAX];
static unsigned num_targets;

float point_distance_squared(AbsolutePoint *p0, AbsolutePoint *p1) {
    return powf(p0->x - p1->x, 2) + powf(p0->y - p1->y, 2);
}

void add_evict(VariancePoint *points, unsigned *current, unsigned max, VariancePoint npoint) {
    if (*current < max) {
        points[(*current)++] = npoint;
        return;
    } else {
        float smallest_distance = point_distance_squared(&(points + 0)->p, &(points + 1)->p);
        unsigned evict_index = 0;

        unsigned i, j;
        for (i = 0; i < max; i++) {
            for (j = i + 1; j < max; j++) {
                float current_distance = point_distance_squared(&(points + i)->p, &(points + j)->p);
                if (current_distance < smallest_distance) {
                    smallest_distance = current_distance;
                    evict_index = i;
                }
            }
        }

        for (i = 0; i < max; i++) {
            float current_distance = point_distance_squared(&(points + i)->p, &npoint.p);
            if (current_distance < smallest_distance) {
                // The new point has the smallest distance, so dont add it.
                return;
            }
        }

        points[evict_index] = npoint;
    }
}

void world_init(OrientPoint _rover, unsigned _num_targets, float initial_target_spawn_radius,
                AbsolutePoint *borders, unsigned total_border_points) {
    num_arena_border_points = total_border_points;
    unsigned i;
    for (i = 0; i < total_border_points; i++)
        arena_border_points[i] = borders[i];
    num_clear_points = 0;
    num_occupied_points = 0;

    rover = _rover;
    num_targets = _num_targets;
    // TODO: Intialize these to be in a uniform circle around the center.
    for (i = 0; i < num_targets; i++) {
        float angle = i * 2 * (float)M_PI / num_targets;
        VelocityPoint p = {{initial_target_spawn_radius * cosf(angle), initial_target_spawn_radius * sinf(angle)}, {0.0, 0.0}};
        targets[i] = p;
    }
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

void world_add_movement(OrientPoint movement) {
    rover = movement;
}

void world_update() {
}

unsigned world_retrieve_arena_border_points(AbsolutePoint **points) {
    *points = arena_border_points;
    return num_arena_border_points;
}

unsigned world_retrieve_clear_points(VariancePoint **points) {
    *points = clear_points;
    return num_clear_points;
}

unsigned world_retrieve_occupied_points(VariancePoint **points) {
    *points = occupied_points;
    return num_occupied_points;
}

OrientPoint* world_retrieve_rover() {
    return &rover;
}

unsigned world_retrieve_targets(VariancePoint **_targets) {
    unsigned i;
    for (i = 0; i < num_targets; i++) {
        target_stat_points[i].p = targets[i].pos;
        // TODO: Calculate variance.
        target_stat_points[i].v = 0.1f;
    }
    *_targets = target_stat_points;
    return num_targets;
}
