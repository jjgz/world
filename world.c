#include "world.h"
#include <stdlib.h>
#include <stdint.h>

typedef struct {
    AbsolutePoint pos;
    AbsolutePoint vel;
} VelocityPoint;

// Points at which we discovered an arena border.
static AbsolutePoint arena_border_points[MAX_BORDER_POINTS];
static unsigned num_arena_border_points;
static unsigned arena_border_evict_row;

// Points at which we know the area is clear.
static VariancePoint clear_points[WORLD_CLEAR_POINTS];
static unsigned num_clear_points;
static unsigned clear_points_evict_row;

// Points at which we know the area is occupied.
static VariancePoint occupied_points[WORLD_OCCUPIED_POINTS];
static unsigned num_occupied_points;
static unsigned occupied_points_evict_row;

static OrientPoint rover;

// Points at which we think there is a target.
static VelocityPoint targets[WORLD_TARGET_POINTS_MAX];
static VariancePoint target_stat_points[WORLD_TARGET_POINTS_MAX];
static unsigned num_targets;

bool intersection_point(AbsolutePoint a0, AbsolutePoint a1, AbsolutePoint b0,
                        AbsolutePoint b1, AbsolutePoint *intersection) {
    float x1 = a0.x;
    float x2 = a1.x;
    float x3 = b0.x;
    float x4 = b1.x;
    float y1 = a0.y;
    float y2 = a1.y;
    float y3 = b0.y;
    float y4 = b1.y;

    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (denom > MIN_INTERSECTION_DENOM) {
        intersection->x = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / denom;
        intersection->y = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / denom;

        // Check if the intersection point is on the line.
        return between(x1, x2, intersection->x) && between(x3, x4, intersection->x) &&
            between(y1, y2, intersection->y) && between(y3, y4, intersection->y);
    }

    return false;
}

void add_evict(VariancePoint *points, unsigned *current, unsigned max, unsigned *evict_row, VariancePoint npoint) {
    if (*current < max) {
        points[(*current)++] = npoint;
        return;
    } else {
        float smallest_distance = point_distance_squared(&(points + 0)->p, &(points + 1)->p);
        unsigned evict_index = 0;

        unsigned i, j;
        for (i = *evict_row; i < max; i += ADD_EVICT_ROWS) {
            for (j = i + 1; j < max; j++) {
                float current_distance = point_distance_squared(&(points + i)->p, &(points + j)->p);
                if (current_distance < smallest_distance) {
                    smallest_distance = current_distance;
                    evict_index = i;
                }
            }
        }

        // Move to the next evict row.
        (*evict_row)++;
        *evict_row %= ADD_EVICT_ROWS;

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
    arena_border_evict_row = 0;
    clear_points_evict_row = 0;
    occupied_points_evict_row = 0;
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

void world_add_front_right_ir_sensor_reading(float distance) {
    static const float longest_ir_distance = 0.5f;
    AbsolutePoint start = rover.vp.p;
    AbsolutePoint initial_spacing = angle_delta(rover.angle - 0.2915f, 0.2175f);
    start.x += initial_spacing.x;
    start.y += initial_spacing.y;
    float distance_effective;
    if (distance < longest_ir_distance) {
        distance_effective = distance;
    } else {
        distance_effective = longest_ir_distance;
    }
    AbsolutePoint point_spacing = angle_delta(rover.angle, distance_effective / WORLD_IR_SENSOR_POINTS);
    unsigned i;
    for (i = 0; i < WORLD_IR_SENSOR_POINTS; i++) {
        // TODO: Compute variance.
        VariancePoint p = {start, 0.0, {0, 0, 0, 0}};
        add_evict(clear_points, &num_clear_points, WORLD_CLEAR_POINTS, &clear_points_evict_row, p);
        start.x += point_spacing.x;
        start.y += point_spacing.y;
    }
    if (distance < longest_ir_distance) {
        VariancePoint p = {start, 0.0, {0, 0, 0, 0}};
        add_evict(occupied_points, &num_occupied_points, WORLD_OCCUPIED_POINTS, &occupied_points_evict_row, p);
    }
}

void world_add_front_left_ir_sensor_reading(float distance) {
    static const float longest_ir_distance = 0.5f;
    AbsolutePoint start = rover.vp.p;
    AbsolutePoint initial_spacing = angle_delta(rover.angle + 0.197395f, 0.212459f);
    start.x += initial_spacing.x;
    start.y += initial_spacing.y;
    float distance_effective;
    if (distance < longest_ir_distance) {
        distance_effective = distance;
    } else {
        distance_effective = longest_ir_distance;
    }
    AbsolutePoint point_spacing = angle_delta(rover.angle, distance_effective / WORLD_IR_SENSOR_POINTS);
    unsigned i;
    for (i = 0; i < WORLD_IR_SENSOR_POINTS; i++) {
        // TODO: Compute variance.
        VariancePoint p = {start, 0.0, {0, 0, 0, 0}};
        add_evict(clear_points, &num_clear_points, WORLD_CLEAR_POINTS, &clear_points_evict_row, p);
        start.x += point_spacing.x;
        start.y += point_spacing.y;
    }
    if (distance < longest_ir_distance) {
        VariancePoint p = {start, 0.0, {0, 0, 0, 0}};
        add_evict(occupied_points, &num_occupied_points, WORLD_OCCUPIED_POINTS, &occupied_points_evict_row, p);
    }
}

void world_add_left_ir_sensor_reading(float distance) {
    static const float longest_ir_distance = 0.5f;
    AbsolutePoint start = rover.vp.p;
    AbsolutePoint initial_spacing = angle_delta(rover.angle + (float)M_PI / 2, 0.125f);
    start.x += initial_spacing.x;
    start.y += initial_spacing.y;
    float distance_effective;
    if (distance < longest_ir_distance) {
        distance_effective = distance;
    } else {
        distance_effective = longest_ir_distance;
    }
    AbsolutePoint point_spacing = angle_delta(rover.angle + (float)M_PI / 2, distance_effective / WORLD_IR_SENSOR_POINTS);
    unsigned i;
    for (i = 0; i < WORLD_IR_SENSOR_POINTS; i++) {
        // TODO: Compute variance.
        VariancePoint p = {start, 0.0, {0, 0, 0, 0}};
        add_evict(clear_points, &num_clear_points, WORLD_CLEAR_POINTS, &clear_points_evict_row, p);
        start.x += point_spacing.x;
        start.y += point_spacing.y;
    }
    if (distance < longest_ir_distance) {
        VariancePoint p = {start, 0.0, {0, 0, 0, 0}};
        add_evict(occupied_points, &num_occupied_points, WORLD_OCCUPIED_POINTS, &occupied_points_evict_row, p);
    }
}

void world_update_movement(OrientPoint movement) {
    rover = movement;
}

void world_rover_aligned() {
    // Initial assumption: Assume the latest movement has the correct alignment.
    // Iterate over all border edges.
    unsigned i;
    AbsolutePoint rover_center = rover.vp.p;
    AbsolutePoint rover_forwards = angle_delta(rover.angle, 5.0f);
    rover_forwards.x += rover_center.x;
    rover_forwards.y += rover_center.y;
    AbsolutePoint intersection;
    bool is_intersected = false;
    float best_intersection = 50.0f;
    unsigned best_index;
    // NOTE: This fails if num_arena_border_points is 1, but otherwise does not.
    for (i = 0; i < num_arena_border_points; i++) {
        AbsolutePoint *a = &arena_border_points[i];
        AbsolutePoint *b = &arena_border_points[(i+1) % num_arena_border_points];
        AbsolutePoint current_intersect;
        bool current_intersected = intersection_point(*a, *b, rover_center,
                        rover_forwards, &current_intersect);
        if (current_intersected) {
            float distance_squared = point_distance_squared(&rover_center, &current_intersect);
            if (distance_squared < best_intersection) {
                best_intersection = distance_squared;
                intersection = current_intersect;
                is_intersected = true;
                best_index = i;
            }
        }
    }
    
    if (is_intersected) {
        // Go the amount backwards by which the alignment sensor is forwards
        AbsolutePoint rover_backwards = angle_delta(-rover.angle, 0.21f);
        intersection.x += rover_backwards.x;
        intersection.y += rover_backwards.y;
        // Set this as the new rover position.
        rover.vp.p = intersection;
        
        AbsolutePoint *a = &arena_border_points[best_index];
        AbsolutePoint *b = &arena_border_points[(best_index+1) % num_arena_border_points];
        
        // Get the atan2 of the border delta (which is counterclockwise).
        float border_angle = atan2(b->y - a->y, b->x - a->x);
        // Set the rover's angle to be the border angle turned right by pi/2.
        rover.angle = border_angle - M_PI_2;
    }
}

void world_update() {
    // TODO: Look for targets.
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
