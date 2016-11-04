#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>

#include "world.h"
#include "draw.h"
#include "physics.h"

#define MAX_TEST_ADD_EVICT_POINTS 2048
#define MAX_TEST_ADD_EVICT_EVICTS 512
#define TEST_ADD_EVICT_SEED 8

#define TEST_PHYSICS_POINTS 1024
#define TEST_PHYSICS_FRAMES 512
#define TEST_PHYSICS_SEED 123

#define TEST_WORLD_SEED 18274
#define TEST_WORLD_WALKS 1000
#define TEST_WORLD_WALLS 20
#define TEST_WORLD_WALK_DIS_DELTA 0.02
#define TEST_WORLD_WALK_ANG_DELTA 0.03

static VariancePoint test_points[MAX_TEST_ADD_EVICT_POINTS];

static VariancePoint physics_points[TEST_PHYSICS_POINTS];
static AbsolutePoint physics_velocities[TEST_PHYSICS_POINTS];

size_t get_world_bytes(void);
size_t get_world_bytes() {
    return MAX_BORDER_POINTS * sizeof(AbsolutePoint) +
        (WORLD_CLEAR_POINTS + WORLD_OCCUPIED_POINTS) * sizeof(VariancePoint) +
        sizeof(OrientPoint);
}

VariancePoint rand_point(void);
VariancePoint rand_point() {
    VariancePoint p = {{rand() / (float)RAND_MAX * 5.0f, rand() / (float)RAND_MAX * 5.0f}, rand() / (float)RAND_MAX * 5.0f, {0, 0, 0, 0}};
    return p;
}

AbsolutePoint rand_abs_point(void);
AbsolutePoint rand_abs_point() {
    AbsolutePoint p = {rand() / (float)RAND_MAX * 5.0f, rand() / (float)RAND_MAX * 5.0f};
    return p;
}

float test_average_closest_distance(unsigned count);
float test_average_closest_distance(unsigned count) {
    float total_distance = 0.0f;
    unsigned i, j;
    for (i = 0; i < count; i++) {
        if (i + 1 < count) {
            float closest_distance_squared = point_distance_squared(&(test_points + 0)->p, &(test_points + 1)->p);
            for (j = i + 2; j < count; j++) {
                float current_distance_squared = point_distance_squared(&(test_points + i)->p, &(test_points + j)->p);
                if (current_distance_squared < closest_distance_squared) {
                    closest_distance_squared = current_distance_squared;
                }
            }
            total_distance += sqrtf(closest_distance_squared);
        }
    }
    return total_distance;
}

void render_points(VariancePoint *points, unsigned total);
void render_points(VariancePoint *points, unsigned total) {
    draw_render_start();
    unsigned i;
    for (i = 0; i < total; i++) {
        draw_render_point((int)(points[i].p.x / 5.0f * (float)DRAW_DIMS), (int)(points[i].p.y / 5.0f * (float)DRAW_DIMS));
    }
    draw_render_end();
}

void test_draw(VariancePoint *points, unsigned total);
void test_draw(VariancePoint *points, unsigned total) {
    draw_init();
    render_points(points, total);
    // Wait for window to be closed.
    while (!draw_check_close());
    draw_close();
}

void test_physics(void);
void test_physics() {
    srand(TEST_PHYSICS_SEED);
    unsigned count = 0;
    unsigned i;
    draw_init();
    unsigned evict_row = 0;
    while (count < TEST_PHYSICS_POINTS) {
        add_evict(physics_points, &count, TEST_PHYSICS_POINTS, &evict_row, rand_point());
        render_points(physics_points, count);

        if (draw_check_close()) {
            draw_close();
            return;
        }
    }

    for (i = 0; i < TEST_PHYSICS_POINTS; i++) {
        physics_velocities[i].x = 0.0;
        physics_velocities[i].y = 0.0;
    }

    for (i = 0; i < TEST_PHYSICS_FRAMES; i++) {
        unsigned j, k;
        for (j = 0; j < count; j++) {
            for (k = j + 1; k < count; k++) {
                AbsolutePoint a = phys_gravity(&physics_points[j].p, &physics_points[k].p, 0.1f, 0.000001f);
                physics_velocities[j].x += a.x;
                physics_velocities[j].y += a.y;
                physics_velocities[k].x -= a.x;
                physics_velocities[k].y -= a.y;
            }
        }

        for (j = 0; j < TEST_PHYSICS_POINTS; j++) {
            phys_drag(physics_velocities + j, 0.96f);
            physics_points[j].p.x += physics_velocities[j].x;
            physics_points[j].p.y += physics_velocities[j].y;
        }

        render_points(physics_points, TEST_PHYSICS_POINTS);

        if (draw_check_close()) {
            draw_close();
            return;
        }
    }

    draw_close();
}

void test_add_evict(void);
void test_add_evict() {
    srand(TEST_ADD_EVICT_SEED);
    draw_init();
    unsigned count = 0;
    unsigned evict_row;
    while (count < MAX_TEST_ADD_EVICT_POINTS) {
        add_evict(test_points, &count, MAX_TEST_ADD_EVICT_POINTS, &evict_row, rand_point());
        render_points(test_points, count);
        if (draw_check_close()) {
            draw_close();
            return;
        }
    }

    unsigned i;
    for (i = 0; i < MAX_TEST_ADD_EVICT_EVICTS; i++) {
        add_evict(test_points, &count, MAX_TEST_ADD_EVICT_POINTS, &evict_row, rand_point());
        render_points(test_points, count);
        if (draw_check_close()) {
            draw_close();
            return;
        }
    }

    draw_close();
}

void draw_world(void);
void draw_world() {
    unsigned k;
    draw_render_start();

    VariancePoint *clear_points;
    unsigned num_clears = world_retrieve_clear_points(&clear_points);
    for (k = 0; k < num_clears; k++)
        draw_render_color_point((int)(clear_points[k].p.x / 5.0f * (float)DRAW_DIMS), (int)(clear_points[k].p.y / 5.0f * (float)DRAW_DIMS), 0, 0, 0xFF);

    VariancePoint *occupied_points;
    unsigned num_occupied = world_retrieve_occupied_points(&occupied_points);
    for (k = 0; k < num_occupied; k++)
        draw_render_color_point((int)(occupied_points[k].p.x / 5.0f * (float)DRAW_DIMS), (int)(occupied_points[k].p.y / 5.0f * (float)DRAW_DIMS), 0xFF, 0, 0);

    draw_render_rover((int)(world_retrieve_rover()->vp.p.x / 5.0f * (float)DRAW_DIMS), (int)(world_retrieve_rover()->vp.p.y / 5.0f * (float)DRAW_DIMS), world_retrieve_rover()->angle);

    draw_render_end();
}

void test_world(void);
void test_world() {
    srand(TEST_WORLD_SEED);
    draw_init();

    // Initialize rover position and orientation.
    OrientPoint rover_point = {{{2.0, 2.0}, 0.0, {0, 0, 0, 0}}, 0.0, 0.0};

    world_init(rover_point, 0, 0, 0, 0);

    AbsolutePoint walls[TEST_WORLD_WALLS][2];

    unsigned i, j, k;

    for (i = 0; i < TEST_WORLD_WALLS; i++) {
        walls[i][0] = rand_abs_point();
        walls[i][1] = rand_abs_point();
    }

    for (i = 0; i < TEST_WORLD_WALKS; i++) {
        AbsolutePoint moving_to = rand_abs_point();
        AbsolutePoint delta = point_delta(&rover_point.vp.p, &moving_to);
        // Compute distance to travel.
        float delta_distance = sqrtf(delta_distance_squared(delta));
        unsigned move_segments = (unsigned)(fabs((double)delta_distance) / TEST_WORLD_WALK_DIS_DELTA);
        // Compute the angle to travel at.
        float delta_angle = (float)atan2((double)delta.y, (double)delta.x) - rover_point.angle;
        unsigned angle_segments = (unsigned)(fabs((double)delta_angle) / TEST_WORLD_WALK_ANG_DELTA);

        AbsolutePoint move_delta = {delta.x / move_segments, delta.y / move_segments};

        // Turn towards the point.
        for (j = 0; j < angle_segments; j++) {
            rover_point.angle += delta_angle / angle_segments;
            world_update_movement(rover_point);
            // Forward IR sensor.
            AbsolutePoint angle_s_del_forward_right = angle_delta(rover_point.angle - 0.2915f, 0.2175f);
            AbsolutePoint angle_del_forward_right = angle_delta(rover_point.angle, 0.6f);
            AbsolutePoint start_range_pos_forward_right = {rover_point.vp.p.x + angle_s_del_forward_right.x, rover_point.vp.p.y + angle_s_del_forward_right.y};
            AbsolutePoint end_range_pos_forward_right =
                {start_range_pos_forward_right.x + angle_del_forward_right.x, start_range_pos_forward_right.y + angle_del_forward_right.y};
            // Left IR sensor.
            AbsolutePoint angle_s_del_forward_left = angle_delta(rover_point.angle + 0.197395f, 0.212459f);
            AbsolutePoint angle_del_forward_left = angle_delta(rover_point.angle, 0.6f);
            AbsolutePoint start_range_pos_forward_left = {rover_point.vp.p.x + angle_s_del_forward_left.x, rover_point.vp.p.y + angle_s_del_forward_left.y};
            AbsolutePoint end_range_pos_forward_left =
                {start_range_pos_forward_left.x + angle_del_forward_left.x, start_range_pos_forward_left.y + angle_del_forward_left.y};
            // Right IR sensor.
            AbsolutePoint angle_s_del_left = angle_delta(rover_point.angle + (float)M_PI / 2, 0.125f);
            AbsolutePoint angle_del_left = angle_delta(rover_point.angle + (float)M_PI / 2, 0.6f);
            AbsolutePoint start_range_pos_left = {rover_point.vp.p.x + angle_s_del_left.x, rover_point.vp.p.y + angle_s_del_left.y};
            AbsolutePoint end_range_pos_left =
                {start_range_pos_left.x + angle_del_left.x, start_range_pos_left.y + angle_del_left.y};

            float shortest_distance_forward_right = 0.8f;
            float shortest_distance_forward_left = 0.8f;
            float shortest_distance_left = 0.8f;
            AbsolutePoint intersection;
            for (k = 0; k < TEST_WORLD_WALLS; k++) {
                // Forward-right
                if (intersection_point(walls[k][0], walls[k][1], start_range_pos_forward_right, end_range_pos_forward_right, &intersection)) {
                    float next_distance = sqrtf(point_distance_squared(&start_range_pos_forward_right, &intersection));
                    if (next_distance < shortest_distance_forward_right)
                        shortest_distance_forward_right = next_distance;
                }
                // Forward-left
                if (intersection_point(walls[k][0], walls[k][1], start_range_pos_forward_left, end_range_pos_forward_left, &intersection)) {
                    float next_distance = sqrtf(point_distance_squared(&start_range_pos_forward_left, &intersection));
                    if (next_distance < shortest_distance_forward_left)
                        shortest_distance_forward_left = next_distance;
                }
                // Left
                if (intersection_point(walls[k][0], walls[k][1], start_range_pos_left, end_range_pos_left, &intersection)) {
                    float next_distance = sqrtf(point_distance_squared(&start_range_pos_left, &intersection));
                    if (next_distance < shortest_distance_left)
                        shortest_distance_left = next_distance;
                }
            }

            world_add_front_right_ir_sensor_reading(shortest_distance_forward_right);
            world_add_front_left_ir_sensor_reading(shortest_distance_forward_left);
            world_add_left_ir_sensor_reading(shortest_distance_left);

            draw_world();

            if (draw_check_close()) {
                draw_close();
                return;
            }
        }

        // Move towards the point.
        for (j = 0; j < move_segments; j++) {
            rover_point.vp.p.x += move_delta.x;
            rover_point.vp.p.y += move_delta.y;
            world_update_movement(rover_point);
            // Forward IR sensor.
            AbsolutePoint angle_s_del_forward_right = angle_delta(rover_point.angle - 0.2915f, 0.2175f);
            AbsolutePoint angle_del_forward_right = angle_delta(rover_point.angle, 0.6f);
            AbsolutePoint start_range_pos_forward_right = {rover_point.vp.p.x + angle_s_del_forward_right.x, rover_point.vp.p.y + angle_s_del_forward_right.y};
            AbsolutePoint end_range_pos_forward_right =
                {start_range_pos_forward_right.x + angle_del_forward_right.x, start_range_pos_forward_right.y + angle_del_forward_right.y};
            // Left IR sensor.
            AbsolutePoint angle_s_del_forward_left = angle_delta(rover_point.angle + 0.197395f, 0.212459f);
            AbsolutePoint angle_del_forward_left = angle_delta(rover_point.angle, 0.6f);
            AbsolutePoint start_range_pos_forward_left = {rover_point.vp.p.x + angle_s_del_forward_left.x, rover_point.vp.p.y + angle_s_del_forward_left.y};
            AbsolutePoint end_range_pos_forward_left =
                {start_range_pos_forward_left.x + angle_del_forward_left.x, start_range_pos_forward_left.y + angle_del_forward_left.y};
            // Right IR sensor.
            AbsolutePoint angle_s_del_left = angle_delta(rover_point.angle + (float)M_PI / 2, 0.125f);
            AbsolutePoint angle_del_left = angle_delta(rover_point.angle + (float)M_PI / 2, 0.6f);
            AbsolutePoint start_range_pos_left = {rover_point.vp.p.x + angle_s_del_left.x, rover_point.vp.p.y + angle_s_del_left.y};
            AbsolutePoint end_range_pos_left =
                {start_range_pos_left.x + angle_del_left.x, start_range_pos_left.y + angle_del_left.y};

            float shortest_distance_forward_right = 0.8f;
            float shortest_distance_forward_left = 0.8f;
            float shortest_distance_left = 0.8f;
            AbsolutePoint intersection;
            for (k = 0; k < TEST_WORLD_WALLS; k++) {
                // Forward-right
                if (intersection_point(walls[k][0], walls[k][1], start_range_pos_forward_right, end_range_pos_forward_right, &intersection)) {
                    float next_distance = sqrtf(point_distance_squared(&start_range_pos_forward_right, &intersection));
                    if (next_distance < shortest_distance_forward_right)
                        shortest_distance_forward_right = next_distance;
                }
                // Forward-left
                if (intersection_point(walls[k][0], walls[k][1], start_range_pos_forward_left, end_range_pos_forward_left, &intersection)) {
                    float next_distance = sqrtf(point_distance_squared(&start_range_pos_forward_left, &intersection));
                    if (next_distance < shortest_distance_forward_left)
                        shortest_distance_forward_left = next_distance;
                }
                // Left
                if (intersection_point(walls[k][0], walls[k][1], start_range_pos_left, end_range_pos_left, &intersection)) {
                    float next_distance = sqrtf(point_distance_squared(&start_range_pos_left, &intersection));
                    if (next_distance < shortest_distance_left)
                        shortest_distance_left = next_distance;
                }
            }

            world_add_front_right_ir_sensor_reading(shortest_distance_forward_right);
            world_add_front_left_ir_sensor_reading(shortest_distance_forward_left);
            world_add_left_ir_sensor_reading(shortest_distance_left);

            draw_world();

            if (draw_check_close()) {
                draw_close();
                return;
            }
        }
    }

    draw_close();
}

int main() {
    printf("World global byte requirement: %lu\n", get_world_bytes());
    test_add_evict();
    test_physics();
    test_world();
    printf("Finished...\n");
    getchar();
    return 0;
}
