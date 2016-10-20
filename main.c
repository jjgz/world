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
    while (count < TEST_PHYSICS_POINTS) {
        add_evict(physics_points, &count, TEST_PHYSICS_POINTS, rand_point());
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
            phys_drag(physics_velocities + j, 0.96);
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
    while (count < MAX_TEST_ADD_EVICT_POINTS) {
        add_evict(test_points, &count, MAX_TEST_ADD_EVICT_POINTS, rand_point());
        render_points(test_points, count);
        if (draw_check_close()) {
            draw_close();
            return;
        }
    }

    unsigned i;
    for (i = 0; i < MAX_TEST_ADD_EVICT_EVICTS; i++) {
        add_evict(test_points, &count, MAX_TEST_ADD_EVICT_POINTS, rand_point());
        render_points(test_points, count);
        if (draw_check_close()) {
            draw_close();
            return;
        }
    }

    draw_close();
}

int main() {
    printf("World global byte requirement: %lu\n", get_world_bytes());
    test_physics();
    test_add_evict();
    printf("Finished...\n");
    getchar();
    return 0;
}
