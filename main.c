#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>

#include "world.h"

size_t get_world_bytes(void);
size_t get_world_bytes() {
    return MAX_BORDER_POINTS * sizeof(AbsolutePoint) +
        (WORLD_CLEAR_POINTS + WORLD_OCCUPIED_POINTS) * sizeof(VariancePoint) +
        sizeof(OrientPoint);
}

#define MAX_TEST_ADD_EVICT_POINTS 512
#define MAX_TEST_ADD_EVICT_EVICTS 16384
#define TEST_EVICT_GRAPH_PERIOD 2500
#define TEST_ADD_EVICT_SEED 8

static VariancePoint test_points[MAX_TEST_ADD_EVICT_POINTS];

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

void test_draw(VariancePoint *points, unsigned total, unsigned n);
void test_draw(VariancePoint *points, unsigned total, unsigned n) {
    char filenamebuf[100];
    sprintf(filenamebuf, "data%u.temp", n);
    FILE * temp = fopen(filenamebuf, "w");
    FILE * gnuplotPipe = popen ("gnuplot -persistent", "w");
    unsigned i;
    for (i = 0; i < total; i++) {
        fprintf(temp, "%f %f \n", (double)points[i].p.x, (double)points[i].p.y); //Write the data to a temporary file
    }
    // Set title.
    fprintf(gnuplotPipe, "set title \"Test Plot %u\" \n", n);
    // Set xrange.
    fprintf(gnuplotPipe, "set xrange [0:5] \n");
    // Set xrange.
    fprintf(gnuplotPipe, "set yrange [0:5] \n");
    // Plot file.
    fprintf(gnuplotPipe, "plot 'data%u.temp' \n", n);
    fclose(temp);
    fclose(gnuplotPipe);
}

void test_add_evict(void);
void test_add_evict() {
    srand(TEST_ADD_EVICT_SEED);
    unsigned count = 0;
    while (count < MAX_TEST_ADD_EVICT_POINTS) {
        add_evict(test_points, &count, MAX_TEST_ADD_EVICT_POINTS, rand_point());
    }

    unsigned i;
    for (i = 0; i < MAX_TEST_ADD_EVICT_EVICTS; i++) {
        add_evict(test_points, &count, MAX_TEST_ADD_EVICT_POINTS, rand_point());
        if (i % TEST_EVICT_GRAPH_PERIOD == 0)
            test_draw(test_points, MAX_TEST_ADD_EVICT_POINTS, i / TEST_EVICT_GRAPH_PERIOD);
    }
}

int main() {
    printf("World global byte requirement: %lu\n", get_world_bytes());
    test_add_evict();
    printf("Finished...\n");
    getchar();
    return 0;
}
