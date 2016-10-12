#include "world.h"

void world_init() {
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
