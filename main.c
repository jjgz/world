#include <stdio.h>
#include <stdlib.h>

#include "world.h"

size_t get_world_bytes(void);

size_t get_world_bytes() {
    return (MAX_ARENA_BORDERS + MAX_VISIBILITY_BORDERS + MAX_OBJECT_BORDERS) * sizeof(WorldBorder) +
        (WORLD_ARENA_BORDER_POINTS + WORLD_CLEAR_POINTS + WORLD_OCCUPIED_POINTS) * sizeof(WorldPoint) +
        2 * sizeof(OrientPoint) +
        sizeof(bool);
}

int main() {
    printf("World global byte requirement: %lu", get_world_bytes());
    getchar();
    return 0;
}
