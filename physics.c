#include "physics.h"

#include <math.h>

static const AbsolutePoint zerov = {0.0, 0.0};

AbsolutePoint phys_gravity(AbsolutePoint *p0, AbsolutePoint *p1, float radius_squared, float mag) {
    float dx = p1->x - p0->x;
    float dy = p1->y - p0->y;
    float distance_squared = powf(dx, 2) + powf(dy, 2);
    if (distance_squared > radius_squared) {
        float multiplier = mag / powf(distance_squared, 3);
        AbsolutePoint a = {multiplier * dx, multiplier * dy};
        return a;
    } else if (distance_squared > SMALLEST_ALLOWED_DISTANCE) {
        float multiplier = mag / radius_squared;
        AbsolutePoint a = {multiplier * dx, multiplier * dy};
        return a;
    }
    return zerov;
}

AbsolutePoint phys_hooke(AbsolutePoint *p0, AbsolutePoint *p1, float mag) {
    float dx = p1->x - p0->x;
    float dy = p1->y - p0->y;
    AbsolutePoint a = {mag * dx, mag * dy};
    return a;
}

void phys_drag(AbsolutePoint *v, float rat) {
    v->x *= rat;
    v->y *= rat;
}
