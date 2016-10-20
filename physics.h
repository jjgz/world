#ifndef PHYSICS_H
#define PHYSICS_H

#include "world.h"

#define SMALLEST_ALLOWED_DISTANCE 0.0001f

/// Takes two points, the radius squared at which the acceleration scales linearly, and the magnitude of the interaction.
/// Returns an acceleration.
AbsolutePoint phys_gravity(AbsolutePoint *p0, AbsolutePoint *p1, float radius_squared, float mag);
/// Takes two points and the magnitude of the interaction.
/// Returns an acceleration.
AbsolutePoint phys_hooke(AbsolutePoint *p0, AbsolutePoint *p1, float mag);
/// Takes a velocity and modifies it to be a ratio of the original based on `rat`.
void phys_drag(AbsolutePoint *v, float rat);

#endif // PHYSICS_H

