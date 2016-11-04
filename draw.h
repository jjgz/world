#ifndef DRAW_H
#define DRAW_H

#include <stdbool.h>
#include <stdint.h>

#define DRAW_DIMS 1024
#define DRAW_ROVER_WIDTH 34
#define DRAW_ROVER_LENGTH 68

void draw_init(void);
void draw_render_start(void);
void draw_render_point(int x, int y);
void draw_render_color_point(int x, int y, uint8_t r, uint8_t g, uint8_t b);
void draw_render_rover(int x, int y, float angle);
void draw_render_end(void);
bool draw_check_close(void);
void draw_close(void);

#endif // DRAW_H

