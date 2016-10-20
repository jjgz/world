#ifndef DRAW_H
#define DRAW_H

#include <stdbool.h>

#define DRAW_DIMS 1024

void draw_init(void);
void draw_render_start(void);
void draw_render_point(int x, int y);
void draw_render_end(void);
bool draw_check_close(void);
void draw_close(void);

#endif // DRAW_H

