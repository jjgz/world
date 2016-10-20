#include "draw.h"

#include <SDL2/SDL.h>
#include <stdlib.h>

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Texture *rover_texture = NULL;

void draw_init() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        printf("Failed to initialize SDL: %s", SDL_GetError());
        exit(1);
    }

    if (SDL_CreateWindowAndRenderer(DRAW_DIMS, DRAW_DIMS, 0, &window, &renderer)) {
        printf("Failed to create SDL window and renderer: %s", SDL_GetError());
        exit(1);
    }

    SDL_GL_SetSwapInterval(1);

    rover_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, 1);

    static uint8_t pixel[] = {0xFF, 0x00, 0xFF, 0x00};

    SDL_UpdateTexture(rover_texture, 0, (void*)pixel, 4);
}

void draw_render_start() {
    SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);
}

void draw_render_point(int x, int y) {
    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, x, y);
}

void draw_render_color_point(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    SDL_SetRenderDrawColor(renderer, r, g, b, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, x, y);
}

void draw_render_rover(int x, int y, float angle) {
    SDL_Rect dstrect;
    dstrect.x = x - DRAW_ROVER_LENGTH/2;
    dstrect.y = y - DRAW_ROVER_WIDTH/2;
    dstrect.w = DRAW_ROVER_LENGTH;
    dstrect.h = DRAW_ROVER_WIDTH;
    SDL_RenderCopyEx(renderer,
                     rover_texture,
                     0,
                     &dstrect,
                     (double)angle / M_PI * 180.0,
                     0,
                     SDL_FLIP_NONE);
}

void draw_render_end() {
    SDL_RenderPresent(renderer);
}

bool draw_check_close() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            return true;
        }
    }
    return false;
}

void draw_close() {
    SDL_DestroyTexture(rover_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    SDL_Quit();
}
