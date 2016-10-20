#include "draw.h"

#include <SDL2/SDL.h>

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;

void draw_init() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        printf("Failed to initialize SDL: %s", SDL_GetError());
        exit(1);
    }

    if (SDL_CreateWindowAndRenderer(DRAW_DIMS, DRAW_DIMS, 0, &window, &renderer)) {
        printf("Failed to create SDL window and renderer: %s", SDL_GetError());
        exit(1);
    }
}

void draw_render_start() {
    SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);
}

void draw_render_point(int x, int y) {
    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, x, y);
}

void draw_render_end() {
    SDL_RenderPresent(renderer);
}

bool draw_check_close() {
    SDL_Event event;
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
        return true;
    }
    return false;
}

void draw_close() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    SDL_Quit();
}
