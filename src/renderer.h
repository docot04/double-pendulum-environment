#ifndef RENDERER_H
#define RENDERER_H

#include <SDL2/SDL.h>
#include "physics.h"

#define CART_WIDTH       40
#define CART_HEIGHT      20
#define JOINT_SIZE       16
#define POLE_THICKNESS   6
#define GROUND_Y         450

typedef struct {
    SDL_Window *window;
    SDL_Renderer *renderer;

    int width;
    int height;

    float pixels_per_meter;
} Renderer;

int renderer_init(
    Renderer *r,
    int width,
    int height
);

void renderer_draw(
    Renderer *r,
    State *state
);

void renderer_destroy(
    Renderer *r
);

#endif