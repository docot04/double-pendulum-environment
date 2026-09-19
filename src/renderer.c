#include "renderer.h"
#include <stdio.h>
#include <math.h>

static void draw_pole(SDL_Renderer *renderer, int x1, int y1, int x2, int y2) {
    float dx = (float)(x2 - x1);
    float dy = (float)(y2 - y1);
    float length = sqrtf(dx * dx + dy * dy);

    if (length == 0.0f)
        return;
    
    float px = -dy / length;
    float py =  dx / length;

    int half = POLE_THICKNESS / 2;

    for (int i = -half; i <= half; i++) {

        int offset_x = (int)(px * i);
        int offset_y = (int)(py * i);

        SDL_RenderDrawLine(
            renderer,
            x1 + offset_x,
            y1 + offset_y,
            x2 + offset_x,
            y2 + offset_y
        );
    }
}

int renderer_init(Renderer *r, int width, int height) {
    r->width = width;
    r->height = height;
    r->pixels_per_meter = 120.0f;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(
            stderr,
            "SDL_Init failed: %s\n",
            SDL_GetError()
        );

        return 0;
    }

    r->window = SDL_CreateWindow(
        "Double Pendulum Environment",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        width,
        height,
        SDL_WINDOW_SHOWN
    );

    if (r->window == NULL) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 0;
    }

    r->renderer = SDL_CreateRenderer(
        r->window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

    if (r->renderer == NULL) {
        fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(r->window);
        SDL_Quit();
        return 0;
    }

    return 1;
}

void renderer_draw(Renderer *r, State *state) {
    SDL_Renderer *renderer = r->renderer;

    SDL_SetRenderDrawColor(renderer, 245, 245, 245, 255);
    SDL_RenderClear(renderer);

    float cart_x = state->x;
    int cart_center_x = r->width / 2 + (int)(cart_x * r->pixels_per_meter);
    int cart_center_y = GROUND_Y - CART_HEIGHT / 2;
    SDL_Rect cart = {
        cart_center_x - CART_WIDTH / 2,
        cart_center_y - CART_HEIGHT / 2,
        CART_WIDTH,
        CART_HEIGHT
    };

    SDL_SetRenderDrawColor(renderer, 40, 40, 40, 255);
    SDL_RenderFillRect(renderer, &cart);

    float pivot_x = (float)cart_center_x;
    float pivot_y = (float)( cart_center_y - CART_HEIGHT / 2);

    float L1 = env.L1 * r->pixels_per_meter;
    float L2 = env.L2 * r->pixels_per_meter;

    float t1 = state->t1;
    float t2 = state->t2;

    float pole1_x = pivot_x + L1 * sinf(t1);
    float pole1_y = pivot_y - L1 * cosf(t1);

    float pole2_x = pole1_x + L2 * sinf(t2);
    float pole2_y = pole1_y - L2 * cosf(t2);

    SDL_SetRenderDrawColor(renderer, 220, 60, 60, 255);
    draw_pole(renderer, (int)pivot_x, (int)pivot_y, (int)pole1_x, (int)pole1_y);
    SDL_SetRenderDrawColor(renderer, 50, 140, 220, 255);
    draw_pole(renderer, (int)pole1_x, (int)pole1_y, (int)pole2_x, (int)pole2_y);
    SDL_SetRenderDrawColor(renderer, 40, 180, 70, 255);

    SDL_Rect joint1 = {
        (int)pivot_x - JOINT_SIZE / 2,
        (int)pivot_y - JOINT_SIZE / 2,
        JOINT_SIZE,
        JOINT_SIZE
    };

    SDL_RenderFillRect(renderer, &joint1);

    SDL_Rect joint2 = {
        (int)pole1_x - JOINT_SIZE / 2,
        (int)pole1_y - JOINT_SIZE / 2,
        JOINT_SIZE,
        JOINT_SIZE
    };

    SDL_RenderFillRect(renderer, &joint2);

    SDL_Rect joint3 = {
        (int)pole2_x - JOINT_SIZE / 2,
        (int)pole2_y - JOINT_SIZE / 2,
        JOINT_SIZE,
        JOINT_SIZE
    };

    SDL_RenderFillRect(renderer, &joint3);
    SDL_SetRenderDrawColor(renderer, 40, 40, 40, 255);
    SDL_RenderDrawLine(renderer, 0, GROUND_Y, r->width, GROUND_Y);
    SDL_RenderPresent(renderer);
}

void renderer_destroy(Renderer *r) {
    if (r->renderer != NULL)
        SDL_DestroyRenderer(r->renderer);    
    if (r->window != NULL) 
        SDL_DestroyWindow(r->window);

    SDL_Quit();
}