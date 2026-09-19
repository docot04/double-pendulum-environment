#ifndef PENDULUM_H
#define PENDULUM_H

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <errno.h>

// math
#define PI 3.14159265358979323846f

// simulation
#define FPS 30
#define DT (1.0f / FPS)

// environment
#define GRAVITY_ACCL 9.81f
#define CART_MASS 2.5f
#define CART_ACTUATION_FORCE 25.0f
#define POLE1_MASS 1.0f
#define POLE2_MASS 0.5f
#define POLE1_LENGTH 1.0F
#define POLE2_LENGTH 1.0F

// render
#define CART_WIDTH       40
#define CART_HEIGHT      20
#define JOINT_SIZE       16
#define POLE_THICKNESS   6
#define GROUND_Y         450


/**
 * Environment state
 * 
 * x: cart horizontal position
 * x_dot: cart horizontal velocity
 * t1, t2: pole angle (0 = upright)
 * t1_dot, t2_dot: pole angular velocity
 */
typedef struct {
    float x;
    float x_dot;
    float t1;
    float t2;
    float t1_dot;
    float t2_dot;
} State;

/**
 * Environment parameters
 * 
 * g: gravitational acceleration
 * M: cart mass
 * max_F: cart actuator force
 * m1, m2: pole mass
 * L1, L2: pole length
 */
typedef struct {
    float g;
    float M;
    float max_F;
    float m1;
    float m2;
    float L1;
    float L2;
} EnvParams;
extern EnvParams env;

/**
 * SDL2 Renderer
 */
typedef struct {
    SDL_Window *window;
    SDL_Renderer *renderer;
    int width;
    int height;
    float pixels_per_meter;
} Renderer;

void physics_init(State *state);
void physics_step(State *state, float input_normalized, float dt);
int renderer_init(Renderer *r, int width, int height);
void renderer_draw(Renderer *r, State *state);
void renderer_destroy(Renderer *r);

#endif