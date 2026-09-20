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
#include <string.h>

// math
#define PI 3.14159265358979323846f
#define SQR(x) ((x) * (x))

// simulation
#define FPS 30
#define DT (1.0f / FPS)

// environment
#define GRAVITY_ACCL 9.81f
#define CART_MASS 1.5f
#define CART_ACTUATION_FORCE 20.0f
#define POLE1_MASS 0.5f
#define POLE2_MASS 0.25f
#define POLE1_LENGTH 0.8F
#define POLE2_LENGTH 0.8F

// task parameters
#define RAIL_LIMIT 5.0f
#define MAX_EPISODE_TIME 10.0f
#define SUCCESS_ANGLE (5.0f * PI / 180.0f)
#define SUCCESS_ANGULAR_VEL 1.0f
#define SUCCESS_TIME 2.0f
#define ALIVE_REWARD 2.0f
#define TERMINATION_REWARD -5.0f
#define POLE_POSITION_WEIGHT -0.2f
#define CART_VELOCITY_WEIGHT -0.05f
#define POLE_ANGULAR_VELOCITY_WEIGHT -0.05f
#define CART_POSITION_WEIGHT -0.05f
#define SUCCESS_REWARD 10.0f

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
 * Status of the episode
 */
typedef enum {
    EPISODE_RUNNING,
    EPISODE_TERMINATED
} EpisodeStatus;


/**
 * Task state to determine reward and episode completion
 */
typedef struct {
    float episode_time;
    float stable_time;
    int success_given; // prevent success reward from given multiple times in same episode
    EpisodeStatus status;
} Agent;

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

/**
 * main running modes
 */
typedef enum {
    MODE_MANUAL,
    MODE_INTERFACE,
    MODE_INTERFACE_HEADLESS
} RunMode;


void physics_init(State *state);
void physics_step(State *state, float input_normalized, float dt);

void agent_init(Agent *agent);
float agent_get_reward(Agent *agent, const State *state, float dt);
EpisodeStatus agent_get_status(const Agent *agent);

int renderer_init(Renderer *r, int width, int height);
void renderer_draw(Renderer *r, State *state);
void renderer_destroy(Renderer *r);

#endif