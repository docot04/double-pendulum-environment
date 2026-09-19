#ifndef PHYSICS_H
#define PHYSICS_H
#include <math.h>
#include <stdlib.h>
#include <time.h>
#define PI 3.14159265358979323846f

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
void physics_init(State *state);
void physics_step(State *state, float input_normalized, float dt);

#endif