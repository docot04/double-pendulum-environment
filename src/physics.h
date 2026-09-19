#ifndef PHYSICS_H
#define PHYSICS_H
#define PI 3.14159265358979323846
#include <math.h>
#include <stdlib.h>
#include <time.h>

typedef struct {
    float x;      // cart position on track
    float x_dot;  // cart velocity

    float t1;     // angle of pole 1 (0 = perfectly upright)
    float t1_dot; // angular velocity of pole 1

    float t2;     // angle of pole 2 (0 = perfectly upright)
    float t2_dot; // angular velocity of pole 2
} State;

typedef struct {
    float g;      // gravity (10.0 m/s^2)
    
    float M;      // cart mass (1.0 kg)
    float max_F;  // max force (actuator strength) (2.0 N)

    float m1;     // pole 1 mass (1.0 kg)
    float L1;     // pole 1 length (1.0 m)
    
    float m2;     // pole 2 mass (0.5 kg)
    float L2;     // pole 2 length (1.0 m)
} EnvParams;

extern EnvParams env;

void physics_init(State* s);
void physics_step(State* s, float input_force, float dt);

#endif