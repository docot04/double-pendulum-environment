#include "physics.h"

EnvParams env = {
    .M = 2.5f,
    .m1 = 1.0f,
    .m2 = 0.5f,
    .L1 = 1.0f,
    .L2 = 1.0f,
    .g = 10.0f,
    .max_F = 25.0f
};

void physics_init(State* s) {

    // seed using time and clock for randomness
    srand((unsigned int)(time(NULL) ^ clock()));
    
    s->x = 0.0f;
    s->x_dot = 0.0f;
    
    // start upright with a slight random perturbation (~1-2 degrees) seeded by time
    s->t1 = ((float)rand() / RAND_MAX - 0.5f) * 0.04f;
    s->t2 = ((float)rand() / RAND_MAX - 0.5f) * 0.04f;
    
    s->t1_dot = 0.0f;
    s->t2_dot = 0.0f;
}

static void get_accels(const State* s, float F, float* x_acc, float* t1_acc, float* t2_acc) {
    // Note: 0 is upright, so we offset trig functions by sin/cos adjustments if needed,
    float st1 = sinf(s->t1);
    float ct1 = cosf(s->t1);
    float st2 = sinf(s->t2);
    float ct2 = cosf(s->t2);
    float s_diff = sinf(s->t1 - s->t2);
    float c_diff = cosf(s->t1 - s->t2);
    
    // mass matrix M components (Lagrangian formulation)
    float a = env.M + env.m1 + env.m2;
    float b = (env.m1 + env.m2) * env.L1 * ct1;
    float c = env.m2 * env.L2 * ct2;
    float d = (env.m1 + env.m2) * env.L1 * env.L1;
    float e = env.m2 * env.L1 * env.L2 * c_diff;
    float f = env.m2 * env.L2 * env.L2;
    
    // force vector B components (including gravity torque pointing upright)
    float u = F + (env.m1 + env.m2) * env.L1 * s->t1_dot * s->t1_dot * st1 + env.m2 * env.L2 * s->t2_dot * s->t2_dot * st2;
    float v = (env.m1 + env.m2) * env.g * env.L1 * st1 - env.m2 * env.L1 * env.L2 * s->t2_dot * s->t2_dot * s_diff;
    float w = env.m2 * env.g * env.L2 * st2 + env.m2 * env.L1 * env.L2 * s->t1_dot * s->t1_dot * s_diff;
    float det = a* (d*f - e*e) - b* (b*f - c*e) + c* (b*e - c*d);
    *x_acc  = (u* (d*f - e*e) - b* (v*f - w*e) + c* (v*e - w*d)) / det;
    *t1_acc = (a* (v*f - w*e) - u* (b*f - c*e) + c* (b*w - c*v)) / det;
    *t2_acc = (a* (d*w - e*v) - b* (b*w - c*v) + u* (b*e - c*d)) / det;
}

void physics_step(State* s, float input_normalized, float dt) {
    float F = input_normalized * env.max_F;
    
    // micro-substepping for numerical stability
    int substeps = 15;
    float h = dt / substeps;
    
    for (int i = 0; i < substeps; i++) {
        float x_acc, t1_acc, t2_acc;
        get_accels(s, F, &x_acc, &t1_acc, &t2_acc);
        s->x_dot  += x_acc * h;
        s->t1_dot += t1_acc * h;
        s->t2_dot += t2_acc * h;

        // slight damping to prevent numerical energy explosion
        s->x_dot  *= 0.999f;
        s->t1_dot *= 0.999f;
        s->t2_dot *= 0.999f;
        
        s->x  += s->x_dot * h;
        s->t1 += s->t1_dot * h;
        s->t2 += s->t2_dot * h;
        
        // normalize angles between -PI and PI
        while (s->t1 > PI)  s->t1 -= 2.0f * PI;
        while (s->t1 < -PI) s->t1 += 2.0f * PI;
        while (s->t2 > PI)  s->t2 -= 2.0f * PI;
        while (s->t2 < -PI) s->t2 += 2.0f * PI;
    }
}