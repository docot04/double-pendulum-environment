#include "pendulum.h"

EnvParams env = {
    .g = GRAVITY_ACCL,
    .M = CART_MASS,
    .max_F = CART_ACTUATION_FORCE,
    .m1 = POLE1_MASS,
    .m2 = POLE2_MASS,
    .L1 = POLE1_LENGTH,
    .L2 = POLE2_LENGTH
};

/**
 * Initialize the physical state
 * 
 * Starts the cart at rest and places the poles almost upright with a small random angular perturbation
 */
void physics_init(State *state) {

    // seed random number generator with time() + clock()
    srand((unsigned int)(time(NULL) ^ clock()));

    // initial cart state
    state->x = 0.0f;
    state->x_dot = 0.0f;

    // initial pole angle (start upright with a slighht angular perturbation
    // ([-0.02, 0.02] radians or approximately ±1.15 degrees))
    state->t1 = ((float)rand() / RAND_MAX - 0.5f) * 0.04f;
    state->t2 = ((float)rand() / RAND_MAX - 0.5f) * 0.04f;

    // initial pole angular velocity
    state->t1_dot = 0.0f;
    state->t2_dot = 0.0f;
}

/**
 * Calculate accelerations 
 */
static void get_accels(const State *state, float F, float *x_acc, float *t1_acc, float *t2_acc) {

    // trigonometric values for t1 and t2
    float st1 = sinf(state->t1);
    float ct1 = cosf(state->t1);
    float st2 = sinf(state->t2);
    float ct2 = cosf(state->t2);

    // trigonometric valyes for difference between pole angles
    float s_diff = sinf(state->t1 - state->t2);
    float c_diff = cosf(state->t1 - state->t2);

    /*
    * The equations of motion can be written as:
    * M * q_ddot = B
    * 
    * or:
    *
    *     | a  b  c |   | x_ddot  |   | u |
    *     | b  d  e | * | t1_ddot | = | v |
    *     | c  e  f |   | t2_ddot |   | w |
    */

    // MASS MATRIX COMPONENTS (Lagrangian formulation)

    // total translational mass
    // a = M + m1 + m2
    float a = env.M + env.m1 + env.m2;

    // Coupling between cart acceleration and pole 1
    // b = (m1 + m2) * L1 * cos(t1)
    float b = (env.m1 + env.m2) * env.L1 * ct1;

    // Coupling between cart acceleration and pole 2
    // c = m2 * L2 * cos(t2)
    float c = env.m2 * env.L2 * ct2;

    // Rotational inertia contribution of pole 1
    // d = (m1 + m2) * L1²
    float d = (env.m1 + env.m2) * env.L1 * env.L1;

    // Coupling between pole 1 and pole 2
    // e = m2 * L1 * L2 * cos(t1-t2)
    float e = env.m2 * env.L1 * env.L2 * c_diff;

    // Rotational inertia contribution of pole 2
    // f = m2 * L2²
    float f = env.m2 * env.L2 * env.L2;

    // FORCE VECTOR COMPONENTS

    // cart equation
    // u = F + [(m1 + m2) * L1 * t1_dot² * sin(t1)] + [m2 * L2 * t2_dot² * sin(t2)]
    // where term1 = force from agent, term2 = force from both poles, term3 = force from pole 2
    float u = F + (env.m1 + env.m2) * env.L1 * state->t1_dot * state->t1_dot * st1 + env.m2 * env.L2 * state->t2_dot * state->t2_dot * st2;

    // pole 1 equation
    // v = [(m1 + m2) * g * L1 * sin(t1)] - [m2 * L1 * L2 * t2_dot² * sin(t1 - t2)]
    // where term1 = gravity, term2 = interaction caused by rotational motion of pole 2
    float v = (env.m1 + env.m2) * env.g * env.L1 * st1 - env.m2 * env.L1 * env.L2 * state->t2_dot * state->t2_dot * s_diff;

    // pole 2 equation
    // w = [m2 * g * L2 * sin(t2)] + [m2 * L1 * L2 * t1_dot² * sin(t1 - t2)]
    // where term1 = gravity, term2 = interaction with pole 1
    float w = env.m2 * env.g * env.L2 * st2 + env.m2 * env.L1 * env.L2 * state->t1_dot * state->t1_dot * s_diff;

    // determinant of mass matrix: det(M)
    // det(M) = a(df - e²) - b(bf - ce) + c(be - cd)
    float det = a * (d*f - e*e) - b * (b*f - c*e) + c * (b*e - c*d);

    // cart acceleration
    // x_ddot = [ u(df - e²) - b(vf - we) + c(ve - wd) ] / det(M)
    *x_acc = (u * (d*f - e*e) - b * (v*f - w*e) + c * (v*e - w*d)) / det;
    
    // pole 1 angular acceleration
    // t1_ddot = [ a(vf - we) - u(bf - ce) + c(bw - cv) ] / det(M)
    *t1_acc =(a * (v*f - w*e) - u * (b*f - c*e) + c * (b*w - c*v)) / det;
    
    // pole 2 angular acceleration
    // t2_ddot = [ a(dw - ev) - b(bw - cv) + u(be - cd) ] / det(M)
    *t2_acc =(a * (d*w - e*v) - b * (b*w - c*v) + u * (b*e - c*d)) / det;
}

/**
 * Advance the physics simulation by dt seconds
 * 
 * 1. Input_normalized in range [-1, 1] converted into horizontal force (F = input_normalized * max_F)
 * 2. calculate accelerations
 * 3. integrate acceleration -> velocity
 * 4. apply slight numerical damping
 * 5. integrate velocity -> position
 * 6. normalize angles
 */
void physics_step(State *state, float input_normalized,float dt) {

    // 1. input converted into horizontal force
    // F = input_normalized * max_F
    float F = input_normalized * env.max_F;

    // we divide the entire dt calculation into 15 smaler steps to perform integrations
    int substeps = 15;
    float h = dt / substeps;

    for (int i = 0; i < substeps; i++) {
        float x_acc, t1_acc, t2_acc;

        // 2. calculate accelerations (using current state and applied force)
        // x_acc = x_ddot, t1_acc  = t1_ddot, t2_acc  = t2_ddot
        get_accels(state, F, &x_acc, &t1_acc, &t2_acc);

        // 3. integrate acceleration -> velocity (using euler integration)
        // velocity = old velocity + acceleration * dt (here time = h)
        state->x_dot = state->x_dot + x_acc * h;
        state->t1_dot = state->t1_dot + t1_acc * h;
        state->t2_dot = state->t2_dot + t2_acc * h;

        // 4. applying slight numerical damping
        // (added to prevent numerical errors from continuously injecting energy into the simulation.)
        state->x_dot = state->x_dot * 0.999f;
        state->t1_dot = state->t1_dot * 0.999f;
        state->t2_dot = state->t2_dot * 0.999f;

        // 5. integrate velocity -> position (using euler integration)
        // position = old position + new velocity * dt (here time = h)
        state->x = state->x + state->x_dot * h;
        state->t1 = state->t1 + state->t1_dot * h;
        state->t2 = state->t2 + state->t2_dot * h;

        // 6. normalize angles
        // (keep angles within [-Pi, +PI])
        while (state->t1 > PI) state->t1 -= 2.0f * PI;
        while (state->t1 < -PI) state->t1 += 2.0f * PI;
        while (state->t2 > PI) state->t2 -= 2.0f * PI;
        while (state->t2 < -PI)state->t2 += 2.0f * PI;
    }
}
