#include "pendulum.h"

/**
 * Initialize / reset agent state
 */
void agent_init(Agent *agent) {
    agent->episode_time = 0.0f;
    agent->stable_time = 0.0f;
    agent->success_given = 0;
    agent->status = EPISODE_RUNNING;
}

/**
 * Reward function
 * Reward = f(Alive, Termination, Pole1 & Pole2 position, Cart velocity, Pole angular velocity, Cart position, Success)
 * Episode terminates when cart leaves rail limits or maximum episode time is reached
 */
float agent_get_reward(Agent *agent, const State *state, float dt) {

    // update episode time
    agent->episode_time += dt;

    // check termination
    int cart_outside = fabsf(state->x) > RAIL_LIMIT;
    int time_limit_reached = agent->episode_time >= MAX_EPISODE_TIME;
    int terminated = cart_outside || time_limit_reached;

    // 1. Alive
    // reward for every timestep the episode is running.
    float alive_reward = 0.0f;
    if (!terminated) alive_reward = ALIVE_REWARD;

    // 2. Termination penalty
    // penalize ending the episode when cart leaves the rail
    float termination_reward = 0.0f;
    if (terminated) {
        if (cart_outside) termination_reward = TERMINATION_REWARD;
        agent->status = EPISODE_TERMINATED;
    }

    // 3. Pole1 & Pole 2position
    // Penalize squared distance from upright to target theta=0
    float pole_position_reward = POLE_POSITION_WEIGHT * (SQR(state->t1) + SQR(state->t2));

    // 4. Cart velocity
    // Penalize cart movement to encourages the cart to eventually settle.
    float cart_velocity_reward = CART_VELOCITY_WEIGHT * fabsf(state->x_dot);

    // 5. Pole angular velocity
    // Penalize pole movement (kept small to encourage swing up)
    float pole_angular_velocity_reward = POLE_ANGULAR_VELOCITY_WEIGHT *(fabsf(state->t1_dot) + fabsf(state->t2_dot));

    // CHANGE TO THIS IF AGENT REFUSES TO SWING UP
    
    // float pole_angular_velocity_reward = 0.0f;
    // int near_upright = fabsf(state->t1) < (15.0f * PI / 180.0f) && fabsf(state->t2) < (15.0f * PI / 180.0f);
    // if (near_upright) pole_angular_velocity_reward =-0.05f * (fabsf(state->t1_dot) + fabsf(state->t2_dot));

    // 7. Cart position
    // Penalize moving too far from origin
    float cart_position_reward = CART_POSITION_WEIGHT * SQR(state->x / RAIL_LIMIT);

    // 8. Success
    // |t1|, |t2| < SUCCESS_ANGLE & |t1_dot|, |t2_dot| < SUCCESS_ANGULAR_VEL
    // simultaneously for SUCCESS_TIME, and then continue balancing

    float success_reward = 0.0f;
    int stable =
        fabsf(state->t1) < SUCCESS_ANGLE &&
        fabsf(state->t2) < SUCCESS_ANGLE &&
        fabsf(state->t1_dot) < SUCCESS_ANGULAR_VEL &&
        fabsf(state->t2_dot) < SUCCESS_ANGULAR_VEL;
    if (stable) {
        agent->stable_time += dt;
        if (agent->stable_time >= SUCCESS_TIME && !agent->success_given) {
            success_reward = SUCCESS_REWARD;
            agent->success_given = 1;
        }
    } else {
        // lost stability, so start counting again
        agent->stable_time = 0.0f;
    }

    // total reward
    float total_reward = alive_reward + termination_reward + pole_position_reward + cart_velocity_reward + pole_angular_velocity_reward + cart_position_reward + success_reward;

    return total_reward;
}

/**
 * Get current episode status
 */
EpisodeStatus agent_get_status(const Agent *agent) {
    return agent->status;
}
