#include "pendulum.h"

// make stdin non blocking (for interface mode)
static int set_stdin_nonblocking() {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags == -1) return 0;
    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) return 0;
    return 1;
}

/**
 * read from stdin without blocking
 * 1:  a complete value was received
 * 0:  no complete value available
 * -1: error
 */
static int read_input(float *value) {
    static char buffer[128];
    static int buffer_len = 0;
    char temp[64];
    ssize_t n = read(STDIN_FILENO, temp, sizeof(temp));
    if (n == 0)return -1;
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    int got_value = 0;
    for (ssize_t i = 0; i < n; i++) {
        char c = temp[i];
        if (c == '\n') {
            buffer[buffer_len] = '\0';
            if (buffer_len > 0) {
                char *end;
                float v = strtof(buffer, &end);
                if (end != buffer) {
                    *value = v;
                    got_value = 1;
                }
            }
            buffer_len = 0;
        }
        else {
            if (buffer_len < (int)sizeof(buffer) - 1) buffer[buffer_len++] = c;
            else buffer_len = 0;
        }
    }
    return got_value;
}

// Protocol for stdout
// FORMAT: OK x x_dot t1 t1_dot t2 t2_dot reward complete
static void print_state(const State *state, float reward, int complete) {
    printf(
        "OK %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d\n",
        state->x,
        state->x_dot,
        state->t1,
        state->t1_dot,
        state->t2,
        state->t2_dot,
        reward,
        complete
    );
    fflush(stdout);
}

int main(int argc, char *argv[]) {
    RunMode mode;
    int use_renderer = 1;
    // parse CLA
    if (argc != 2) {
        printf("ERR\n");
        fflush(stdout);
        return 1;
    }
    if (strcmp(argv[1], "manual") == 0) mode = MODE_MANUAL;
    else if (strcmp(argv[1], "interface") == 0) mode = MODE_INTERFACE;
    else if (strcmp(argv[1], "interface-headless") == 0) {
        mode = MODE_INTERFACE_HEADLESS;
        use_renderer = 0;
    } else {
        printf("ERR\n");
        fflush(stdout);
        return 1;
    }

    // initialize state
    State state;
    Agent agent;
    physics_init(&state);
    agent_init(&agent);

    // initialize SDL
    Renderer renderer;
    if (use_renderer) {
        if (!renderer_init(&renderer, 900, 600)) {
            printf("ERR\n");
            fflush(stdout);
            return 1;
        }
    }

    // interface mode (make stdin non blocking)
    if (mode == MODE_INTERFACE || mode == MODE_INTERFACE_HEADLESS) {
        if (!set_stdin_nonblocking()) {
            if (use_renderer) renderer_destroy(&renderer);
            printf("ERR\n");
            fflush(stdout);
            return 1;
        }
    }

    // simulation timing
    struct timespec sleep_time;
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 1000000000L / FPS;

    // current action
    float current_input = 0.0f;
    int running = 1;

    // main loop
    while (running) {

        // manual mode
        if (mode == MODE_MANUAL) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) running = 0;
                else if (event.type == SDL_KEYDOWN) {
                    if (event.key.keysym.sym == SDLK_s) {
                        physics_init(&state);
                        agent_init(&agent);
                        current_input = 0.0f;
                    }
                }
            }
            const Uint8 *keys = SDL_GetKeyboardState(NULL);
            current_input = 0.0f;
            if (keys[SDL_SCANCODE_A]) current_input = -1.0f;
            if (keys[SDL_SCANCODE_D]) current_input = 1.0f;
        }


        // interface mode (3=quit, 2=reset after completion)
        if (mode == MODE_INTERFACE || mode == MODE_INTERFACE_HEADLESS) {
            float received_input;
            int result = read_input(&received_input);
            if (result == -1) {
                printf("ERR\n");
                fflush(stdout);
                break;
            }
            if (result == 1) {
                if (received_input == 3.0f) running = 0;
                else if (received_input == 2.0f && agent_get_status(&agent) != EPISODE_RUNNING) {
                    physics_init(&state);
                    agent_init(&agent);
                    current_input = 0.0f;
                }
                else if (received_input >= -1.0f && received_input <= 1.0f) {
                    // keep action until another arrives and continue simulation
                    current_input = received_input;
                }
            }
        }

        // stop physics after episode completion
        if (agent_get_status(&agent) != EPISODE_RUNNING) {
            if (use_renderer) renderer_draw(&renderer, &state);
            nanosleep(&sleep_time, NULL);
            continue;
        }

        // increment simulation step
        physics_step(&state, current_input, DT);

        // reward function
        float reward = agent_get_reward(&agent, &state, DT);

        // check completion
        int complete = agent_get_status(&agent) != EPISODE_RUNNING;

        // render frame
        if (use_renderer) renderer_draw(&renderer, &state);

        // send state via stdin
        print_state(&state, reward, complete);

        // sleep until next step
        nanosleep(&sleep_time, NULL);
    }

    // SDL cleanup
    if (use_renderer) renderer_destroy(&renderer);
    return 0;
}