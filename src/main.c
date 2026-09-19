#include "pendulum.h"

int main() {
    State state;

    // initialize the physics environment
    physics_init(&state);

    // initialize Renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 900, 600))
        return 1;

    // make stdin non blocking to continue the simulation even if no action is passed
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);

    if (flags == -1) {
        perror("fcntl(F_GETFL)");
        return 1;
    }

    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl(F_SETFL)");
        return 1;
    }

    struct timespec sleep_time;
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 1000000000L / FPS;

    // current action (if no action received for a frame, set 0)
    float input = 0.0f;
    char buffer[32];

    // main simulation loop
    while (1) {

        // read action from non-blocking stdin, returns immediately if no data received
        ssize_t n = read(STDIN_FILENO, buffer, sizeof(buffer) - 1);
        if (n > 0) {

            // convert received bytes into a C string
            buffer[n] = '\0';
            char *end;
            float new_input = strtof(buffer, &end);

            // only accept the input if strtof() actually found a valid number
            if (end != buffer) {
                input = new_input;

                // clamp action to [-1, +1]
                if (input > 1.0f) input = 1.0f;
                if (input < -1.0f) input = -1.0f;
            }
        }
        else if (n == -1) {
            // not a fatal error
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("read");
                break;
            }

            // if no input, input = 0
            input = 0.0f;
        } else if (n == 0) {
            // terminate environment
            fprintf(stderr, "stdin closed\n");
            break;
        }

        // advance physics by 1/fps of a second
        physics_step(&state, input, DT);
        renderer_draw(&renderer, &state);

        // send state to stdout
        // format: x x_dot t1 t1_dot t2 t2_dot
        printf("%.6f %.6f %.6f %.6f %.6f %.6f\n", state.x, state.x_dot, state.t1, state.t1_dot, state.t2, state.t2_dot);
        fflush(stdout);

        // pause for 1/30 seconds
        nanosleep(&sleep_time, NULL);
    }
    renderer_destroy(&renderer);
    return 0;
}