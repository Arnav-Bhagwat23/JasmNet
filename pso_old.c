#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

#define NUM_DRONES 10
#define DIMENSIONS 3
#define MAX_ITERATIONS 50

typedef struct {
    double position[DIMENSIONS];
    double velocity[DIMENSIONS];
    double best_position[DIMENSIONS];
    double best_score;
} Drone;

double random_double(double min, double max) {
    return min + (double)rand() / RAND_MAX * (max - min);
}

double fitness(double position[DIMENSIONS], double target[DIMENSIONS]) {
    double sum = 0.0;
    for (int i = 0; i < DIMENSIONS; i++) {
        sum += pow(position[i] - target[i], 2);
    }
    return sqrt(sum);
}

void initialize_drones(Drone drones[], double bounds[2], double target[DIMENSIONS]) {
    for (int i = 0; i < NUM_DRONES; i++) {
        for (int j = 0; j < DIMENSIONS; j++) {
            drones[i].position[j] = random_double(bounds[0], bounds[1]);
            drones[i].velocity[j] = random_double(-1, 1);
            drones[i].best_position[j] = drones[i].position[j];
        }
        drones[i].best_score = fitness(drones[i].position, target);
    }
}

void update_drones(Drone drones[], double bounds[2], double target[DIMENSIONS], double global_best_position[DIMENSIONS], double *global_best_score) {
    double w = 0.5, c1 = 1.5, c2 = 1.5;

    for (int i = 0; i < NUM_DRONES; i++) {
        double current_score = fitness(drones[i].position, target);

        // Update personal best
        if (current_score < drones[i].best_score) {
            drones[i].best_score = current_score;
            for (int j = 0; j < DIMENSIONS; j++) {
                drones[i].best_position[j] = drones[i].position[j];
            }
        }

        // Update global best
        if (current_score < *global_best_score) {
            *global_best_score = current_score;
            for (int j = 0; j < DIMENSIONS; j++) {
                global_best_position[j] = drones[i].position[j];
            }
        }

        // Update velocity and position
        for (int j = 0; j < DIMENSIONS; j++) {
            double r1 = random_double(0, 1);
            double r2 = random_double(0, 1);

            drones[i].velocity[j] = w * drones[i].velocity[j]
                + c1 * r1 * (drones[i].best_position[j] - drones[i].position[j])
                + c2 * r2 * (global_best_position[j] - drones[i].position[j]);

            drones[i].position[j] += drones[i].velocity[j];

            // Enforce bounds
            if (drones[i].position[j] < bounds[0]) drones[i].position[j] = bounds[0];
            if (drones[i].position[j] > bounds[1]) drones[i].position[j] = bounds[1];
        }
    }
}

void optimize(double bounds[2], double target[DIMENSIONS]) {
    Drone drones[NUM_DRONES];
    double global_best_position[DIMENSIONS];
    double global_best_score = DBL_MAX;

    initialize_drones(drones, bounds, target);

    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        update_drones(drones, bounds, target, global_best_position, &global_best_score);

        printf("Iteration %d/%d, Best Fitness: %.4f, Global Best Position: (", iter + 1, MAX_ITERATIONS, global_best_score);
        for (int j = 0; j < DIMENSIONS; j++) {
            printf("%.2f", global_best_position[j]);
            if (j < DIMENSIONS - 1) printf(", ");
        }
        printf(")\n");
    }
}

int main() {
    srand(time(NULL));

    double bounds[2] = {0.0, 100.0};
    double target[DIMENSIONS] = {50.0, 50.0, 50.0};

    optimize(bounds, target);

    return 0;
}
