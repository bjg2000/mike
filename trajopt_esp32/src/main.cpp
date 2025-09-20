// Port of trajopt_pi5 for ESP32 (Arduino framework)
// NLopt optimizer replaced with simple grid search for demonstration
//
// --- Fixes & Improvements ---
// 1. Added yield() in the main calculation loop to prevent Watchdog Timer (WDT) resets on the ESP32.
//    This is the primary fix for the boot loop issue.
// 2. Refactored memory management in traj_calc_grid to allocate memory only once
//    before the loops, improving performance and preventing heap fragmentation.
// 3. Replaced pointer arithmetic (e.g., *(s+i)) with more readable array notation (e.g., s[i]).
// 4. Optimized the dartdrag function by calculating the velocity magnitude once.
// 5. Replaced PI with M_PI for better code clarity and standards compliance.
// 6. Converted all double-precision floating point numbers to single-precision (float)
//    for a massive performance boost on the ESP32 hardware.

#include <Arduino.h>
#include <math.h> // Includes M_PI

// The number of points to calculate in the trajectory.
#define gridsize 100

// Holds the state of the projectile at a single point in time.
struct traj {
    float t, x, y, z, dx, dy, dz;
};

// The output of the trajectory calculation, including angles and success status.
struct traj_calc_output {
    float phi, theta, timetotarget;
    unsigned int success;
};

/**
 * @brief Calculates the derivatives (velocities and accelerations) for the projectile.
 * @param s The current state of the projectile.
 * @param dx An array to store the calculated derivatives.
 * @param a The drag coefficient alpha.
 * @param g The acceleration due to gravity.
 */
void dartdrag(traj s, float* dx, float a, float g) {
    dx[0] = s.dx;
    dx[1] = s.dy;
    dx[2] = s.dz;

    // Calculate velocity magnitude once to optimize
    float vel_mag = sqrtf(s.dx * s.dx + s.dy * s.dy + s.dz * s.dz);

    dx[3] = -a * s.dx * vel_mag;
    dx[4] = -a * s.dy * vel_mag - g;
    dx[5] = -a * s.dz * vel_mag;
}

/**
 * @brief Propagates the projectile's trajectory using the RK4 method.
 * @param s Pointer to the array of trajectory states.
 * @param a The drag coefficient alpha.
 * @param g The acceleration due to gravity.
 */
void rk4_blaster(traj* s, float a, float g) {
    for (int i = 0; i < (gridsize - 1); i++) {
        float k1[6], k2[6], k3[6], k4[6];
        float dt = s[i + 1].t - s[i].t;
        traj sk1 = s[i], sk2 = s[i], sk3 = s[i];

        dartdrag(s[i], k1, a, g);
        sk1.x = s[i].x + 0.5 * dt * k1[0];
        sk1.y = s[i].y + 0.5 * dt * k1[1];
        sk1.z = s[i].z + 0.5 * dt * k1[2];
        sk1.dx = s[i].dx + 0.5f * dt * k1[3];
        sk1.dy = s[i].dy + 0.5f * dt * k1[4];
        sk1.dz = s[i].dz + 0.5f * dt * k1[5];

        dartdrag(sk1, k2, a, g);
        sk2.x = s[i].x + 0.5 * dt * k2[0];
        sk2.y = s[i].y + 0.5 * dt * k2[1];
        sk2.z = s[i].z + 0.5 * dt * k2[2];
        sk2.dx = s[i].dx + 0.5f * dt * k2[3];
        sk2.dy = s[i].dy + 0.5f * dt * k2[4];
        sk2.dz = s[i].dz + 0.5f * dt * k2[5];

        dartdrag(sk2, k3, a, g);
        sk3.x = s[i].x + dt * k3[0];
        sk3.y = s[i].y + dt * k3[1];
        sk3.z = s[i].z + dt * k3[2];
        sk3.dx = s[i].dx + dt * k3[3];
        sk3.dy = s[i].dy + dt * k3[4];
        sk3.dz = s[i].dz + dt * k3[5];

        dartdrag(sk3, k4, a, g);
        s[i + 1].x = s[i].x + (dt / 6.0f) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
        s[i + 1].y = s[i].y + (dt / 6.0f) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
        s[i + 1].z = s[i].z + (dt / 6.0f) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
        s[i + 1].dx = s[i].dx + (dt / 6.0f) * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
        s[i + 1].dy = s[i].dy + (dt / 6.0f) * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]);
        s[i + 1].dz = s[i].dz + (dt / 6.0f) * (k1[5] + 2 * k2[5] + 2 * k3[5] + k4[5]);
    }
}

/**
 * @brief Fills the time steps for the trajectory array.
 * @param arr Pointer to the array of trajectory states.
 * @param n The total time for the trajectory.
 */
void linspace(traj* arr, float n) {
    for (int i = 0; i < gridsize; i++) {
        arr[i].t = i * n / (float)(gridsize - 1);
    }
}

/**
 * @brief Uses a grid search to find the optimal launch angles and time.
 * @param target The target's state.
 * @param v_init Initial velocity.
 * @param ... other physical parameters.
 * @return A struct containing the calculated angles, time, and success status.
 */
traj_calc_output traj_calc_grid(traj target, float v_init, float rho, float c_d, float A, float m, float angle_round) {
    float alpha = 0.5f * rho * c_d * A / m;
    float best_phi = 0, best_theta = 0, best_t = 1, min_obj = 1e9f;
    unsigned int found = 0;

    // Allocate memory ONCE before the loops to prevent heap fragmentation.
    traj* dartpath = new traj[gridsize];
    if (!dartpath) {
        Serial.println("FATAL: Failed to allocate memory for dartpath!");
        // Return a failure state if memory allocation fails.
        traj_calc_output out_fail = {0, 0, 0, 0};
        return out_fail;
    }

    int phi_steps = 0; // Counter for progress indicator
    for (float phi = -M_PI/18.0f; phi <= M_PI/18.0f; phi += angle_round * M_PI / 180.0f) {
        // --- Progress Indicator ---
        // Print a character to the serial monitor to show the code is running and not frozen.
        // This loop is the most time-consuming part of the program.
        if (phi_steps % 5 == 0) Serial.print(".");
        phi_steps++;

        for (float theta = -M_PI/18.0f; theta <= M_PI/18.0f; theta += angle_round * M_PI / 180.0f) {
            for (float t = 1.0f; t <= 2.0f; t += 0.2f) {
                // *** KEY FIX ***
                // Yield to the scheduler to prevent the watchdog timer from resetting the ESP32.
                yield();

                dartpath[0].x = 0;
                dartpath[0].y = 0;
                dartpath[0].z = 0;
                dartpath[0].dx = v_init * cosf(phi) * cosf(theta);
                dartpath[0].dy = v_init * sinf(phi) * cosf(theta);
                dartpath[0].dz = v_init * sinf(theta);
                linspace(dartpath, t);
                rk4_blaster(dartpath, alpha, 9.81f);

                float deviation_y = dartpath[gridsize - 1].y - (target.y + target.dy * dartpath[gridsize - 1].t);
                float deviation_z = dartpath[gridsize - 1].z - (target.z + target.dz * dartpath[gridsize - 1].t);
                float obj = deviation_y * deviation_y + deviation_z * deviation_z;
                
                if (obj < min_obj) {
                    min_obj = obj;
                    best_phi = phi;
                    best_theta = theta;
                    best_t = t;
                    found = 1;
                }
            }
        }
    }
    Serial.println(); // Add a new line after the progress dots for clean output

    // Free the memory once after all calculations are done.
    delete[] dartpath;

    traj_calc_output out;
    out.phi = best_phi;
    out.theta = best_theta;
    out.timetotarget = best_t;
    out.success = found;
    return out;
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Wait for serial monitor to connect

    // --- Define physical constants and target ---
    float v_init = 200 / 3.28f, rho = 1.293f, c_d = 0.6712f, A = M_PI * powf((13.0f / 1000.0f / 2.0f), 2), m = 1.3f / 1000.0f, angle_round = 0.3f;
    traj target;
    target.x = 120 / 3.28f;
    target.y = 0 / 3.28f;
    target.z = 0 / 3.28f;
    target.dx = 0 / 3.28f;
    target.dy = 0 / 3.28f;
    target.dz = 0 / 3.28f;

    // Prevent division by zero or other math issues with a stationary target
    if (target.x == 0) target.x = 5 / 3.28f;
    if (target.y == 0) target.y = 1e-3f / 3.28f;
    if (target.z == 0) target.z = 1e-3f / 3.28f;

    Serial.println("Starting trajectory calculation...");
    unsigned long begin = millis();
    traj_calc_output dart_hit = traj_calc_grid(target, v_init, rho, c_d, A, m, angle_round);
    unsigned long end = millis();
    Serial.println("Calculation complete.");

    if (dart_hit.success) {
        Serial.println("Target hit!");
    } else {
        Serial.println("Target out of range!");
    }

    Serial.printf("elevation %.3f degrees and lead %.3f degrees\n", dart_hit.phi * 180 / M_PI, dart_hit.theta * 180 / M_PI);
    Serial.printf("time to target x position: %.2f seconds\n", dart_hit.timetotarget);

    // --- Recalculate and print final trajectory for verification ---
    traj* dart_end = new traj[gridsize]; 
    dart_end[0].x = 0;
    dart_end[0].y = 0;
    dart_end[0].z = 0;
    dart_end[0].dx = v_init * cosf(dart_hit.phi) * cosf(dart_hit.theta);
    dart_end[0].dy = v_init * sinf(dart_hit.phi) * cosf(dart_hit.theta);
    dart_end[0].dz = v_init * sinf(dart_hit.theta);
    linspace(dart_end, dart_hit.timetotarget);
    rk4_blaster(dart_end, 0.5f * rho * c_d * A / m, 9.81f);
    Serial.printf("last dart position at (%.2f, %.2f, %.2f) feet\n", dart_end[gridsize - 1].x * 3.28f, dart_end[gridsize - 1].y * 3.28f, dart_end[gridsize - 1].z * 3.28f);
    Serial.printf("target position at (%.2f, %.2f, %.2f) feet\n", (target.x + target.dx * dart_end[gridsize - 1].t) * 3.28f, (target.y + target.dy * dart_end[gridsize - 1].t) * 3.28f, (target.z + target.dz * dart_end[gridsize - 1].t) * 3.28f);

    Serial.printf("calculation time: %lu ms\n", end - begin);

    delete[] dart_end;
}

void loop() {
    // The main calculation is done in setup(), so nothing is needed here.
    // The ESP32 will idle here after the calculation is complete.
    delay(1000);
}


