#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// --- Pin definitions ---
#define STEP_PIN_R 2
#define DIR_PIN_R 4
#define STEP_PIN_L 18
#define DIR_PIN_L 5

#define STEPS_PER_REV 1600   // 1.8° step, 8 microstepping

// --- Internal state ---
static unsigned long last_step_left = 0;
static unsigned long last_step_right = 0;

static unsigned long step_interval_left = 1000;
static unsigned long step_interval_right = 1000;

static unsigned long last_measure = 0;
static unsigned long step_count_left = 0;
static unsigned long step_count_right = 0;
static float vel_left = 0;
static float vel_right = 0;
static uint8_t measure_counter = 0;

// --- Init ---
inline void motor_driver_init() {
    pinMode(STEP_PIN1, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    pinMode(STEP_PIN2, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);

    digitalWrite(DIR_PIN1, LOW);
    digitalWrite(DIR_PIN2, LOW);

    last_measure = millis();
}

// Convert commanded velocity [-2, 2] rev/sec into step intervals
inline void set_velocity(double vel, unsigned long &interval, uint8_t dirPin) {
    if (vel > 0) digitalWrite(dirPin, HIGH);
    else digitalWrite(dirPin, LOW);

    double steps_per_sec = fabs(vel) * STEPS_PER_REV;
    if (steps_per_sec < 1) steps_per_sec = 1; // prevent div by 0
    interval = (unsigned long)(1000000.0 / steps_per_sec / 2.0); // HIGH+LOW = 1 step
}

// --- Main driver loop ---
inline void motor_driver_loop(double left_vel, double right_vel) {
    // Update commanded velocities
    set_velocity(left_vel, step_interval_left, DIR_PIN1);
    set_velocity(right_vel, step_interval_right, DIR_PIN2);

    unsigned long now = micros();

    // --- Left motor ---
    if (now - last_step_left >= step_interval_left) {
        digitalWrite(STEP_PIN1, HIGH);
        delayMicroseconds(2);
        digitalWrite(STEP_PIN1, LOW);
        last_step_left = now;
        step_count_left++;
    }

    // --- Right motor ---
    if (now - last_step_right >= step_interval_right) {
        digitalWrite(STEP_PIN2, HIGH);
        delayMicroseconds(2);
        digitalWrite(STEP_PIN2, LOW);
        last_step_right = now;
        step_count_right++;
    }

    // --- Feedback update every 8 measurements ---
    if (millis() - last_measure >= 50) {
        measure_counter++;
        if (measure_counter >= 8) {
            unsigned long dt = millis() - last_measure;
            vel_left  = (step_count_left  * 1000.0) / dt; // steps/sec
            vel_right = (step_count_right * 1000.0) / dt; // steps/sec
            step_count_left = 0;
            step_count_right = 0;
            last_measure = millis();
            measure_counter = 0;
        }
    }
}

// --- Getters ---
inline float get_left_velocity()  { return vel_left; }
inline float get_right_velocity() { return vel_right; }

#endif
