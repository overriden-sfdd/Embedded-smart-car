#ifndef CAR_HPP_
#define CAR_HPP_

#include <Arduino.h>

// Simple structure to hold Wheel data. Might be redundant.
struct Wheel
{
    float diameter;
    uint32_t pulses_per_rotation;
};

// Class holding all data and functions for movement.
class Motor
{
public:
    Motor(Wheel, uint8_t, uint8_t,
        uint16_t min_speed = 0, uint16_t max_speed = 1023);

    // If speed is negative, direction will be changed implicitly.
    void update_speed(int);
    // Work with Arduino HIGH and LOW definitions.
    void update_direction(int);
    // Stops the motor, i.e. speed is set to 0.
    void stop();

    int get_direction() const;
    int get_speed() const;

    float get_wheel_diameter() const;
    float get_wheel_radius() const;

    // Convert motor pulses to cm. For pulse_cm_ratio
    // please refer to main.cpp comment in the top of the file.
    static inline long pulses_to_cm(long pulses, float pulse_cm_ratio = 0.0730208f)
    {
        return ceil(pulses * pulse_cm_ratio * 100) / 100;
    }

public:
    volatile long pulses = 0;
private:
    Wheel wheel;
    uint8_t speed_pin, dir_pin;
    uint16_t min_speed, max_speed;
    int direction = LOW;
    int speed = 0;
    float cm_pulse_ratio;
}; 

// Main car class holding motors, compass and anything else.
class Car
{
public:
    Car(Motor, Motor, int compass_bus = 0x60);

    void update_compass_angle();
    // If speed is negative, direction will be changed implicitly.
    void update_speed(int, int);
    // Turns off or on ALL motors car has.
    void toggle_motors_power();
    // Stops ALL motors car has.
    void stop();

    // Values =< 180 -> rotate left, > 180 -> rotate right.
    void turn_degree(int);
    // Drive car forward or backward given value cm.
    void drive_n_cm(int);

    uint8_t get_direction() const;
    long get_compass_angle() const;
    long get_distance_traveled() const;
    uint8_t is_powered_on() const;

private:
    // Slightly redundant separation.
    void turn_car(int);
    // User is not allowed to change direction manually.
    void update_direction(int, int);

public:
    Motor left_motor;
    Motor right_motor;

private:
    int compass_bus;
    long compass_angle = 0;
    uint8_t motors_power = 0;
    uint64_t accumulated_distance = 0;
};

#endif