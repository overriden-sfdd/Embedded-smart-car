#include "car.hpp"

#include <math.h>

#include <Wire.h>

Car::Car(Motor left_motor, Motor right_motor, int compass_bus)
    : left_motor{ left_motor }, right_motor{ right_motor },
    compass_bus{ compass_bus } {}

void Car::update_speed(int left_speed, int right_speed)
{
    this->update_direction(left_speed >= 0 ? HIGH : LOW, right_speed >= 0 ? HIGH : LOW);
    left_motor.update_speed(abs(left_speed));
    right_motor.update_speed(abs(right_speed));
}

void Car::update_direction(int left_dir, int right_dir)
{
    left_motor.update_direction(left_dir);
    right_motor.update_direction(right_dir);
}

void Car::update_compass_angle()
{
    // Simple usage of I2C bus to retrieve compass angle.
    Wire.beginTransmission(this->compass_bus);
    Wire.write(1);
    Wire.endTransmission(false);
    Wire.requestFrom(this->compass_bus, 1, true);

    while (Wire.available()) {
        // Scale angle. First divide, then multiply.
        // P.S: No need for long anymore.
        compass_angle = Wire.read() / 255.0 * 360;
    }
}

long Car::get_compass_angle() const
{
    return compass_angle;
}

long Car::get_distance_traveled() const
{
    // Sometimes pulses will not be equal.
    // We need to take the biggest number of pulses for distance.
    auto max_pulses = max(
        abs(left_motor.pulses),
        abs(right_motor.pulses)
    );
    return accumulated_distance + Motor::pulses_to_cm(max_pulses);
}

void Car::drive_n_cm(int distance)
{
    if (!distance) {
        return;
    } else if (distance > 0) {
        this->update_speed(200, 200);
    } else {
        this->update_speed(-200, -200);
    }

    auto lhs_start_pulses = left_motor.pulses;
    // Block and ride while traveled distance is < than desired.
    while (abs(static_cast<int>(Motor::pulses_to_cm(
        left_motor.pulses - lhs_start_pulses))) < abs(distance)) {
    }
    // Reset speed.
    this->stop();
}

void Car::turn_car(int degr)
{
    // Copy initial angle.
    auto start_angle = compass_angle;
    auto degr_more = 0;
    // Block and rotate.
    while (degr_more < degr) {
        // Update angle and copy it on each iteration.
        this->update_compass_angle();
        auto angle_now = this->get_compass_angle();
        // Something strange is happening if difference > 300.
        if (abs(angle_now - start_angle) > 300) {
            start_angle = angle_now > 180 ? 360 : 0;
        }
        // Accumulate rotated degrees and update start angle.
        degr_more += abs(angle_now - start_angle);
        start_angle = angle_now;
    }
}

void Car::turn_degree(int degr)
{
    if (degr > 0 && degr <= 180) {
        // Rotate left.
        this->update_speed(-200, 200);
    } else {
        // Rotate right.
        this->update_speed(200, -200);
    }

    this->turn_car(abs(degr));
    this->stop();
}

void Car::stop()
{
    // Update accumulated_distance when we stop our motors.
    // Need this because we reset pulses on stop.
    accumulated_distance = this->get_distance_traveled();
    // Better have array of motors rather than doing this.
    left_motor.stop();
    right_motor.stop();
}

uint8_t Car::is_powered_on() const
{
    return motors_power;
}

void Car::toggle_motors_power()
{
    motors_power = !motors_power;
}


Motor::Motor(Wheel wheel, uint8_t speed_pin, uint8_t dir_pin,
    uint16_t min_speed, uint16_t max_speed)
    : wheel{ wheel }, speed_pin{ speed_pin }, dir_pin{ dir_pin },
    min_speed{ min_speed }, max_speed{ max_speed }
{
    // Calculate cm_pulse_ratio according to heuristics.
    // For more info check top of the main.cpp file.
    auto C = wheel.diameter * PI;
    this->cm_pulse_ratio = C / wheel.pulses_per_rotation;
}

int Motor::get_direction() const
{
    return direction;
}

int Motor::get_speed() const
{
    return speed;
}

float Motor::get_wheel_diameter() const
{
    return wheel.diameter;
}

float Motor::get_wheel_radius() const
{
    return wheel.diameter / 2;
}

void Motor::update_speed(int speed)
{
    this->speed = speed;
    analogWrite(speed_pin, speed);
}

void Motor::update_direction(int direciton)
{
    this->direction = direciton;
    digitalWrite(dir_pin, direction);
}

void Motor::stop()
{
    this->pulses = 0;
    update_speed(0);
}