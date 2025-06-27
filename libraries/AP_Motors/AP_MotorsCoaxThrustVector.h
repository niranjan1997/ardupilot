#pragma once

#include "AP_Motors/AP_MotorsMulticopter.h"

// Custom coaxial thrust vectoring motor driver
class AP_MotorsCoaxThrustVector : public AP_MotorsMulticopter {
public:
    AP_MotorsCoaxThrustVector(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    void setup_motors() override;
    void output_to_motors() override;

private:
    // Actuator outputs (-1 to +1) for 4 servos
    float _actuator_out[4] = {0};

    // Thrust outputs (0 to +1) for the two rotors
    float _thrust_yt_ccw = 0.0f;
    float _thrust_yt_cw = 0.0f;

    // Helpers
    void rc_write_angle(uint8_t motor_idx, float value);
    uint16_t output_to_pwm(float value) const;
};
