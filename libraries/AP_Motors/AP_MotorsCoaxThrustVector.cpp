#include "AP_MotorsCoaxThrustVector.h"

AP_MotorsCoaxThrustVector::AP_MotorsCoaxThrustVector(uint16_t speed_hz)
    : AP_MotorsMulticopter(speed_hz)
{}

void AP_MotorsCoaxThrustVector::setup_motors() {
    // Add 4 servos for vectoring
    add_motor(AP_MOTORS_MOT_1, 0.0f, 0.0f);
    add_motor(AP_MOTORS_MOT_2, 0.0f, 0.0f);
    add_motor(AP_MOTORS_MOT_3, 0.0f, 0.0f);
    add_motor(AP_MOTORS_MOT_4, 0.0f, 0.0f);

    // Add 2 motors for main thrust
    add_motor(AP_MOTORS_MOT_5, 0.0f, 0.0f, true); // CW
    add_motor(AP_MOTORS_MOT_6, 0.0f, 0.0f, true); // CCW
}

void AP_MotorsCoaxThrustVector::output_to_motors() {
    // Servos
    rc_write_angle(AP_MOTORS_MOT_1, _actuator_out[0]);
    rc_write_angle(AP_MOTORS_MOT_2, _actuator_out[1]);
    rc_write_angle(AP_MOTORS_MOT_3, _actuator_out[2]);
    rc_write_angle(AP_MOTORS_MOT_4, _actuator_out[3]);

    // Motors
    rc_write(AP_MOTORS_MOT_5, output_to_pwm(_thrust_yt_cw));
    rc_write(AP_MOTORS_MOT_6, output_to_pwm(_thrust_yt_ccw));
}

void AP_MotorsCoaxThrustVector::rc_write_angle(uint8_t motor_idx, float value) {
    // Map -1 to +1 -> PWM 900-2000
    const float pwm_min = 900.0f;
    const float pwm_max = 2000.0f;
    const float pwm_mid = (pwm_min + pwm_max) * 0.5f;
    const float pwm_range = (pwm_max - pwm_min) * 0.5f;

    uint16_t pwm = (uint16_t)(pwm_mid + constrain_float(value, -1.0f, 1.0f) * pwm_range);
    rc_write(motor_idx, pwm);
}

uint16_t AP_MotorsCoaxThrustVector::output_to_pwm(float value) const {
    // Map 0 to +1 -> PWM 1000-2000
    const float pwm_min = 1000.0f;
    const float pwm_max = 2000.0f;
    value = constrain_float(value, 0.0f, 1.0f);
    return (uint16_t)(pwm_min + value * (pwm_max - pwm_min));
}
