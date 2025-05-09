// === ESPServo.cpp ===

#include "ESPServo.h"

ESPServo::ESPServo() : _pin(255) {}

void ESPServo::attach(uint8_t pin) {
    _pin = pin;
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, (gpio_num_t)_pin);

    mcpwm_config_t pwm_config = {
        .frequency = 50, // 50Hz = 20ms period
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    write(90); // Center servo
}

void ESPServo::write(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    writeMicroseconds(angleToPulseUs(angle));
}

void ESPServo::writeMicroseconds(uint32_t us) {
    if (us < SERVO_MIN_PULSEWIDTH_US) us = SERVO_MIN_PULSEWIDTH_US;
    if (us > SERVO_MAX_PULSEWIDTH_US) us = SERVO_MAX_PULSEWIDTH_US;
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, us);
}

uint32_t ESPServo::angleToPulseUs(int angle) {
    return (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) / SERVO_MAX_DEGREE + SERVO_MIN_PULSEWIDTH_US;
}
