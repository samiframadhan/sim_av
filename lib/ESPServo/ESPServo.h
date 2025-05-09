/*
  ESPServo - Arduino library for controlling hobby servos on ESP32 using legacy MCPWM driver
  Inspired by Espressif's servo example
  https://github.com/yourusername/ESPServo
*/

#ifndef ESPSERVO_H
#define ESPSERVO_H

#include <Arduino.h>
#include "driver/mcpwm.h"

class ESPServo {
public:
    ESPServo();

    /**
     * Attach the servo to an MCPWM-compatible GPIO pin
     * @param pin The GPIO number to use (e.g., 18)
     */
    void attach(uint8_t pin);

    /**
     * Set servo angle (0-180 degrees)
     * @param angle Servo angle
     */
    void write(int angle);

    /**
     * Set servo pulse width in microseconds
     * @param us Pulse width in microseconds (1000 to 2000us typically)
     */
    void writeMicroseconds(uint32_t us);

private:
    uint8_t _pin;
    static constexpr uint32_t SERVO_MIN_PULSEWIDTH_US = 1000;
    static constexpr uint32_t SERVO_MAX_PULSEWIDTH_US = 2000;
    static constexpr uint32_t SERVO_MAX_DEGREE = 180;

    uint32_t angleToPulseUs(int angle);
};

#endif // ESPSERVO_H