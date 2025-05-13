/**
 * Project:     BasicOTA for VSCode-PlatformIO
 * Inspiration: https://lastminuteengineers.com/esp32-ota-updates-arduino-ide/
*/

#include <Arduino.h>
#include <WiFi.h>
#include "BasicOTA.hpp"
#include <ESPServo.h>
#include <WebSocketsServer.h>
#include <ESP32Encoder.h>
#include <QuickPID.h>

#define SSID      "Mi 10"
#define PASSWORD  "samiunsamiun"

BasicOTA OTA;
ESPServo servo;
ESP32Encoder encoder;
float encoder_input = 0.0; // Variable to hold the encoder count
float output = 0.0; // Variable to hold the PID output
float setpoint = 0.0; // Variable to hold the setpoint for the PID controller
QuickPID pid(&encoder_input, &output, &setpoint, 2.0, 5.0, 1.0, QuickPID::Action::direct);
// WebSocket server on port 81
WebSocketsServer webSocket(81);
const int servoPin = 13; // GPIO pin for the servo
const int servoAngle = 90; // Angle to set the servo to

const int pulse_per_revolution = 489; // Number of pulses per revolution for the encoder
const int max_rpm = 300; // Maximum RPM for the motor

// Struct to hold servo and throttle values
struct ServoThrottle {
  float servoValue;
  float throttleValue;
};

void runOTA(void *pvParam){
  while (true) {
    OTA.handle();
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 1 second
  }
}

void runWebSocket(void *pvParam){
  while (true) {
    webSocket.loop();
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 1 second
  }
}

// Variables to store received values (0.0 - 100.0)
float steer = 50.0;
float throttle = 0.0;

// WebSocket event handler
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("Client [%u] connected from %s\n", num, ip.toString().c_str());
      break;
    }
    case WStype_DISCONNECTED: {
      Serial.printf("Client [%u] disconnected\n", num);
      break;
    }
    case WStype_BIN: {
      // Expect exactly two floats (8 bytes total)
      if (length == sizeof(ServoThrottle)) {
        ServoThrottle data;
        memcpy(&data, payload, sizeof(ServoThrottle));
        // Assign to control variables
        if (data.servoValue < 0.0 || data.servoValue > 100.0) {
          Serial.println("Invalid servo value");
          return;
        }
        if (data.throttleValue < 0.0 || data.throttleValue > 100.0) {
          Serial.println("Invalid throttle value");
          return;
        }
        steer = data.servoValue;
        throttle = data.throttleValue;
        setpoint = (throttle / 100.0) * max_rpm; // Convert throttle to RPM
        pid.Compute(); // Compute the PID output
        Serial.printf("Control Output\n", output);
        Serial.printf("Steer: %.2f, Throttle: %.2f\n", steer, setpoint);
        // TODO: Use steer & throttle to drive motors/servos
      } else {
        Serial.printf("Unexpected binary length: %u bytes\n", length);
      }
      break;
    }
    case WStype_TEXT: {
      // Optionally handle text commands here
      break;
    }
    default:
      break;
  }
}

void playTone(int tone_val, int duration) {
  tone(17, tone_val, duration); // Set GPIO 17 to the specified frequency
  delay(duration * 1.30);
  noTone(17);
}

void mario_task(void *pvParam) {
  while (true) {
    // playTone(660, 100); delay(150);
    // playTone(660, 100); delay(300);
    // playTone(660, 100); delay(300);
    // playTone(510, 100); delay(100);
    // playTone(660, 100); delay(300);
    // playTone(770, 100); delay(550);
    // playTone(380, 100); delay(575);

    // playTone(510, 100); delay(450);
    // playTone(380, 100); delay(400);
    // playTone(320, 100); delay(500);
    // playTone(440, 100); delay(300);
    // playTone(480, 80); delay(330);
    // playTone(450, 100); delay(150);
    // playTone(430, 100); delay(300);
    // playTone(380, 100); delay(200);
    // playTone(660, 80); delay(200);
    // playTone(760, 50); delay(200);
    // playTone(860, 100); delay(300);
    // playTone(700, 80); delay(150);
    // playTone(760, 50); delay(350);
    // playTone(660, 80); delay(300);
    // playTone(520, 80); delay(150);
    // playTone(580, 80); delay(150);
    // playTone(480, 80); delay(500);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
  }
}

void control_task(void *pvParam) {
  int delta = 0;
  int last_encoder_input = 0;
  while (true) {
    // Read the encoder count
    delta = encoder.getCount();
    delta = delta - last_encoder_input;
    last_encoder_input = encoder.getCount();
    encoder_input = (float)delta / pulse_per_revolution * 60.0; // Convert to RPM
    // Compute the PID output
    pid.Compute();
    Serial.printf("Encoder Input: %.2f RPM; Control output: %.2f; Setpoint: %.2f\n", encoder_input, output, setpoint);
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 1 second
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Startup");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  tone(17, 1000); // Set GPIO 18 to 1 kHz
  vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
  tone(17, 0); // Stop the tone
  servo.attach(servoPin); // Attach the servo to the specified pin
  servo.writeMicroseconds(1500); // Set the servo to 1000 microseconds (minimum pulse width)
  pinMode(25, INPUT_PULLDOWN); // Set GPIO 25 as input for the encoder
  pinMode(26, INPUT_PULLDOWN); // Set GPIO 26 as input for the encoder
  pinMode(18, OUTPUT); // Set GPIO 18 as output for the throttle
  pinMode(23, OUTPUT); // Set GPIO 23 as output for the throttle
  analogWriteFrequency(50000); // Set the PWM frequency to 50 kHz
  analogWriteResolution(8); // Set the PWM resolution to 8 bits (0-255)

  pid.SetOutputLimits(-255, 255); // Set the output limits for the PID controller
  pid.SetMode(QuickPID::Control::automatic); // Set the PID controller to automatic mode
  pid.SetSampleTimeUs(10000); // Set the sample time to 10 ms

  OTA.begin(); // Setup settings

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // Setup the encoder
  encoder.attachHalfQuad(26, 25); // Attach the encoder to GPIO pins 34 and 35
  encoder.setCount(0); // Initialize the encoder count to 0
  encoder.setFilter(100); // Set the filter to 100 counts

  // Run the OTA loop in another thread
  xTaskCreate(
    runOTA,       // Function to implement the task
    "runOTA",     // Name of the task
    10000,        // Stack size in words
    NULL,         // Task input parameter
    1,            // Priority of the task
    NULL);        // Task handle

  // Run the WebSocket loop in another thread
  xTaskCreate(
    runWebSocket, // Function to implement the task
    "runWebSocket", // Name of the task
    10000,        // Stack size in words
    NULL,         // Task input parameter
    1,            // Priority of the task
    NULL);        // Task handle

  // Run the Mario task in another thread
  xTaskCreate(
    mario_task,   // Function to implement the task
    "mario_task", // Name of the task
    10000,        // Stack size in words
    NULL,         // Task input parameter
    1,            // Priority of the task
    NULL);        // Task handle

  // Run the control task in another thread
  xTaskCreate(
    control_task, // Function to implement the task
    "control_task", // Name of the task
    10000,        // Stack size in words
    NULL,         // Task input parameter
    1,            // Priority of the task
    NULL);        // Task handle
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 1 second
  servo.write(steer*1.8); // Set the servo to the desired angle
  
  // Print the encoder count
  // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
  // servo.writeMicroseconds(2000);
  digitalWrite(18, output < 0 ? HIGH: LOW); // Set GPIO 25 to HIGH
  analogWrite(23, abs(output)); // Set GPIO 26 to the absolute value of throttle
}