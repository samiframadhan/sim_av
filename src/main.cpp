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

#define SSID      "Mi 10"
#define PASSWORD  "samiunsamiun"

BasicOTA OTA;
ESPServo servo;
ESP32Encoder encoder;
// WebSocket server on port 81
WebSocketsServer webSocket(81);
const int servoPin = 13; // GPIO pin for the servo
const int servoAngle = 90; // Angle to set the servo to

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
float steer = 0.0;
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
        steer = data.servoValue;
        throttle = data.throttleValue;
        Serial.printf("Steer: %.2f, Throttle: %.2f\n", steer, throttle);
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
  servo.attach(servoPin); // Attach the servo to the specified pin
  servo.writeMicroseconds(1500); // Set the servo to 1000 microseconds (minimum pulse width)

  OTA.begin(); // Setup settings

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // Setup the encoder
  encoder.attachHalfQuad(36, 39); // Attach the encoder to GPIO pins 34 and 35
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
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
  servo.write(steer*1.8); // Set the servo to the desired angle
  
  // Print the encoder count
  Serial.printf("Encoder Count: %d\n", encoder.getCount());
  // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
  // servo.writeMicroseconds(2000);
}