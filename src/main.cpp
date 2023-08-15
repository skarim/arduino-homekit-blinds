/**
 * ESP32 smart blinds
 * Using a stepper motor
 * Powered by a TMC2209 stepper driver
 */

#include <EEPROM.h>
#include <TMCStepper.h>
#include <ESPAsyncWebServer.h>
#include <Config.h>
#include "esp_task_wdt.h"

#define LED_PIN 2

#define SERIAL_PORT Serial2

#define EN_PIN    25  // Driver enable pin
#define DIR_PIN   26  // Driver direction pin
#define STEP_PIN  27  // Driver step pin

#define CLK             14    // Clock
#define DRIVER_ADDRESS  0b00  // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE         0.11f // SilentStepStick series use 0.11

#define STEPS_DELAY_FAST  750   // delay between steps in microseconds
#define STEPS_DELAY_SLOW  1000  // delay between steps in microseconds

#define STEPS_PER_REV 1600  // number of steps per revolution
#define MOTOR_CURRENT 2800  // motor current in milliamps
#define MICROSTEPS    256   // microsteps per step
#define TOTAL_ROTATIONS 10  // total number of rotations to open/close blinds

#define SERIAL_BAUD_RATE    115200

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AsyncWebServer server(80);
TaskHandle_t MoveStepperTask;

// blinds settings
unsigned int totalSteps = STEPS_PER_REV * TOTAL_ROTATIONS;

// position vars (0 is closed, 100 is open)
double currentPosition = 100;
double desiredPosition = 100;
bool moving = false;

void moveStepper(int steps, int dir) {
  digitalWrite(LED_PIN, HIGH); // turn on light
  Serial.print("moving steps: ");
  Serial.println(steps);

  int stepsDelay = dir ? STEPS_DELAY_FAST : STEPS_DELAY_SLOW;

  moving = true;
  digitalWrite(EN_PIN, LOW); // enable stepper motor driver
  digitalWrite(DIR_PIN, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepsDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepsDelay);

    // update current position
    int direction = dir == 1 ? -1 : 1;
    currentPosition = currentPosition + (direction * 1.0 / totalSteps * 100);
  }

  moving = false;
  digitalWrite(EN_PIN, HIGH); // disable stepper motor driver
  digitalWrite(LED_PIN, LOW); // turn off light
}

void moveBlinds(void * pvParameters) {
  int dir = 0;
  int delta = 0;
  if (desiredPosition > currentPosition) {
    // move up (opening)
    dir = 0;
    delta = desiredPosition - currentPosition;
  } else if (desiredPosition < currentPosition) {
    // move down (closing)
    dir = 1;
    delta = currentPosition - desiredPosition;
  }
  int steps = delta * totalSteps / 100;

  if (steps != 0) {
    moveStepper(steps, dir);
  }
  vTaskDelete(NULL);  // end task
}

void runServer() {
  // get position
  server.on("/position", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(currentPosition));
  });

  // set position
  server.on("/set", HTTP_POST, [] (AsyncWebServerRequest *request) {
    if (request->hasParam("position")) {
      AsyncWebParameter* p = request->getParam("position");
      desiredPosition = p->value().toDouble();
      Serial.print("desired position: ");
      Serial.println(desiredPosition);

      // stop any running tasks
      if (MoveStepperTask != NULL) {
        vTaskDelete(MoveStepperTask);
        MoveStepperTask = NULL;
      }

      esp_task_wdt_init(60, false);
      xTaskCreatePinnedToCore(
        moveBlinds,           // Task function
        "MoveBlindsTask",     // Task name
        10000,                // Stack size
        NULL,                 // Parameters
        1,                    // Priority
        &MoveStepperTask,     // Task handle
        0                     // Core number
      );

      request->send(204);
    } else {
      request->send(400, "text/plain", "Invalid Request");
    }
  });

  server.begin();
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);           // initialize serial0 for logging
  Serial2.begin(SERIAL_BAUD_RATE);          // initialize serial2 for UART motor control
  driver.begin();                           // Initialize driver over UART

  Serial.println("init start");
  
  // Set pinmodes
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Turn off lights
  digitalWrite(LED_PIN, LOW);

  // Enable driver
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.rms_current(MOTOR_CURRENT);  // set motor RMS current
  driver.microsteps(MICROSTEPS);      // set microsteps
  driver.blank_time(24);              // driver blanking time
  driver.irun(31);                    // motor run current
  driver.TCOOLTHRS(0xFFFFF);          // coolstep threshold
  driver.SGTHRS(255);                 // stallguard threshold

  Serial.print("current set to:");
  Serial.println(driver.rms_current());

  // use stealthChop for quiet operation
  driver.toff(3);
  driver.en_spreadCycle(true);
  driver.pwm_autoscale(true);

  // wifi connect
  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  runServer();
}

void loop() {
}
