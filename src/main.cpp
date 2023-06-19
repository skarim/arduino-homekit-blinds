/**
 * ESP 8266 smart blinds
 * Using a stepper motor
 * Powered by a TMC2209 stepper driver
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

#define LED_ESP_PIN 2
#define LED_MCU_PIN 16

#define EN_PIN    10  // Driver enable pin
#define DIR_PIN   4   // Driver irection pin
#define STEP_PIN  5   // Driver step pin

#define SW_SCK          5     // Clock
#define SW_TX           1     // Serial receive pin
#define SW_RX           3     // Serial transmit pin
#define DRIVER_ADDRESS  0b00  // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE         0.11f // SilentStepStick series use 0.11

#define STEP_DELAY    1000  // delay between steps in microseconds
#define STEPS_PER_REV 900   // number of steps per revolution

TMC2209Stepper driver(&Serial, R_SENSE, DRIVER_ADDRESS);

// spin direction
bool dir = false;

void moveStepper(int steps, int dir) {
  digitalWrite(EN_PIN, LOW); // enable stepper motor driver
  digitalWrite(DIR_PIN, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
  }
  digitalWrite(EN_PIN, HIGH); // disable stepper motor driver
}

void setup() {
  Serial.begin(115200);         // initialize hardware serial for UART motor control
  driver.beginSerial(115200);   // Initialize UART
  
  // Set pinmodes
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LED_ESP_PIN, OUTPUT);
  pinMode(LED_MCU_PIN, OUTPUT);

  // Enable driver board
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.rms_current(2800); // Set motor RMS current
  driver.microsteps(0);     // Set microsteps

  // use stealthChop for quiet operation
  driver.toff(3);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
}

void loop() {
  digitalWrite(LED_ESP_PIN, HIGH);  // turn off esp light
  digitalWrite(LED_MCU_PIN, LOW);   // turn on mcu light
  moveStepper(STEPS_PER_REV, 1);

  delay(1000); // pause for a second

  digitalWrite(LED_ESP_PIN, LOW);   // turn on esp light
  digitalWrite(LED_MCU_PIN, HIGH);  // turn off light
  moveStepper(STEPS_PER_REV, 0);

  delay(1000); // pause for a second
}
