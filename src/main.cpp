/**
 * ESP 8266 smart blinds
 * Using a stepper motor
 * Powered by a TMC2209 stepper driver
 */

#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <ESPAsyncWebServer.h>
#include <Config.h>

#define LED_ESP_PIN 2
#define LED_MCU_PIN 16

#define EN_PIN    14  // Driver enable pin
#define DIR_PIN   4   // Driver direction pin
#define STEP_PIN  5   // Driver step pin

#define SW_SCK          5     // Clock
#define SW_TX           9     // Serial receive pin
#define SW_RX           10    // Serial transmit pin
#define DRIVER_ADDRESS  0b00  // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE         0.11f // SilentStepStick series use 0.11

#define STEP_DELAY    2000  // delay between steps in microseconds
#define STEPS_PER_REV 1600  // number of steps per revolution
#define MOTOR_CURRENT 2800  // motor current in milliamps
#define MICROSTEPS    256   // microsteps per step
#define MAX_CONTINUOUS_STEPS  STEPS_PER_REV/4

#define SERIAL_BAUD_RATE    115200
#define SW_SERIAL_BAUD_RATE 19200

SoftwareSerial SWSerial(SW_RX, SW_TX);
TMC2209Stepper driver(&SWSerial, R_SENSE, DRIVER_ADDRESS);
AsyncWebServer server(80);

// blinds settings
unsigned int totalSteps = STEPS_PER_REV * TOTAL_ROTATIONS;

// blind position / state vars
double startingPosition = 0;
double currentPosition = 0;
double desiredPosition = 0;
bool spinning = false;
int direction; // 1 is closing, -1 is opening

void moveStepper(int steps, int dir) {
  int currentSteps = min(steps, MAX_CONTINUOUS_STEPS);
  Serial.print("moving ");
  Serial.println(currentSteps);

  digitalWrite(LED_ESP_PIN, LOW); // turn on esp light
  digitalWrite(EN_PIN, LOW); // enable stepper motor driver
  digitalWrite(DIR_PIN, dir);
  for (int i = 0; i < currentSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    // yield();
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    // yield();
    delayMicroseconds(STEP_DELAY);
  }
  digitalWrite(EN_PIN, HIGH); // disable stepper motor driver
  digitalWrite(LED_ESP_PIN, HIGH); // turn off esp light

  int additionalSteps = steps - currentSteps;
  Serial.print("remaining steps: ");
  Serial.println(additionalSteps);
  if (additionalSteps > 0) {
    moveStepper(additionalSteps, dir);
  }
}

void runServer() {
  // get position
  server.on("/position", HTTP_GET, [] (AsyncWebServerRequest *request) {
    digitalWrite(LED_MCU_PIN, LOW);
    request->send(200, "text/plain", String(currentPosition));
    digitalWrite(LED_MCU_PIN, HIGH);
  });

  server.begin();
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);           // initialize hardware serial for logging
  SWSerial.begin(SW_SERIAL_BAUD_RATE);      // initialize software serial for UART motor control
  driver.beginSerial(SW_SERIAL_BAUD_RATE);  // Initialize driver over UART

  Serial.println("init start");
  
  // Set pinmodes
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LED_ESP_PIN, OUTPUT);
  pinMode(LED_MCU_PIN, OUTPUT);

  // Turn off lights
  digitalWrite(LED_ESP_PIN, HIGH);
  digitalWrite(LED_MCU_PIN, HIGH);

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
  uint16_t msread=driver.microsteps();
  Serial.print(F("Read microsteps via UART to test UART receive : "));
  Serial.println(msread);

  Serial.println("spin forward");
  digitalWrite(LED_ESP_PIN, LOW);  // turn on esp light
  digitalWrite(LED_MCU_PIN, LOW);   // turn on mcu light
  digitalWrite(EN_PIN, LOW);
  moveStepper(STEPS_PER_REV*3, 1);
  digitalWrite(EN_PIN, HIGH);
  delay(5000); // pause for a second

  Serial.println("spin backward");
  digitalWrite(LED_ESP_PIN, LOW);   // turn on esp light
  digitalWrite(LED_MCU_PIN, HIGH);  // turn off light
  digitalWrite(EN_PIN, LOW);
  moveStepper(STEPS_PER_REV*3, 0);
  digitalWrite(EN_PIN, HIGH);
  delay(5000); // pause for a second
}
