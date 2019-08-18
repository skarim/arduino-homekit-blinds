/**
 * Blinds
 *
 * ESP 8266 smart blinds
 * Documentatino TBD
 */

#include <Arduino.h>
#include <Servo.h>
#include <ESPAsyncWebServer.h>
#include <Config.h>

AsyncWebServer server(80);

// milliseconds of rotation per 1% change in blinds
const unsigned int millisPerPercent = 1000;

const char* SET_POSITION_PARAM = "position";

// servo configs
const int servoPin = 2;
Servo servo;

// blind position / state vars
double startingPosition = 0;
double currentPosition = 0;
double desiredPosition = 0;
bool spinning = false;
int direction; // -1 is closing, 1 is opening

// stores the last time a cycle was initiated
long previousMillis = 0;

// calculated interval (ms) of spin cycle
long interval = 0;

void startSpinning(double newPosition){
  digitalWrite(LED_BUILTIN, LOW);
  if (newPosition != desiredPosition || newPosition != currentPosition) {
    if (spinning) {
      // detach from servo before spinning again
      servo.detach();
    }

    desiredPosition = newPosition;

    unsigned long currentMillis = millis();

    Serial.print("current position: ");
    Serial.println(currentPosition);
    Serial.print("desired position: ");
    Serial.println(desiredPosition);

    // calculate spin cycle length (ms)
    interval = (long)(abs(desiredPosition - currentPosition) * millisPerPercent);
    Serial.print("calculated interval: ");
    Serial.println(interval);

    // start moving servo in desired direction
    int dutyCycle = desiredPosition > currentPosition ? 0 : 180;
    servo.attach(servoPin);
    servo.write(dutyCycle);

    // set state vars
    direction = desiredPosition > currentPosition ? 1 : -1;
    previousMillis = currentMillis;
    spinning = true;
    startingPosition = currentPosition;
    Serial.print("calculated direction: ");
    Serial.println(direction);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void runServer(){
  // get position
  server.on("/current_position", HTTP_GET, [] (AsyncWebServerRequest *request) {
    digitalWrite(LED_BUILTIN, LOW);
    request->send(200, "text/plain", String(currentPosition));
    digitalWrite(LED_BUILTIN, HIGH);
  });

  // set position with ?position=N (0 to 100)
  server.on("/set", HTTP_GET, [] (AsyncWebServerRequest *request) {
    digitalWrite(LED_BUILTIN, LOW);
    if (request->hasParam(SET_POSITION_PARAM)) {
      double newPosition = request->getParam(SET_POSITION_PARAM)->value().toDouble();
      startSpinning(newPosition);
    }
    request->send(204);
    digitalWrite(LED_BUILTIN, HIGH);
  });

  // 404
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404);
  });

  server.begin();
}

void setup(){
  Serial.begin(115200);

  // stop servo from spinning while turning on
  servo.detach();

  // initialize LED pins as outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);

  // wifi connect
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  WiFi.hostname(HOSTNAME);

  digitalWrite(2, HIGH);

  runServer();

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop(){
  unsigned long currentMillis = millis();

  if (spinning){
    // calculate current position
    double percentCompleted = ((double)(currentMillis - previousMillis))/((double)interval);
    currentPosition = (percentCompleted * (desiredPosition-startingPosition)) + startingPosition;

    // if position has been reached or is out of range, stop spinning
    if (currentPosition == desiredPosition || percentCompleted > 1 || currentPosition < 0 || currentPosition > 100){
      servo.detach();
      spinning = false;
      currentPosition = desiredPosition;
      Serial.println("finished spinning!");
      digitalWrite(2, HIGH);
    }
  }
}
