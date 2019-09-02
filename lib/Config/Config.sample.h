/**
 * WiFi Config
 */
#define WIFI_SSID "your wifi network name"
#define WIFI_PASSWORD "your wifi password"
#define HOSTNAME "smart-blinds"

/**
 * Seconds it takes to open/close your blinds
 */
#define SECONDS_TO_CLOSE 10
#define SECONDS_TO_OPEN 15

/**
 * Servo Reset Settings
 * 
 * MG 995/6 servos stop spinning after 14 seconds
 * and need a 150ms "break" before we can use again
 * 
 * If your servo doesn't require this, set
 * SERVO_RESET_ENABLED to False
 */
#define SERVO_RESET_ENABLED true
#define SERVO_RESET_EVERY_SECONDS 14
#define SERVO_RESET_DELAY_MILLISECONDS 150
