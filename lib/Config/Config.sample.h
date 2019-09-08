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
 * SERVO_RESET_ENABLED to false
 */
#define SERVO_RESET_ENABLED true
#define SERVO_RESET_EVERY_SECONDS 14
#define SERVO_RESET_DELAY_MILLISECONDS 150

/**
 * Servo Stop Settings
 * 
 * DS 3218/3225 MG servos keep spinning after detatching
 * Need to set duty cycle to zero point (usually 90)
 * for a few ms for it stop spinning
 * 
 * If your servo doesn't require this, set
 * SERVO_STOP_SIGNAL_REQUIRED to false
 */
#define SERVO_STOP_SIGNAL_REQUIRED true
#define SERVO_STOP_DUTYCYCLE 90
#define SERVO_STOP_SIGNAL_MILLISECONDS 50
