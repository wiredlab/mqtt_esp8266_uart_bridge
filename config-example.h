/*
 * configuration
 *
 * copy to "config.h" and fill in the values
 * !!!! DO NOT CHECK-IN THIS FILE TO git !!!!
 */

// on-board LED blinks once per packet
//   generic board: pin 2
#define LED_PIN 2
#define BLINK_MS 20


// WiFi config
const unsigned long WIFI_RETRY_DELAY = 300000;  // milliseconds   300000
const char *WLAN_SSID[] = {"name1", "name2"};
const char *WLAN_PASS[] = {"pass1", "pass2"};
const int NUM_WLANS = 2;


// SNTP time config
const char *NTP_SERVER = "us.pool.ntp.org";
const unsigned long NTP_UPDATE_INTERVAL = 1800000;  // ms between NTP queries

// Status updates
const unsigned long STATUS_INTERVAL = 300000;  // ms between status messages
const int NO_PACKETS_INTERVALS_ZOMBIE_RESTART = 10;  // no-packet status intervals? --> restart to resurrect

// MQTT settings
const char *MQTT_TOPIC_PREFIX = "valpo/mqtt8266bridge/";
const char *MQTT_ANNOUNCE_TOPIC = "/status";
const char *MQTT_DOWNLINK_TOPIC = "/stdin";
const char *MQTT_UPLINK_TOPIC = "/stdout";
const char *MQTT_SERVER = "CHANGE_ME";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "CHANGE_ME";
const char *MQTT_PASS = "CHANGE_ME";


// Serial settings
#define SERIAL_TIMEOUT 100  // timeout 100ms
#define CHAR_TIMEOUT 5 // char timeout (ms)
