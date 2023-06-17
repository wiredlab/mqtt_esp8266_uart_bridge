/*
 * ESP8266 - UART-MQTT bridge
 *
 * (C) 2023 Dan White <dan.white@valpo.edu>
 *
 * Take frames on the UART RX pin and publish them via MQTT.
 * Subscribe to MQTT topic and send contents out UART TX pin.
 * Perodically publish a /status message.
 *
 * See config-example.h for configuration constants then copy to config.h and
 * edit as appropriate.
 *
 * GPL-3.0 License
*/

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>

#include "time.h"

/*
 * Extra libraries installed from the Library Manager
 */
// https://github.com/arduino-libraries/NTPClient
#include <NTPClient.h>

// https://arduinojson.org/
#include <ArduinoJson.h>



/*
 * configuration includes passwords/etc
 * include separately to not leak private information
 */
#include "config.h"

/* https://github.com/fabianoriccardi/git-describe-arduino
 * add a GIT_VERSION string that is updated every compile event
 */
#include "git-version.h"


/*
 * Globals
 */
String topic;
struct tm timeinfo;

uint8_t mac[6];
String my_mac;

// blink-per-message housekeeping
unsigned int nBlinks = 0;
bool led_state = 1;
bool in_blink = false;
typeof(millis()) last_blink = 0;

// status update housekeeping
typeof(millis()) last_status = 30000;  // nonzero to defer our first status until triggered
unsigned long inPackets = 0;
unsigned long outPackets = 0;


WiFiClient wifi;
ESP8266WiFiMulti wifiMulti;
WiFiUDP udp;
NTPClient ntpClient(udp, NTP_SERVER, 0, NTP_UPDATE_INTERVAL);
PubSubClient mqtt(wifi);
typeof(millis()) last_wifi_check = -WIFI_RETRY_DELAY;
unsigned long last_mqtt_check = 0;



/*
 * Read a line from the UART port.
 *
 * Assumes that the bytes of a frame are sent within CHAR_TIMEOUT of each
 * other, and that the frame is terminated by a newline character '\n'.
 */
String serialRead() {
  String buf = "";
  char inChar;
  uint32_t char_time = millis();
  while (Serial.available() || millis() - char_time < CHAR_TIMEOUT) {
    inChar = (char)Serial.read();
    if (inChar == '\n') {
      break;
    } else if (inChar == '\r') {
      char_time = millis();
      continue;
    } else if ((int)inChar != 255) {
      buf += inChar;
      char_time = millis();
    }
  }
  return buf;
}



/*
 * Return a string in RFC3339 format of the current time.
 * Will return a placeholder if there is no network connection to an
 * NTP server.
 */
String getIsoTime()
{
  char timeStr[21] = {0};  // NOTE: change if strftime format changes

  time_t time_now = ntpClient.getEpochTime();
  localtime_r(&time_now, &timeinfo);

  if (timeinfo.tm_year <= (2016 - 1900)) {
    // seems to be a bogus value, so return a placeholder
    return String("YYYY-MM-DDTHH:MM:SSZ");
  } else {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(timeStr);
  }
}



/*
 * Given a byte array of length (n), return the ASCII hex representation
 * and properly zero pad values less than 0x10.
 * String(0x08, HEX) will yield '8' instead of the expected '08' string
 */
String hexToStr(uint8_t* arr, int n)
{
  String result;
  result.reserve(2*n);
  for (int i = 0; i < n; ++i) {
    if (arr[i] < 0x10) {result += '0';}
    result += String(arr[i], HEX);
  }
  return result;
}



/*
 * Called whenever a payload is received from a subscribed MQTT topic.
 */
void mqtt_receive_callback(char* topic, byte* payload, unsigned int length) {

  if (strcmp(topic, (MQTT_TOPIC_PREFIX + my_mac + MQTT_CONTROL_TOPIC).c_str()) == 0) {
      // TODO: use this to change behavior
  } else if (strcmp(topic, (MQTT_TOPIC_PREFIX + my_mac + MQTT_DOWNLINK_TOPIC).c_str()) == 0) {
      // Directly pass through the bytes
      // TODO: use COBS or other method to better denote payload frame
      Serial.write(payload, length);
  }

  inPackets++;

  // LED blinks once per payload
  nBlinks += 1;
}



/*
 * Check WiFi connection, attempt to reconnect.
 * This blocks until a connection is (re)established.
 */
bool check_wifi()
{
  if (WiFi.status() != WL_CONNECTED) {
    if ((millis() - last_wifi_check) >= WIFI_RETRY_DELAY) {
      Serial.print("[INFO] WiFi connecting.");

      int retries = 10;
      while (retries > 0 && wifiMulti.run() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        retries--;
      }

      if (retries == 0) {
        String msg = "[INFO] WiFi: failed, waiting for " + String(WIFI_RETRY_DELAY/1000) + " seconds before trying again";
        Serial.println(msg);
      } else {
        Serial.println(" " + WiFi.localIP().toString());
      }
      last_wifi_check = millis();
    } // retry delay
  } // !connected
  return (WiFi.status() == WL_CONNECTED);
}



/*
 * Check the MQTT connection state and attempt to reconnect.
 * If we do reconnect, then subscribe to MQTT_CONTROL_TOPIC and
 * MQTT_DOWNLINK_TOPIC and publish to MQTT_STATUS_TOPIC with status type and
 * telemetry.
 */
bool check_mqtt() { if (mqtt.connected()) { return true; }

  // no point to continue if no network
  if (WiFi.status() != WL_CONNECTED) { return false; }

  // reconnect
  Serial.print("[INFO] MQTT reconnect...");
  // Attempt to connect
  int connect_status = mqtt.connect(my_mac.c_str(), MQTT_USER, MQTT_PASS,
                   (MQTT_TOPIC_PREFIX + my_mac + MQTT_STATUS_TOPIC).c_str(),
                   2,  // willQoS
                   1,  // willRetain
                   "{\"state\":\"disconnected\"}");
  if (connect_status) {
    // let everyone know we are alive
    pub_status_mqtt("connected");

    // ... and resubscribe to control and downlink topics
    mqtt.subscribe((MQTT_TOPIC_PREFIX + my_mac + MQTT_CONTROL_TOPIC).c_str());
    mqtt.subscribe((MQTT_TOPIC_PREFIX + my_mac + MQTT_DOWNLINK_TOPIC).c_str());
  } else {
    Serial.print("failed, rc=");
  }
  Serial.println(mqtt.state());
  return mqtt.connected();
}



/*
 * Publish a status message with system telemetry.
 */
bool pub_status_mqtt(const char *state)
{
  bool retval = false;

  // JSON formatted payload
  StaticJsonDocument<512> status_json;
  status_json["state"] = state;
  status_json["time"] = getIsoTime();
  status_json["uptime_ms"] = millis();
  status_json["packets_in"] = inPackets;
  status_json["packets_out"] = outPackets;
  status_json["ssid"] = WiFi.SSID();
  status_json["rssi"] = WiFi.RSSI();
  status_json["ip"] = WiFi.localIP().toString();
  status_json["hostname"] = WiFi.getHostname();
  status_json["version"] = GIT_VERSION;
  status_json["heap_free"] = ESP.getFreeHeap();

  uint8_t buf[512];
  size_t len = serializeJson(status_json, buf);

  //Serial.println(buf);

  if (mqtt.connected()) {
    retval = mqtt.publish(
      (MQTT_TOPIC_PREFIX + my_mac + MQTT_STATUS_TOPIC).c_str(),
      buf,
      len,
      true);
  }

  if (retval) { nBlinks++; }

  return retval;
}







/***********************************************************
 * SETUP
 ***********************************************************/
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led_state);

  Serial.println();
  Serial.println();

  /*
   * setup WiFi
   */
  //WiFi.mode(WIFI_STA);
  for (int i=0; i<NUM_WLANS; i++) {
    wifiMulti.addAP(WLAN_SSID[i], WLAN_PASS[i]);
  }

  WiFi.macAddress(mac);
  my_mac = hexToStr(mac, 6);
  String msg = "[INFO] MAC: " + my_mac;
  Serial.println(msg);

  msg = "mqtt8266bridge-" + my_mac;
  WiFi.setHostname(msg.c_str());

  // Connect to WiFi
  check_wifi();


  /*
   * setup MQTT
   */
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqtt_receive_callback);
  mqtt.setBufferSize(512);
  topic = MQTT_TOPIC_PREFIX + my_mac + MQTT_UPLINK_TOPIC;

  delay(1000);

  /*
   * setup NTP time sync
   */
  ntpClient.begin();
  ntpClient.update();
}







/***********************************************************
 * LOOP
 ***********************************************************/
void loop() {
  bool wifi_good;
  bool mqtt_good;
  String rxBuffer;

  wifi_good = check_wifi();
  mqtt_good = check_mqtt();

  if (wifi_good) {
    ntpClient.update();
    mqtt.loop();
  }

  if (mqtt_good) {
    //celebrate!
  }

  rxBuffer = serialRead();
  if (rxBuffer.length() > 0) {
    mqtt.publish((MQTT_TOPIC_PREFIX + my_mac + MQTT_UPLINK_TOPIC).c_str(),
                 (uint8_t *)rxBuffer.c_str(),
                 rxBuffer.length(),
                 false);
    outPackets++;
    nBlinks++;
  }



  /*
   * Handle periodic events
   */
  unsigned long now = millis();

  // Handle blinking without using delay()
  if (nBlinks > 0) {
    if (now - last_blink >= BLINK_MS) {
      last_blink = now;
      if (in_blink) { // then finish
        digitalWrite(LED_PIN, led_state);
        nBlinks--;
        in_blink = false;
      } else {
        digitalWrite(LED_PIN, !led_state);
        in_blink = true;
      }
    }
  }

  static int no_packet_intervals = NO_PACKETS_INTERVALS_ZOMBIE_RESTART;
  if (now - last_status >= STATUS_INTERVAL) {
    pub_status_mqtt("status");

    if (inPackets > 0 || outPackets > 0) {
      // something happened, so things are ok?
      no_packet_intervals = NO_PACKETS_INTERVALS_ZOMBIE_RESTART;
    } else {
      no_packet_intervals--;

      if (no_packet_intervals == 0) {
        // no heard packets is a potential problem
        pub_status_mqtt("restarting");
        ESP.restart();
      }
    }

    inPackets = 0;
    outPackets = 0;
    last_status = millis();
  }
  /*
   * end periodic events
   */


  delay(1);
}
