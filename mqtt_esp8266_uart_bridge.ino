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
unsigned long last_status = 30000;  // nonzero to defer our first status until triggered
unsigned long inPackets = 0;
unsigned long outPackets = 0;
bool cold_boot = true;  // treat power-on differently


WiFiClient wifi;
ESP8266WiFiMulti wifiMulti;
WiFiUDP udp;
NTPClient ntpClient(udp, NTP_SERVER, 0, NTP_UPDATE_INTERVAL);
PubSubClient mqtt(wifi);
unsigned long last_wifi_check = 0;
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
 * Called whenever a payload is received from a subscribed MQTT topic
 */
void mqtt_receive_callback(char* topic, byte* payload, unsigned int length) {
  char buffer[256];
  buffer[0] = 0;
  int len = 0;

  // Directly pass through the bytes
  // TODO: use COBS to denote payload frame
  Serial.write(payload, length);
  inPackets++;

  // this will effectively be a half-blink, forcing the LED to the
  // requested state
  nBlinks += 1;
  //pub_status_mqtt("rx-callback");
}





/*
 * Check WiFi connection, attempt to reconnect.
 * This blocks until a connection is (re)established.
 */
bool check_wifi()
{
  if (cold_boot || WiFi.status() != WL_CONNECTED) {
    if ((millis() - last_wifi_check) >= WIFI_RETRY_DELAY) {
      Serial.print("WiFi connecting");
      Serial.print("*");

      int retries = 5;
      while (retries > 0 && wifiMulti.run() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        retries--;
      }

      if (retries == 0) {
        String msg = "WiFi: failed, waiting for " + String(WIFI_RETRY_DELAY/1000) + " seconds before trying again";
        Serial.println(msg);
      } else {
        Serial.println(WiFi.localIP().toString());
      }
      last_wifi_check = millis();
    } // retry delay
  } // !connected
  return (WiFi.status() == WL_CONNECTED);
}


/*
 * Check the MQTT connection state and attempt to reconnect.
 * If we do reconnect, then subscribe to MQTT_DOWNLINK_TOPIC and
 * make an announcement to MQTT_ANNOUNCE_TOPIC with the WiFi SSID and
 * local IP address.
 */
bool check_mqtt()
{
  if (mqtt.connected()) { return true; }

  // no point to continue if no network
  if (WiFi.status() != WL_CONNECTED) { return false; }

  // reconnect
  Serial.print("MQTT reconnect...");
  // Attempt to connect
  int connect_status = mqtt.connect(my_mac.c_str(), MQTT_USER, MQTT_PASS,
                   (MQTT_PREFIX_TOPIC + my_mac + MQTT_ANNOUNCE_TOPIC).c_str(),
                   2,  // willQoS
                   1,  // willRetain
                   "{\"state\":\"disconnected\"}");
  if (connect_status) {
    // let everyone know we are alive
    pub_status_mqtt("connected");

    // ... and resubscribe to downlink topic
    mqtt.subscribe((MQTT_PREFIX_TOPIC + my_mac + MQTT_DOWNLINK_TOPIC).c_str());
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
  }
  return mqtt.connected();
}



/*
 * Publish a status message with system telemetry.
 */
bool pub_status_mqtt(const char *state)
{
  // JSON formatted payload
  StaticJsonDocument<256> status_json;
  status_json["state"] = state;
  status_json["time"] = getIsoTime();
  status_json["uptime_ms"] = millis();
  status_json["packets_in"] = inPackets;
  status_json["packets_out"] = outPackets;
  status_json["ssid"] = WiFi.SSID();
  status_json["rssi"] = WiFi.RSSI();
  status_json["ip"] = WiFi.localIP().toString();
  //status_json["hostname"] = WiFi.getHostname();
  status_json["version"] = GIT_VERSION;
  status_json["heap_free"] = ESP.getFreeHeap();

  char buf[256];
  serializeJson(status_json, buf);

  Serial.println(buf);

  if (mqtt.connected()) {
    return mqtt.publish((MQTT_PREFIX_TOPIC + my_mac + MQTT_ANNOUNCE_TOPIC).c_str(),
                        buf,
                        true);
  } else {
    return false;
  }
}







/***********************************************************
 * SETUP
 ***********************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.setTimeout(SERIAL_TIMEOUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led_state);


  /*
   * setup WiFi
   */
  //WiFi.mode(WIFI_STA);
  for (int i=0; i<NUM_WLANS; i++) {
    wifiMulti.addAP(WLAN_SSID[i], WLAN_PASS[i]);
  }

  WiFi.macAddress(mac);
  my_mac = hexToStr(mac, 6);
  String msg = "\nMAC: " + my_mac;
  Serial.println(msg);

  msg = "mqtt8266bridge-" + my_mac;
  WiFi.setHostname(msg.c_str());

  int retries = 5;
  while (retries > 0 && wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retries--;
  }
  Serial.println(wifi.localIP().toString());


  /*
   * setup MQTT
   */
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqtt_receive_callback);
  mqtt.setBufferSize(512);
  topic = MQTT_PREFIX_TOPIC + my_mac + MQTT_UPLINK_TOPIC;

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
    mqtt.publish((MQTT_PREFIX_TOPIC + my_mac + MQTT_UPLINK_TOPIC).c_str(),
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

    if (inPackets > 0) {
      no_packet_intervals = NO_PACKETS_INTERVALS_ZOMBIE_RESTART;
    } else {
      Serial.println("no packets :(");
      no_packet_intervals--;

      if (no_packet_intervals == 0) {
        // no heard packets is a potential problem
        pub_status_mqtt("restarting");
        ESP.restart();
      }
    }

    inPackets = 0;
    last_status = millis();
  }
  /*
   * end periodic events
   */


  delay(1);
}
