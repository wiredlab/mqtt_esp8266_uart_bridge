/*
 Basic ESP8266 MQTT example
 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"
*/

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>



// https://arduinojson.org/
#include <ArduinoJson.h>

/*
 * Helper functions.
 */
#include "utilities.h"


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
 * globals
 */
String topic;
struct tm timeinfo;

uint8_t mac[6];
String my_mac;

// blink-per-message housekeeping
unsigned int nBlinks = 0;
bool led_state = 0;
bool in_blink = false;
typeof(millis()) last_blink = 0;

// status update housekeeping
unsigned long last_status = 30000;  // nonzero to defer our first status until triggered
unsigned long nPackets = 0;


WiFiClient wifi;
WiFiMulti wifiMulti;  // use multiple wifi options
WiFiUDP udp;
NTPClient ntpClient(udp, NTP_SERVER, 0, NTP_UPDATE_INTERVAL);
PubSubClient mqtt(wifi);
unsigned long last_wifi_check = 0;
unsigned long last_mqtt_check = 0;



/*
 * Called whenever a payload is received from a subscribed MQTT topic
 */
void mqtt_receive_callback(char* topic, byte* payload, unsigned int length) {
  char buffer[256];
  buffer[0] = 0;

  int len = sprintf(buffer, "MQTT-receive %s ", topic);

  //Serial.print(topic);
  //Serial.print("] ");
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
    len += sprintf(buffer, "%02x", (char)payload[i]);
  }
  //Serial.println();
  len += sprintf(buffer, "\n");

  logger(buffer, sdcard_available);

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    led_state = 1;
  } else {
    led_state = 0;
  }

  // this will effectively be a half-blink, forcing the LED to the
  // requested state
  nBlinks += 1;
  pub_status_mqtt("rx-callback");
}





/*
 * Check WiFi connection, attempt to reconnect.
 * This blocks until a connection is (re)established.
 */
bool check_wifi()
{
  if (cold_boot || WiFi.status() != WL_CONNECTED) {
    if ((millis() - last_wifi_check) >= WIFI_RETRY_DELAY) {
      logger("WiFi connecting", sdcard_available);
      Serial.print("*");
      int retries = 5;
      //Serial.print("*");
      while (retries > 0 && wifiMulti.run() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        retries--;
      }

      if (retries == 0) {
        String msg = "WiFi: failed, waiting for " + String(WIFI_RETRY_DELAY/1000) + " seconds before trying again";
        logger(msg.c_str(), sdcard_available);
      } else {
        logger(WiFi.localIP().toString().c_str(), sdcard_available);
      }
      last_wifi_check = millis();
    } // retry delay
  } // !connected
  return (WiFi.status() == WL_CONNECTED);
}


/*
 * Check the MQTT connection state and attempt to reconnect.
 * If we do reconnect, then subscribe to MQTT_CONTROL_TOPIC and
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
    mqtt.subscribe((MQTT_PREFIX_TOPIC + my_mac + MQTT_CONTROL_TOPIC).c_str());
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
  }
  return mqtt.connected();
}









void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  WiFi.macAddress(mac);
  my_mac = hexToStr(mac, 6);
  String msg = "\nMAC: " + my_mac;
  Serial.println(msg);
  
  setup_wifi();



  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}



void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("outTopic", msg);
  }
}
