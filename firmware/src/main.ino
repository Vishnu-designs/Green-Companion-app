/* Green Companion - ESP8266 + HC-SR04 -> MQTT example
   - Publishes telemetry to: devices/{deviceId}/telemetry
   - JSON payload: { "deviceId", "timestamp", "distance_cm", "level_pct" }
   - Supports OTA (ArduinoOTA)
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

// ----- CONFIG -----
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char* MQTT_SERVER = "192.168.1.100"; // replace with your broker IP or hostname
const uint16_t MQTT_PORT = 1883;
const char* DEVICE_ID = "bin001";
const int PUBLISH_INTERVAL_MS = 30000; // 30 seconds
const float BIN_HEIGHT_CM = 40.0; // measured internal height of bin to sensor

// Pins (NodeMCU mapping)
const uint8_t TRIG_PIN = D1; // GPIO5
const uint8_t ECHO_PIN = D2; // GPIO4

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastPublish = 0;

void setupWiFi() {
  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++tries > 60) { // 30s timeout
      Serial.println("\nWiFi connect timeout, restarting...");
      ESP.restart();
    }
  }
  Serial.println();
  Serial.print("WiFi connected, IP=");
  Serial.println(WiFi.localIP());
}

void setupOTA() {
  ArduinoOTA.setHostname(DEVICE_ID);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA end");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect(DEVICE_ID)) {
      Serial.println("connected");
      // subscribe if required: mqttClient.subscribe("devices/commands");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 2s");
      delay(2000);
    }
  }
}

long readDistanceCm() {
  // HC-SR04 trigger/echo
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // pulseIn returns microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout -> ~5m
  if (duration == 0) return -1; // timeout/error
  long distanceCm = duration * 0.0343 / 2.0;
  return distanceCm;
}

float distanceToLevelPct(float distance_cm) {
  if (distance_cm < 0) return 0.0;
  float level = (1.0 - (distance_cm / BIN_HEIGHT_CM)) * 100.0;
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  return level;
}

String isoTimestamp() {
  // Simple timestamp: unix millis -> You can replace with NTP for real time
  unsigned long t = millis() / 1000;
  char buf[32];
  sprintf(buf, "%lu", t);
  return String(buf);
}

void publishTelemetry(float distance_cm, float level_pct) {
  String topic = String("devices/") + DEVICE_ID + "/telemetry";
  String payload = "{";
  payload += "\"deviceId\":\"" + String(DEVICE_ID) + "\",";
  payload += "\"timestamp\":\"" + isoTimestamp() + "\",";
  payload += "\"distance_cm\":" + String(distance_cm, 2) + ",";
  payload += "\"level_pct\":" + String(level_pct, 1);
  payload += "}";
  Serial.print("Publishing to ");
  Serial.print(topic);
  Serial.print(" payload: ");
  Serial.println(payload);
  mqttClient.publish(topic.c_str(), payload.c_str());
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  setupWiFi();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  setupOTA();
}

void loop() {
  ArduinoOTA.handle();

  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = now;
    long dist = readDistanceCm();
    float level = distanceToLevelPct((float)dist);
    publishTelemetry((float)dist, level);
  }

  delay(50);
}
