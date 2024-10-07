// Subscriber code
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

char wifiSSID[] = "Kartikey";
char wifiPassword[] = "kartikey";


const int ledPin = 6; 

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char mqttBroker[] = "broker.emqx.io";
int mqttPort = 1883;
const char mqttTopic[] = "SIT210/waves";

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  connectToWiFi();
  connectToMQTTBroker();

  mqttClient.subscribe(mqttTopic);
  Serial.print("Subscribed to Topic: ");
  Serial.println(mqttTopic);
  Serial.println("Waiting for messages...");
}

void loop() {
  if (!mqttClient.connected()) {
    connectToMQTTBroker();
  }
  mqttClient.poll();

  while (mqttClient.available()) {
    String message = mqttClient.readString();
    Serial.print("Received Message: ");
    Serial.println(message);
    sendMessage(message);
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifiSSID);

  while (WiFi.begin(wifiSSID, wifiPassword) != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("WiFi Connected");
}

void connectToMQTTBroker() {
  Serial.print("Connecting to MQTT Broker: ");
  Serial.println(mqttBroker);

  while (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("MQTT Broker Connected");
}

void sendMessage(String message) {
  if (message == "Wave") {
    Serial.println("Wave Detected");
    flashLEDWavePattern();
  } 
  
}

void flashLEDWavePattern() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

