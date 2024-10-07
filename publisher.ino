#include <WiFiNINA.h>       // Include the WiFiNINA library for Wi-Fi functionality
#include <ArduinoMqttClient.h> // Include the Arduino MQTT client library to communicate via MQTT

// Wi-Fi network credentials
char wifiSSID[] = "Kartikey";  // Wi-Fi SSID (network name)
char wifiPassword[] = "kartikey";     // Wi-Fi password

// Pin configurations for ultrasonic sensor and LED
const int triggerPin = 9; // Ultrasonic sensor trigger pin
const int echoPin = 10;   // Ultrasonic sensor echo pin
const int ledPin = 6;     // LED pin (not used in this code)

// Variables for calculating distance using the ultrasonic sensor
float duration, distance; // Duration for pulse travel and the calculated distance
float speedOfSound = 0.034; // Speed of sound in cm/us (centimeters per microsecond)

// Create Wi-Fi and MQTT client objects
WiFiClient wifiClient;              // Wi-Fi client object to handle Wi-Fi connection
MqttClient mqttClient(wifiClient);  // MQTT client object using the Wi-Fi connection

String lastMessage = "";  // Stores the last sent MQTT message

// MQTT broker details
const char mqttBroker[] = "broker.emqx.io"; // URL of the MQTT broker (public broker in this case)
int mqttPort = 1883;                         // MQTT uses port 1883 by default
const char mqttTopic[] = "SIT210/waves";     // MQTT topic to publish messages to

// Timing variables for controlling sensor measurement intervals
const long interval = 1000; // Set interval to 1000 milliseconds (1 second)
long prevMills = 0;         // Store the previous time the distance was measured

void setup() {
  // Initialize serial communication for debugging at a baud rate of 9600
  Serial.begin(9600);

  // Set the pin modes for the ultrasonic sensor
  pinMode(triggerPin, OUTPUT); // Set trigger pin as output
  pinMode(echoPin, INPUT);     // Set echo pin as input

  connectToWiFi();         // Connect to the Wi-Fi network
  connectToMQTTBroker();   // Connect to the MQTT broker
}

void loop() {
  // Check if the MQTT client is connected to the broker; if not, reconnect
  if (!mqttClient.connected()) {
    connectToMQTTBroker(); // Reconnect to the MQTT broker if disconnected
  }
  
  mqttClient.poll();  // Check for new messages and keep the MQTT client alive

  long currMills = millis(); // Get the current time in milliseconds

  // If the time since the last distance measurement exceeds the interval, take a new measurement
  if (currMills - prevMills >= interval) {
    prevMills = currMills;   // Update the previous time to the current time

    distance = MeasureDistance();  // Measure the distance using the ultrasonic sensor

    Serial.print("Distance: ");    // Print the distance to the serial monitor for debugging
    Serial.println(distance);

    message(); // Send an MQTT message based on the distance measured
  }
}

void connectToWiFi() {
  // Print the Wi-Fi SSID (name) to which we're connecting
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifiSSID);

  // Keep attempting to connect to Wi-Fi until successful
  while (WiFi.begin(wifiSSID, wifiPassword) != WL_CONNECTED) {
    Serial.print(".");   // Print a dot to indicate connection attempts
    delay(1000);         // Wait 1 second before trying again
  }

  Serial.println("WiFi Connected"); // Print a message when connected
}

void connectToMQTTBroker() {
  // Print the MQTT broker URL we're connecting to
  Serial.print("Connecting to MQTT Broker: ");
  Serial.println(mqttBroker);

  // Keep attempting to connect to the MQTT broker until successful
  while (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print(".");   // Print a dot to indicate connection attempts
    delay(1000);         // Wait 1 second before trying again
  }

  Serial.println("MQTT Broker Connected"); // Print a message when connected
}

float MeasureDistance() {
  // Trigger the ultrasonic sensor to send out a pulse
  digitalWrite(triggerPin, LOW);  // Clear the trigger pin
  delayMicroseconds(3);           // Wait for 3 microseconds
  digitalWrite(triggerPin, HIGH); // Set the trigger pin HIGH to send a pulse
  delayMicroseconds(10);          // Wait for 10 microseconds (pulse duration)
  digitalWrite(triggerPin, LOW);  // Set the trigger pin LOW again

  // Measure the time it takes for the echo pulse to return
  duration = pulseIn(echoPin, HIGH, 25000); // Get pulse duration with a 25ms timeout

  // Calculate the distance based on the duration (distance = time * speed of sound / 2)
  return (duration * speedOfSound) / 2;
}

void message() {
  // Check if the measured distance is between 0 and 15 cm
  if (distance > 0 && distance < 15) {
    mqttClient.beginMessage(mqttTopic); // Begin composing an MQTT message

      mqttClient.print("Wave");        // Publish "Wave" message
      Serial.println("Wave Message Sent!"); // Print confirmation to serial monitor
    
    mqttClient.endMessage();  // Finish sending the MQTT message

    delay(3000);  // Delay for 3 seconds before the next message (to avoid spamming)
  }
}
