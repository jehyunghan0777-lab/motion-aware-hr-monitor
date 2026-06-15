#include <Arduino.h>
#include <WiFi.h>

HardwareSerial STM32Serial(1);

const char* ssid = "ESP32_WIFI";
const char* password = "12345678";

WiFiServer server(3333);
WiFiClient client;

String received;

void setup() {
  Serial.begin(115200);
  delay(1000);

  STM32Serial.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("ESP32 Serial Communication with STM32 Initialized");

  WiFi.mode(WIFI_AP);

  if (!WiFi.softAP(ssid, password)) {
    Serial.println("Failed to start WiFi AP!");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("WiFi AP started successfully!");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  Serial.println("TCP server started on port 3333");
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();

    if (client) {
      Serial.println("New client connected");
      client.setNoDelay(true);
    }
  }

  while (STM32Serial.available()) {
    char c = STM32Serial.read();

    if (c == '\n') {
      received.trim();

      if (received.length() > 0) {
        if (client && client.connected()) {
          client.println(received);
        }

        Serial.print("Sent: ");
        Serial.println(received);
      }

      received = "";
    } else {
      received += c;
    }
  }
}