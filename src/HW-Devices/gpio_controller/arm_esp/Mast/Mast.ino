#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

typedef struct struct_message {
  int d;
  
} struct_message;

struct_message myData;

Servo mast;

void OnDataRecv(const esp_now_recv_info *info,
                const uint8_t *incomingData,
                int len) {

  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println(myData.d);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Initialized ESP-NOW");
  Serial.println(WiFi.macAddress());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  mast.attach(3);
}

void loop() {
  String incoming = Serial.readStringUntil('\n');
  incoming.trim();
  if (incoming[0] == 'M') {
    String num = incoming.substring(1);
    int us = num.toInt();
    mast.write(us);
  }
}
