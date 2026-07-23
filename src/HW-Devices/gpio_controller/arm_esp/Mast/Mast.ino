#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

uint8_t broadcastAddress[] = {0x20, 0x6E, 0xF1, 0x67, 0x2F, 0xB0};

typedef struct struct_message {
  int d;
  
} struct_message;

struct_message myData;

Servo mast;

esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

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

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Initialized ESP-NOW");
  Serial.println(WiFi.macAddress());

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  mast.attach(3);
}

void loop() {
  String incoming = Serial.readStringUntil('\n');
  incoming.trim();
  if (incoming[0] == 'S') {
    String num = incoming.substring(1);
    int us = num.toInt();
    mast.write(us);
  }else if (incoming[0] == 'M') {
    String str = incoming.substring(1);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t* ) str.c_str(), str.length());
   
    if (result == ESP_OK) {
      Serial.println("Sending confirmed");
    }else {
      Serial.println("Sending error");
    }
  }
}
