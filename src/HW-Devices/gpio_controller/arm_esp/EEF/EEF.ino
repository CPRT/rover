#include <esp_now.h>
#include <WiFi.h>
#include "MorseEncoder.h"

// Variables for test data
int int_value;
float float_value;
bool bool_value = true;
const uint8_t sensorPin = 4;

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x20, 0x6E, 0xF1, 0x69, 0xEE, 0xE0};

// Define a data structure
typedef struct struct_message {
  int d;
} struct_message;

// Create a structured object
struct_message myData;

char morseIn[32];

MorseEncoder morseOut;

// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called when data is sent
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *info,
                const uint8_t *incomingData,
                int len) {
  memcpy(morseIn, incomingData, len);
  morseIn[len] = '\0';
  Serial.println(morseIn);
  morseOut.print(morseIn);
}

void setup() {

  // Set up Serial Monitor
  Serial.begin(115200);
  pinMode(sensorPin, INPUT);

  morseOut.begin(21, 18);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Initialized ESP-NOW");
  Serial.println(WiFi.macAddress());

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {

  myData.d = getDistance();
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  } else {
    Serial.println("Sending error");
  }
  delay(100);
}

int getDistance() {
  int16_t t = pulseIn(sensorPin, HIGH);

  if (t == 0) {
    // pulseIn() did not detect the start of a pulse within 1 second.
    return (-2);
  } else if (t > 1850) {
    // No detection.
    return (-1);
  } else {
    // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
    int16_t d = (t - 1000) * 3 / 4;

    // Limit minimum distance to 0.
    if (d < 0) { d = 0; }

    Serial.print(d);
    Serial.println(" mm");
    Serial.print("Is recieving input: ");
    return d;
  }
}
