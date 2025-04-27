#include <esp_now.h>
#include <WiFi.h>

#define PIN_LEFT 11
#define PIN_RIGHT 10
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
   int left;
   int right;
} struct_message;

// Create a struct_message called myData
struct_message myData;
int setfwd = 0;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Right:\t%i\nLeft:\t%i\n", myData.left, myData.right);
  if (myData.left) {
    if (myData.right) {
          digitalWrite(PIN_RIGHT, HIGH);
          digitalWrite(PIN_LEFT, HIGH);
          Serial.println("set STOP 11");
    } else {
          digitalWrite(PIN_RIGHT, LOW);
          digitalWrite(PIN_LEFT, HIGH);
          Serial.println("set LEFT 10");
    }
  } else {
    if (myData.right) {
        digitalWrite(PIN_RIGHT, HIGH);
        digitalWrite(PIN_LEFT, LOW);
        Serial.println("set RIGHT 01");
    } else {
        digitalWrite(PIN_RIGHT, LOW);
        digitalWrite(PIN_LEFT, LOW);
        Serial.println("set FORWARD 00");
    }
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
}
 
void loop() {

}