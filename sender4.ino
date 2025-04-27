#include <esp_now.h>
#include <WiFi.h>
#include "NewPing.h"
#define NUM_SAMPLES 10
const int pinCLK = 39; // Connected to CLK on the rotary encoder
const int pinDT = 40;  // Connected to DT on the rotary encoder

volatile int encoderPos = 0;
int lastEncoderPos = 0;

String movementState = "STOP"; // Could be FORWARD, STOP, LEFT, RIGHT
volatile bool encoderMoved = false;
unsigned long lastEncoderMoveTime = 0;
const unsigned long encoderOverrideTimeout = 1000; // ms (1 second)

int lastCLK;
int currentCLK;
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x34, 0xB7, 0xDA, 0x64, 0x0D, 0x24};
float left_samples[NUM_SAMPLES] = {0};
float right_samples[NUM_SAMPLES] = {0};
int pos_left = 0;
int pos_right = 0;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int left;
  int right;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

#define TRIG_PIN_1 6  //D4
#define ECHO_PIN_1 7  //D3
#define TRIG_PIN_2 10
#define ECHO_PIN_2 11
#define MAX_DISTANCE 400
NewPing sonar(TRIG_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIG_PIN_2, ECHO_PIN_2, MAX_DISTANCE);

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {

  pinMode(pinCLK, INPUT_PULLUP);
  pinMode(pinDT, INPUT_PULLUP);

  // Attach an interrupt to the CLK pin
  attachInterrupt(digitalPinToInterrupt(pinCLK), readEncoder, CHANGE);
  // Init Serial Monitor  
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(TRIG_PIN_1, OUTPUT); // Sets the TRIG_PIN as an Output
  pinMode(ECHO_PIN_1, INPUT); // Sets the ECHO_PIN as an Input
  pinMode(TRIG_PIN_2, OUTPUT); // Sets the TRIG_PIN as an Output
  pinMode(ECHO_PIN_2, INPUT); // Sets the ECHO_PIN as an Input

    // Read the initial state of CLK
  lastCLK = digitalRead(pinCLK);

}

float findMin(float arr[], int size) {
  float minVal = arr[0];
  for (int i = 1; i < size; i++) {
    if (arr[i] < minVal) {
      minVal = arr[i];
    }
  }
  return minVal;
}
 
// Interrupt service routine for reading the encoder
void readEncoder() {
  currentCLK = digitalRead(pinCLK);
  if (currentCLK != lastCLK) {
    if (digitalRead(pinDT) != currentCLK) {
      encoderPos++;
    } else {
      encoderPos--;
    }
    encoderMoved = true; // mark that encoder has moved
    lastEncoderMoveTime = millis(); // timestamp last movement
  }
  lastCLK = currentCLK;
}


void loop() {

  if (encoderPos != lastEncoderPos) {
    Serial.printf("Position: %i\n", encoderPos);
    lastEncoderPos = encoderPos;
  }
  int distance;
  int distance2;
  float dist1 = sonar.ping_cm();
  float dist2 = sonar2.ping_cm();

  if (dist1 == 0) {
    dist1 = 1000;
  }

  if (dist2 == 0) {
    dist2 = 1000;
  }

  if (pos_left < NUM_SAMPLES) {
    // Still filling the array initially
    left_samples[pos_left] = dist1;
    pos_left++;
  } else {
    // Array is full: shift all samples left
    for (int i = 0; i < NUM_SAMPLES - 1; i++) {
      left_samples[i] = left_samples[i + 1];
    }
    // Add the new value at the end
    left_samples[NUM_SAMPLES - 1] = dist1;
  }

    if (pos_right < NUM_SAMPLES) {
    // Still filling the array initially
    right_samples[pos_right] = dist2;
    pos_right++;
  } else {
    // Array is full: shift all samples left
    for (int i = 0; i < NUM_SAMPLES - 1; i++) {
      right_samples[i] = right_samples[i + 1];
    }
    // Add the new value at the end
    right_samples[NUM_SAMPLES - 1] = dist2;
  }

  if (pos_right == NUM_SAMPLES) {
    float minValue = findMin(right_samples, NUM_SAMPLES);
    distance = (int) minValue;
  }
  if (pos_left == NUM_SAMPLES) {
    float minValue2 = findMin(left_samples, NUM_SAMPLES);
    distance2 = (int) minValue2;
  }
  Serial.printf("Distance1: %li\n", distance);
  Serial.printf("Distance2: %li\n", distance2);

  // myData.left = distance;
  // myData.right = distance2;

  unsigned long now = millis();

  // Encoder override active?
  if (encoderMoved && (now - lastEncoderMoveTime <= encoderOverrideTimeout)) {
    // Encoder has moved recently
    if (encoderPos > lastEncoderPos) {
      myData.left  = 0;
      myData.right = 1;
      movementState = "RIGHT";
    } else if (encoderPos < lastEncoderPos) {
      myData.left = 1;
      myData.right = 0;
      movementState = "LEFT";
    }
  } else {
    // Encoder has NOT moved recently — control based on sensors
    encoderMoved = false; // reset encoder move flag

    if ((distance2 < 20) && distance > 20){ // Example threshold 20 cm
      myData.left = 0;
      myData.right = 0;
      movementState = "FORWARD";
    } else {
      myData.left = 1;
      myData.right = 1;
      movementState = "STOP";
    } 
  }

  // Always update last known encoder position
  lastEncoderPos = encoderPos;

  Serial.print("Current Movement: ");
  Serial.println(movementState);

  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(10);
}