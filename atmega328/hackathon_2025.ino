#include "motorControl.h"
#include <Servo.h>
#define LED_PIN 7
#define RED_LED 12
#define LEFT_ULTRA A4
#define RIGHT_ULTRA A5 
#include "NewPing.h"

#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 400
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myServo;
Motor motor;

void setup() {
  // put your setup code here, to run once:
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(LEFT_ULTRA, INPUT);
  pinMode(RIGHT_ULTRA, INPUT);
  myServo.attach(11);
  Serial.begin(9600);
  digitalWrite(RED_LED, HIGH);
  delay(2000);
  digitalWrite(RED_LED, LOW);
}

float measure_distance(float old_distance) {
  float distance;
  // for (int i = 0; i < 3; i++) {
    distance = sonar.ping_cm();
    if (distance < 20.00 && distance != 0) {
      digitalWrite(LED_PIN, LOW);
      motor.stopCar();
      // delay(500);
    } else {
      motor.forward();
      digitalWrite(LED_PIN, HIGH);
      // break;
    }
  // }
  
  if (old_distance < distance || distance == 0) {
    return old_distance;
  }
  return distance;
}

void loop2() {
  myServo.write(0);
  Serial.print("pos = ");
  Serial.println(myServo.read());
  delay(1000);
  myServo.write(45);
  Serial.print("pos = ");
  Serial.println(myServo.read());
  delay(1000);
  myServo.write(360);
  Serial.print("pos = ");
  Serial.println(myServo.read());
  delay(1000);
  myServo.write(45);
  Serial.print("pos = ");
  Serial.println(myServo.read());
  delay(1000);
  // int pos = myServo.read();
  // myServo.write((pos + 5) % 180);
  // 
  // delay(500);
}

void loop() {
  float shortest = 1000;
  myServo.write(0);
  shortest = measure_distance(shortest);
  // Send results to Serial Monitor
  Serial.print("shortest cm = ");
  Serial.println(shortest);
  if (shortest < 20.00 && shortest != 0) {
    myServo.write(180);
    delay(1000);
    float left_dist = sonar.ping_cm();
    if (left_dist > 30 || left_dist == 0) {
      motor.left();
      Serial.print(left_dist);
      Serial.println("swing left");
    } else {
      motor.right();
      Serial.print(left_dist);
      Serial.println(" swing right");
    }
    delay(1000);
    myServo.write(0);
    motor.stopCar();
  } else {
    int leftVal = digitalRead(LEFT_ULTRA);
    int rightVal = digitalRead(RIGHT_ULTRA);
    if (leftVal == HIGH && rightVal == HIGH) {
      motor.stopCar();
      Serial.println("command: Stop");
    } else if (leftVal == HIGH && rightVal == LOW) {
      motor.left();
      Serial.println("command: Left");
    } else if (leftVal == LOW && rightVal == HIGH) {
      motor.right();
      Serial.println("command: Right");
    } else {
      Serial.println("no command, so forward");
      motor.forward();
    }
    delay(500);
  }
}