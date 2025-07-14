#include "BluetoothSerial.h"
#include <Arduino.h>
BluetoothSerial serialBT;
char BT;
int Speed = 100;
//Right
int R1PWM = 19;
int R2PWM = 21;
//Left
int L1PWM = 23;
int L2PWM = 22;

#define rmf 0
#define rmb 1
#define lmf 2
#define lmb 3

void setup() {
  Serial.begin(115200);
  serialBT.begin("My Soccer Bot YT");  //Your BOT Name
  pinMode(R1PWM, OUTPUT);
  pinMode(R2PWM, OUTPUT);
  pinMode(L1PWM, OUTPUT);
  pinMode(L2PWM, OUTPUT);
  
  // Setup PWM channels using the new LEDC API
  if (!ledcAttach(R1PWM, 5000, 8) || 
      !ledcAttach(R2PWM, 5000, 8) || 
      !ledcAttach(L1PWM, 5000, 8) || 
      !ledcAttach(L2PWM, 5000, 8)) {
    Serial.println("LEDC setup failed!");
    while (1); // Stop if LEDC setup fails
  }
}

void loop() {
  while (serialBT.available()) {
    BT = serialBT.read();
//speed control
    if (BT == '0') Speed = 100;
    if (BT == '1') Speed = 110;
    if (BT == '2') Speed = 120;
    if (BT == '3') Speed = 130;
    if (BT == '4') Speed = 140;
    if (BT == '5') Speed = 150;
    if (BT == '6') Speed = 180;
    if (BT == '7') Speed = 200;
    if (BT == '8') Speed = 220;
    if (BT == '9') Speed = 240;
    if (BT == 'q') Speed = 255;
//movement
    if (BT == 'F') go_forward();
    else if (BT == 'B') go_backward();
    else if (BT == 'L') go_left();
    else if (BT == 'R') go_right();
    else if (BT == 'S') stop();
    else if (BT == 'I') forward_right();
    else if (BT == 'J') backward_right();
    else if (BT == 'G') forward_left();
    else if (BT == 'H') backward_left();
  }
}

void go_forward() {
  ledcWrite(R1PWM, Speed);     //right motor forward 
  ledcWrite(R2PWM, 0);        //right motor backward
  ledcWrite(L1PWM, Speed);   //left motor forward
  ledcWrite(L2PWM, 0);      //left motor backward
}
void go_backward() {
  ledcWrite(R1PWM, 0);
  ledcWrite(R2PWM, Speed);
  ledcWrite(L1PWM, 0);
  ledcWrite(L2PWM, Speed);
}
void go_left() {
  ledcWrite(R1PWM, 0);
  ledcWrite(R2PWM, Speed);
  ledcWrite(L1PWM, Speed);
  ledcWrite(L2PWM, 0);
}
void go_right() {
  ledcWrite(R1PWM, Speed);
  ledcWrite(R2PWM, 0);
  ledcWrite(L1PWM, 0);
  ledcWrite(L2PWM, Speed);
}
void stop() {
  ledcWrite(R1PWM, 0);
  ledcWrite(R2PWM, 0);
  ledcWrite(L1PWM, 0);
  ledcWrite(L2PWM, 0);
}
void forward_right() {
  ledcWrite(R1PWM, Speed);
  ledcWrite(R2PWM, 0);
  ledcWrite(L1PWM, 0);
  ledcWrite(L2PWM, 0);
}
void backward_right() {
  ledcWrite(R1PWM, 0);
  ledcWrite(R2PWM, Speed);
  ledcWrite(L1PWM, 0);
  ledcWrite(L2PWM, 0);
}
void forward_left() {
  ledcWrite(R1PWM, 0);
  ledcWrite(R2PWM, 0);
  ledcWrite(L1PWM, Speed);
  ledcWrite(L2PWM, 0);
}
void backward_left() {
  ledcWrite(R1PWM, 0);
  ledcWrite(R2PWM, 0);
  ledcWrite(L1PWM, 0);
  ledcWrite(L2PWM, Speed);
}
