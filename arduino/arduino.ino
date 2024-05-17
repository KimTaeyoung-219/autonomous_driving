#include <Car_Library.h>

int motorA1=2;
int motorA2=3;
int motorA3=5;
int motorA4=4;

int motorDirection1=8;
int motorDirection2=9;
int analogPin=A0;

int val;
int num=0;
int coord;
int speed;

void setup(){
  Serial.begin(9600);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorA3, OUTPUT);
  pinMode(motorA4, OUTPUT);
  pinMode(motorDirection1, OUTPUT);
  pinMode(motorDirection2, OUTPUT);
  pinMode(analogPin,INPUT);
}

void loop(){
  int len;
  if ((len = Serial.available()) >= 4) {
    num+=1;
    // Serial.readBytes((char*)&speed, 4);
    // Serial.readBytes((char*)&coord, 4);

    uint8_t buffer[4];
    Serial.readBytes(buffer, 4);

    memcpy(&speed, buffer, sizeof(uint16_t));
    memcpy(&coord, buffer + sizeof(uint16_t), sizeof(uint16_t));

    int left_speed = speed;
    int right_speed = speed;

    motor_forward(motorA1, motorA2, speed);
    motor_forward(motorA3, motorA4, speed);

    val = potentiometer_Read(analogPin);

    // if (num%10000 == 0){
      // Serial.print("speed: ");
      // Serial.println(speed);
      // Serial.print("coordinate: ");
      // Serial.println(coord);
    // }
    
    if(val<coord){
      analogWrite(motorDirection1, LOW);
      analogWrite(motorDirection2, 100);
    }
    else if(val>coord){
      analogWrite(motorDirection1, 100);
      analogWrite(motorDirection2, LOW);
    }
    else if(val==coord){
      analogWrite(motorDirection1, LOW);
      analogWrite(motorDirection2, LOW);
    }
  }
}











