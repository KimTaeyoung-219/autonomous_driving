#include <Car_Library.h>

int motorA1=2;
int motorA2=3;
int motorA3=4;
int motorA4=5;

int motorDirection1=8;
int motorDirection2=9;
int analogPin=A0;

int val;
int num=0;
int coord;
int speed;
int val2;

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
  num += 1;
  int speed = 0;
  // motor_forward(motorA1, motorA2, speed);
  // motor_forward(motorA3, motorA4, speed);
  motor_forward(motorA2, motorA1, speed);
  motor_forward(motorA3, motorA4, speed);

  // 172: max left, 142: middle, 112: max right
  // 168: max left, 140: middle, 112: max right
  coord = 144;
  val = potentiometer_Read(analogPin);

  if(val<coord){
    analogWrite(motorDirection1, LOW);
    analogWrite(motorDirection2, 250);
  }
  else if(val>coord){
    analogWrite(motorDirection1, 250);
    analogWrite(motorDirection2, LOW);
  }
  else if(val==coord){
    analogWrite(motorDirection1, LOW);
    analogWrite(motorDirection2, LOW);
  }

  if(num%1000 == 0){
    Serial.print("val: ");
    Serial.println(val);
  }
}











