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
int left, right;

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
  if ((len = Serial.available()) >= 6) {
    num+=1;
    // Serial.readBytes((char*)&speed, 4);
    // Serial.readBytes((char*)&coord, 4);
    uint8_t buffer[6];
    Serial.readBytes(buffer, 6);

    memcpy(&left, buffer, sizeof(uint16_t));
    memcpy(&right, buffer + sizeof(uint16_t), sizeof(uint16_t));
    memcpy(&coord, buffer + sizeof(uint16_t) + sizeof(uint16_t), sizeof(uint16_t));

    // coordinate : 28 -> left
    // coordinate : 0 -> right
    // coordinate : 14 -> middle

    val = potentiometer_Read(analogPin);

//     if (num%1 == 0){
//       Serial.print("speed: ");
//       Serial.println(left);
//       // Serial.println(right);
//       Serial.print("coordinate: ");
//       Serial.println(coord);
//       Serial.print("val: ");
//       Serial.println(val);
//     }

    motor_forward(motorA1, motorA2, right);
    motor_forward(motorA3, motorA4, left);
    
    if(val<coord){
      analogWrite(motorDirection1, LOW);
      analogWrite(motorDirection2, 200);
    }
    else if(val>coord){
      analogWrite(motorDirection1, 200);
      analogWrite(motorDirection2, LOW);
    }
    else if(val==coord){
      analogWrite(motorDirection1, LOW);
      analogWrite(motorDirection2, LOW);
    }
  } else {
    val = potentiometer_Read(analogPin);
//     Serial.print(val);
//     Serial.print(" ");
//     Serial.println(coord);

    if(val<coord){
      analogWrite(motorDirection1, LOW);
      analogWrite(motorDirection2, 200);
    }
    else if(val>coord){
      analogWrite(motorDirection1, 200);
      analogWrite(motorDirection2, LOW);
    }
    else if(val==coord){
      analogWrite(motorDirection1, LOW);
      analogWrite(motorDirection2, LOW);
    }
  }
}











