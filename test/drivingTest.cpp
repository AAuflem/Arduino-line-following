#include <Arduino.h>
#include <Arduino.h>
#include <Servo.h>
//for the different modes in the different stages
int speed = 100; // ----------------------------------------------

// line detection pins
const int IR_L = 8;
const int IR_C = 12;
const int IR_R = 13;
//motor controll declarations (pins)
const int SPDA = 5;
const int SPDB = 6;
const int DirB = 7;
const int DirA = 4;

// turning in place speed
int turningSpeed = 140;

// time constant top turn 90 degrees
const int turn90Time = 500;

//time constant to turn 180 degrees
const int turn180Time = 1000;

unsigned long lastTime;

const int ServoM = 9;
Servo myservo;
const int posClosed = 150;
const int posOpen = 10;
const int posLower = 105;


//array containing the return-values of lineDetect();
int lineDetectArray[3] = {0, 0, 0};

//buffers
int dirBuffer;
int deltaT;
unsigned long prevTime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR_L,INPUT);
  pinMode(IR_C,INPUT);
  pinMode(IR_R,INPUT);
  pinMode(SPDA, OUTPUT);
  pinMode(DirA, OUTPUT);
  pinMode(SPDB, OUTPUT);
  pinMode(DirB, OUTPUT);  

 pinMode(ServoM, OUTPUT);
  myservo.attach(9);
  myservo.write(posClosed);
  //pinMode(DirB, OUTPUT);
  delay(2000);
  
}

//motor A direction and speed, first wheel, motor= 0: stop, motor= 1: forward, motor= -1: backward
void MotorA(int motor, int spd){
  if (motor == 1){
    digitalWrite(DirA, HIGH);
    analogWrite(SPDA, spd);
  }
  if(motor== -1){
    digitalWrite(DirA, LOW);
    analogWrite(SPDA, spd);

  }
    if(motor==0){
    digitalWrite(DirA, LOW);
    analogWrite(SPDA, 0);

}
}

//motor B direction and speed,, second wheel motor= 0: stop, motor= 1: forward, motor= -1: backward
void MotorB(int motor, int spd){
  if (motor == 1){
    digitalWrite(DirB, HIGH);
    analogWrite(SPDB, spd);
  }
  if(motor== -1){
    digitalWrite(DirB, LOW);
    analogWrite(SPDB, spd);

  }
    if(motor==0){
    digitalWrite(DirB, LOW);
    analogWrite(SPDB, 0);

}
}
//---
void turnRight(){
  MotorA(1, speed*1.4);
  MotorB(-1, speed*0.9);
}

void turnLeft(){
  MotorB(1, speed*1.4);
  MotorA(-1, speed*0.9);
}

void turnInPlaceR(){
  //double scalingFactor = 1; //  can be changed to change the speed (0-1)
  MotorA(1, turningSpeed);
  MotorB(-1, turningSpeed);
}

void turnInPlaceL(){
  //double scalingFactor = 1; //  can be changed to change the speed (0-1)
  MotorB(1, turningSpeed);
  MotorA(-1, turningSpeed);
}

//dir = 1 forward, dir = -1 backwards, dir = 0 stop
void goStraight(int dir, double factor){
  int scaledSpeed= (int) (speed* factor);
  MotorA(dir, scaledSpeed);
  MotorB(dir, scaledSpeed);
}

void stop(){
  MotorA(0, 0);
  MotorB(0, 0);
}
//---

//assumes T-section, call lastTime = millis(); before using this
void complete90TurnR(){
  lastTime = millis();
  while(lastTime - millis() >= 200){

  }
}
// }
// returns the turning direction (0 == straight, 1== right, -1 == left), if it shoud keep going (0 == stop, 1 == go, -1 == back up) and if there is a T-section
void lineDetection(){
      // when the digitalRead is LOW, no signal is returning to the sensor. aka the sensor is detecting the black line.
      // the middle sensor is opposite, (LOW/HIGH)
  if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)== LOW and digitalRead(IR_R)==HIGH){
      // code for driving straight
    lineDetectArray[0]= 0; //turning direction (let/right)
    lineDetectArray[1]= 1; //speed-direcion (forward/back)
    lineDetectArray[2]= 0; //detercted T-section? (yes/no = 1/0)
  }
  if(digitalRead(IR_L)==LOW and digitalRead(IR_C)==HIGH and digitalRead(IR_R)==HIGH) {
    // code when ended up to the right while detecting centre
    lineDetectArray[0]= -1; // 0.5 AND MAKING IT A DOUBLE-ARRAY?
    lineDetectArray[1]= 1;
    lineDetectArray[2]= 0;
  }
//
  if(digitalRead(IR_L)==LOW and digitalRead(IR_C)==LOW and digitalRead(IR_R)==HIGH) 
  {
    // code when ended up to the right
    lineDetectArray[0]= -1; 
    lineDetectArray[1]= 1;
    lineDetectArray[2]= 0;
  }
//
  if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)==HIGH and digitalRead(IR_R)==LOW) 
  {
    // code when ended up to the Left AND DETECTING CE3NTRE
    lineDetectArray[0]= 1;//0.5 AND MAKING IT A DOUBLE-ARRAY?
    lineDetectArray[1]= 1;
    lineDetectArray[2]= 0;
  }
  //
  if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)==LOW and digitalRead(IR_R)==LOW)
  {
    // code when ended up to the left
    lineDetectArray[0]= 1; 
    lineDetectArray[1]= 1;
    lineDetectArray[2]= 0;
  }
  //
  if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)==HIGH and digitalRead(IR_R)==HIGH) 
  {
    //back it up
    lineDetectArray[0]= 0;
    lineDetectArray[1]= -1;
    lineDetectArray[2]= 0;
  }
  if(digitalRead(IR_L)==LOW and digitalRead(IR_C)==LOW and digitalRead(IR_R)==LOW) 
  {
    // Detecting a T section, need a T_counter //maybe its the drive straight?
    // T_count +=
    lineDetectArray[0]= 0;
    lineDetectArray[1]= 0;
    lineDetectArray[2]= 1;
  }
  //lastDirBuff();
}



void simpleFollowLine(){
  lineDetection();
   if(lineDetectArray[2]==0){
      if(lineDetectArray[1]==1){

        if(lineDetectArray[0] == 0){
          goStraight(1, 1.0);
        }

        if(lineDetectArray[0]== 1){
          turnRight();
        }
        if(lineDetectArray[0]== -1){
          turnLeft();
        }
      }
    
      else if(lineDetectArray[1]== 0){ //stopping
        stop(); 
      }
      else if(lineDetectArray[1]== -1){ //reversing
        goStraight(-1, 1);
    }
   }
}

void loop(){
  
// while(millis()<= 2000){
    // goStraight(1, 1);
// }
// while(millis() <=4000){
    // turnRight();
// }
// while(millis() <= 6000){
    // turnLeft();
// }
// while(millis() <= 8000){
    // goStraight(-1, 1);
// }
// stop();

//goStraight(1,1);

  myservo.write(posClosed);
  if (millis() >= 6000){
  myservo.write(posOpen);
  }

}