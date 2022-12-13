#include <Arduino.h>
#include <Servo.h>
#define LED_R 2
#define LED_G 3
#define buttonPin 5
#define IR_sensor 2
#define ServoM 9
int buttonState = 0;
Servo myservo; 
int posClosed = 150;
int posOpen = 0;
int posLower = 105;
int TimeClosed = 500;
int T_Release = 100;
unsigned long GrabTime;
unsigned long lastTime;
int ScanVal = 0;


// line detection pins
const int IR_L = 8;
const int IR_C = 12;
const int IR_R = 13;
//motor controll declarations (pins)
const int SPDA = 5;
const int SPDB = 6;
const int DirB = 7;
const int DirA = 4;
//PID declarations:
//constants
double kp = 2;  // constant turning
double ki = 2;  // increases the turning over time (be carefull with this one!)
double kd = 5;  // how agressive the turning should start
//variables
unsigned long currentTime, previousTime, time;
double elapsedTime;
double error;
double lastError;
double pidIn, pidOut, setPoint;
double cumError, rateError;

//motor controll declaration
const int maxSpeed = 175;
//const double truningScaleFactor = 1; //between 0 and 1 ()
int speed;
int i = 0;
int n=1;
//motorADir = motorADir*255;

//array containing the return-values of lineDetect();
int lineDetectArray[3] = {0, 0, 0};

//States
enum STATE {INIT, Grab_state, Turn_State1, Lower_state, Release_state}; // Enumerator for system states
STATE current_state{INIT};

unsigned long TurnTime180;
unsigned long TurnTime90;
int TurnSpeed180;


void setup() {
  Serial.begin(9600);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(IR_sensor, INPUT);
  pinMode(ServoM, OUTPUT);
  myservo.attach(9);
  myservo.write(posOpen);
  pinMode(SPDA, OUTPUT);
  pinMode(DirA, OUTPUT);
  pinMode(SPDB, OUTPUT);
  pinMode(DirB, OUTPUT);
  setPoint = 0;
  speed = 80;
  TurnSpeed180 = 150;
  TurnTime180 = 3000;
  TurnTime90 = 1500;

   //timestamps
  currentTime = millis();
}

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

void Grab() 
{
  GrabTime = millis();
  myservo.write(posClosed);
}

void Lower()
{
  myservo.write(posLower);
}

void Release() 
{
  myservo.write(posOpen);
}

void Drive()
{
  MotorB(1, speed);
  MotorA(1, speed);
}

void Stop()
{
  MotorB(0, 0);
  MotorA(0, 0);
}

void turnL()
{
  MotorB(1, 150);
  MotorA(-1, 90);
}

void turnR()
{
  MotorB(-1, TurnSpeed180);
  MotorA(1, TurnSpeed180);
}

void Reverse()
{
  MotorB(-1, speed);
  MotorA(-1, speed);
}

void loop() 
{
    while(true){}
}