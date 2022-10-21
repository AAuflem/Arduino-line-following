#include <Arduino.h>
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
  setPoint =0;
  //pinMode(DirB, OUTPUT);
  
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

// kode for svinging, bruker % for å angi relativ hastighe mellom hjulene, TDOD: teste og evt skrive kode for konvertering mellom input og %
//tar inn flyttall som skaleringsfaktor (0.0 og oppover) 
void turnR(double percent){
  double scaleFactor = 1.0; //scales the turning speed down when below 1.0

  double upscalingFactor = (double)(maxSpeed/speed);
  double downscalingFactor =1.0;
  double diff = percent - upscalingFactor;
  speed = (int) speed*scaleFactor; //allowed to become an int
  if(diff<=0){
    MotorA(1, speed * (1+percent)); // mulig denne må forandres
    MotorB(1, speed);
  }
  else if(diff>0){
    upscalingFactor = percent - diff;      // should give max-speed
    downscalingFactor = 1 -diff;         //reduces speed of second wheel
    MotorA(1, speed * upscalingFactor);
    MotorB(1, speed * downscalingFactor);
  }
}


void turnL(double percent){
  double scaleFactor = 1.0; //scales the turning speed down when below 1.0

  double upscalingFactor = (double)(maxSpeed/speed);
  double downscalingFactor =1.0;
  double diff = percent - upscalingFactor;
  speed = (int) speed*scaleFactor;
  if(diff<=0){
    MotorB(1, speed * (1+percent)); // mulig denne må forandres
    MotorA(1, speed);
  }
  else if(diff>0){
    upscalingFactor = percent - diff;      // should give max-speed
    downscalingFactor = 1 -diff;         //reduces speed of second wheel
    MotorB(1, speed * upscalingFactor);
    MotorA(1, speed * downscalingFactor);
  }
}
//dir = 1 forward, dir = -1 backwards, dir = 0 stop
void goStraight(int dir, double factor){

  MotorA(dir, speed* factor);
  MotorB(dir, speed* factor);
}



void turnInPlaceR(){
  double scalingFactor = 1; //  can be changed to change the speed (0-1)
  MotorA(1, speed * scalingFactor);
  MotorB(-1, speed * scalingFactor);
}

void turnInPlaceL(){
  double scalingFactor = 1; //  can be changed to change the speed (0-1)
  MotorB(1, speed * scalingFactor);
  MotorA(-1, speed * scalingFactor);
}


// returns the turning direction (0 == straight, 1== right, -1 == left), if it shoud keep going (0 == stop, 1 == go, -1 == back up) and if there is a T-section
 void lineDetection(){
      // when the digitalRead is LOW, no signal is returning to the sensor. aka the sensor is detecting the black line.
      // the middle sensor is opposite, (LOW/HIGH)
    if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)==HIGH and digitalRead(IR_R)==HIGH)
    {
      // code for driving straight
      lineDetectArray[0]= 0;
      lineDetectArray[1]= 1;
      lineDetectArray[2]= 0;
    }
    if(digitalRead(IR_L)==LOW and digitalRead(IR_C)==LOW and digitalRead(IR_R)==HIGH) 
    {
      // code when ended up to the right
      lineDetectArray[0]= -1;
      lineDetectArray[1]= 1;
      lineDetectArray[2]= 0;

    }
//
    if(digitalRead(IR_L)==LOW and digitalRead(IR_C)==HIGH and digitalRead(IR_R)==HIGH) 
    {
      // code when ended up to the right
      lineDetectArray[0]= -1; // 0.5 AND MAKING IT A DOUBLE-ARRAY?
      lineDetectArray[1]= 1;
      lineDetectArray[2]= 0;

    }
//
    if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)==LOW and digitalRead(IR_R)==LOW) 
    {
      // code when ended up to the Left
      lineDetectArray[0]= 1;
      lineDetectArray[1]= 1;
      lineDetectArray[2]= 0;

    }
    //
    if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)==HIGH and digitalRead(IR_R)==LOW)
    {
      // code when ended up to the left
      lineDetectArray[0]= 1; //0.5 AND MAKING IT A DOUBLE-ARRAY?
      lineDetectArray[1]= 1;
      lineDetectArray[2]= 0;

    }
    //
    if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)==LOW and digitalRead(IR_R)==HIGH) 
    {
      // stop? // T-cross
      lineDetectArray[0]= 0;
      lineDetectArray[1]= -1;
      lineDetectArray[2]= 0;
    }
    if(digitalRead(IR_L)==LOW and digitalRead(IR_C)==HIGH and digitalRead(IR_R)==LOW) 
    {
      // Detecting a T section, need a T_counter
      // T_count +=
      lineDetectArray[0]= 0;
      lineDetectArray[1]= 0;
      lineDetectArray[2]= 1;
    }
}


void followLine(){
double setTurn =3;
lineDetection();
  if(lineDetectArray[1]==1){

    if(lineDetectArray[0] == 0){
      goStraight(1, 1.0);
    }

    if(lineDetectArray[0]== 1){
      turnR(setTurn);
    }
    if(lineDetectArray[0]== -1)
      turnL(setTurn);
    }
  else if(lineDetectArray[1]== 0){ //stopping
    goStraight(0, 0); 
  }
  else if(lineDetectArray[1]== -1){ //reversing
    goStraight(-1, 0.5);
  }
}

//pid that uses lineDetection() to create a distance
double turnPID(double dir){

  currentTime = millis();
  elapsedTime = (double) (currentTime- previousTime);

  error = setPoint - dir;       // computiong the error, is eiter +1 or -1  when not following the line, not accounted for t-sections and running off track
  cumError +=  error * elapsedTime; //integrating
  rateError = (error - lastError)/elapsedTime; //derivative

  double out = kp*error + ki*cumError + kd*rateError; //summing and scaling the factors of the PID. 
  // rate error is only usefull for instant correction
  // error is constant
  // cumError gives how aggressive the car shoud be at correcting over time.

  //assigning previous/last values for the next round
  previousTime = currentTime;
  lastError = error;

  return out; // not shure about the size of the values
  
}

void followLinePID(){
  double totScaleFactor =1.0; // for scaling the output from turnPID(), For emergencies
  lineDetection(); //initializing the line-detection

  double dirPID = turnPID(lineDetectArray[0])* totScaleFactor;
  if(lineDetectArray[1]==1){

    if(dirPID == 0){
      goStraight(1, 1.0);
    }

    if(dirPID >0){
      turnR(dirPID);
    }
    else{
      turnL(abs(dirPID));
    }
  }
  else if(lineDetectArray[1]== 0){
    goStraight(0, 1);
  }
  else if(lineDetectArray[1]== -1){
    goStraight(-1, 1);
  }
}



// ----------------------------|||||||||||||| main loop |||||||||||||------------------------

// main loop, for now just for testing
void loop() {
  time = millis();
  speed =200;
  //speed =110;
  //speed = (int) (time/100) % 255;
  // if (time<10000){
  //   // turnInPlaceR();
  //   turnR(1);
  // }
  // else{
  //   // turnInPlaceL();
  //   turnL(1);
  // }
  // }
  // else{
  //   turnL(1.0);
  // }
  followLine();
  // if(time <5000){
  // //  goStraight(-1, 1);
  //   turnR(2);
  // }
  // if (time <= 10000 && time >=5000){
  //   // goStraight(0, 1);
  //    goStraight(1, 1); 
  // }
  // if (time > 10000){
  // turnL(2); 
  // }
// lineDetection();
}