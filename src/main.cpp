#include <Arduino.h>
#include <Servo.h>
//for the different modes in the different stages
enum STAGE{STARTUP, FIRST, SECOND, THIRD, FOURTH};
STAGE current_stage{FIRST};

// line detection pins
const int IR_L = 8;
const int IR_C = 12;
const int IR_R = 13;
//motor controll declarations (pins)
const int SPDA = 5;
const int SPDB = 6;
const int DirB = 7;
const int DirA = 4;
// camera
const int camera = 2;
int cupSide = 0;
//GRABBER
#define ServoM 9
Servo myservo;
int posClosed = 150;
int posOpen = 0;
int posLower = 120;

unsigned long lastTime;
enum STATE {INIT, Grab_state, Turn_state, Lower_state, Release_state}; // Enumerator for system states
STATE current_state{INIT};

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
const int maxSpeed = 150;
//const double truningScaleFactor = 1; //between 0 and 1 ()
int speed = 100; // ----------------------------------------------
int i = 0;
int n=1;
//motorADir = motorADir*255;

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
  pinMode(2, INPUT);
  setPoint =0;

  pinMode(ServoM, OUTPUT);
  myservo.attach(9);
  myservo.write(posClosed);
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
//---
void turnRight(){
  MotorA(1, speed*1.6);
  MotorB(-1, speed*0.9);
}

void turnLeft(){
  MotorB(1, speed*1.6);
  MotorA(-1, speed*0.9);
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



// kode for svinging, bruker % for å angi relativ hastighe mellom hjulene, TDOD: teste og evt skrive kode for konvertering mellom input og %
// //tar inn flyttall som skaleringsfaktor (0.0 og oppover) 
// void turnR(double percent){
//   double scaleFactor = 1.0; //scales the turning speed down when below 1.0

//   double upscalingFactor = (double)(maxSpeed/speed);
//   double downscalingFactor =1.0;
//   double diff = percent - upscalingFactor;
//   speed = (int) speed*scaleFactor; //allowed to become an int
//   if(diff<=0){
//     MotorA(1, speed * (1+percent)); // mulig denne må forandres
//     MotorB(1, speed);
//   }
//   else if(diff>0){
//     upscalingFactor = percent - diff;      // should give max-speed
//     downscalingFactor = 1 -diff;         //reduces speed of second wheel
//     MotorA(1, speed * upscalingFactor);
//     MotorB(1, speed * downscalingFactor);
//   }
// }


// void turnL(double percent){
//   double scaleFactor = 1.0; //scales the turning speed down when below 1.0

//   double upscalingFactor = (double)(maxSpeed/speed);
//   double downscalingFactor =1.0;
//   double diff = percent - upscalingFactor;
//   speed = (int) speed*scaleFactor;
//   if(diff<=0){
//     MotorB(1, speed * (1+percent)); // mulig denne må forandres
//     MotorA(1, speed);
//   }
//   else if(diff>0){
//     upscalingFactor = percent - diff;      // should give max-speed
//     downscalingFactor = 1 -diff;         //reduces speed of second wheel
//     MotorB(1, speed * upscalingFactor);
//     MotorA(1, speed * downscalingFactor);
//   }
// }


//stores the last non-zero value of detected turning direction // not yet in use?
void lastDirBuff(){
  if(lineDetectArray[0]!=0){
    dirBuffer = lineDetectArray[0];
  }
}

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
  else if(lineDetectArray[2]==1){ 
    stop();
  }

}


//------------ funksjonene nedenfor brukes ikke ---------------

// void followLine(){
//   double setTurn =3;
//   lineDetection();
//   if(lineDetectArray[1]==1){

//     if(lineDetectArray[0] == 0){
//       goStraight(1, 1.0);
//     }

//     if(lineDetectArray[0]== 1){
//       turnR(setTurn);
//     }
//     if(lineDetectArray[0]== -1)
//       turnL(setTurn);
//     }
//   else if(lineDetectArray[1]== 0){ //stopping
//     goStraight(0, 0); 
//   }
//   else if(lineDetectArray[1]== -1){ //reversing
//     goStraight(-1, 0.5);
//   }
// }

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

// void followLinePID(){
//   double totScaleFactor =1.0; // for scaling the output from turnPID(), For emergencies
//   lineDetection(); //initializing the line-detection

//   double dirPID = turnPID(lineDetectArray[0])* totScaleFactor;
//   if(lineDetectArray[1]==1){

//     if(dirPID == 0){
//       goStraight(1, 1.0);
//     }

//     if(dirPID >0){
//       turnR(dirPID);
//     }
//     else{
//       turnL(abs(dirPID));
//     }
//   }
//   else if(lineDetectArray[1]== 0){
//     goStraight(0, 1);
//   }
//   else if(lineDetectArray[1]== -1){
//     goStraight(-1, 1);
//   }
// }


//ikke ferdig
void outOfBoundsBuffer(){
  int maxTime =300; //milliseconds
  time = millis();
  deltaT += time-prevTime;
  if( deltaT < maxTime){
    if(lineDetectArray[0]==0)

   
    prevTime=  time;
  }
  
}


// grabbing time!
//____________________grabber states
void Grab() 
{
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


void turn180()
{
  MotorB(1, 140);
  MotorA(-1, 140);
}

//_______________grabber case-switch_________
void grabber(){
	switch (current_state)
	{
		case (INIT):
			myservo.write(posOpen);
			lastTime = millis();
			current_state = Grab_state;
		break;
		
		case (Grab_state):
			Grab();
			if (millis() - lastTime >= 500)
			{
				lastTime = millis();
				current_state = Turn_state;
				break;
			}
		break;

		case (Turn_state):
			turn180();
			if(millis() - lastTime >= 1200)
			{
				stop();
				lastTime = millis();
				current_state = Lower_state;
				break;
			}
		break;

		case (Lower_state):
      Lower();
      if (millis() - lastTime >= 500)
      {
				lastTime = millis();
        current_state = Release_state;
        break;
      }
    break;

    case (Release_state):
      Release();
			goStraight(-1, 140);
			while (millis() - lastTime >= 500) 
			{
				stop();
				while(true); // inst this kinda bad?
			}
			
    break;
  }
}




// ----------------------------|||||||||||||| main loop |||||||||||||------------------------

// main loop, for now just for testing
void loop() {
  //time = millis();
  switch(current_stage){
    case(STARTUP):
    lastTime= millis();
    while(millis() - lastTime >= 300){ // the constant need to be changed based on camera angle
        turnInPlaceL();
    }

				lastTime = millis();
        break;
      
  }
}