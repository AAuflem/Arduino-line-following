#include <Arduino.h>
#include <Servo.h>
//for the different modes in the different stages
int speed = 100; // ----------------------------------------------

enum STAGE{INIT, FIRST, SECOND, THIRD, FOURTH};
STAGE current_stage{INIT};




// enum FirstStage{Straight, TSection, Cup};
// FirstStage current_part_firstStage{Straight};

enum FirstStage {Straight, TSection, Grabber_prepare, Go_To_Cup , Grab_state, Turn_state, Drive_state1, Drive_state2, Lower_state, Release_state, getOut, TSection2 ,Go_Next_state}; // Enumerator for system states
FirstStage current_firstStage{Straight};



enum ThirdStage{T1, OBSTACLES, T2, to_Fourth};
ThirdStage current_thirdStage{T1};

enum FourthStage{Begin, Search, Fetch, GoBack, FindSpot, PutDown, Neutral, End};
FourthStage current_fourthStage{Begin};
// ---------------------- pins for 
// line detection pins
const int IR_L = 8;
const int IR_C = 12;
const int IR_R = 13;
//motor controll declarations (pins)
const int SPDA = 5;
const int SPDB = 6;
const int DirB = 7;
const int DirA = 4;
// cup sensor-pin
const int CupDist = 3;
// camera
const int Camera = 2;
const int Camera_center = 10;

//GRABBER
const int ServoM = 9;

// turning in place speed
const int turningSpeed90 =160;
int turningSpeed = 160;
const int turningSpeed180 = 180;


// time constant top turn 90 degrees
const int turn90Time = 800;

//time constant to turn 180 degrees
const int turn180Time = 2100;


// cup-side variable
int cupSide = 0; 

// grabber
Servo myservo;
const int posClosed = 150;
const int posOpen = 10;
const int posLower = 105;

unsigned long lastTime, stepTurnTimestamp;

//motor controll declaration
// const int maxSpeed = 220; //UNUSED
const double truningScaleFactor = 1; //between 0 and 1 ()
//motorADir = motorADir*255;

//array containing the return-values of lineDetect(); first valu is turning direction, second is driving direction, third is T-section detection, //-1 is left/backwards, while 0 means straight/stop/not detected
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
  pinMode(CupDist, INPUT);
  pinMode(Camera, INPUT);
  pinMode(A1, INPUT); // this might not be needed?
  //setPoint =0;

  pinMode(ServoM, OUTPUT);
  myservo.attach(9);
  myservo.write(posOpen);
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
double turnRate = 1;
void turnRight(){
  MotorA(1, (int) speed*1.5);
  MotorB(-1, (int) speed*1.1* turnRate);
}

void turnLeft(){
  MotorB(1, (int) speed*1.5);
  MotorA(-1, (int) speed*1.1* turnRate);
}

void turnInPlaceR(){
  //double scalingFactor = 1; //  can be changed to change the speed (0-1)
  MotorA(1, (int) turningSpeed * 1.1);
  MotorB(-1, (int) turningSpeed * 1.1);
}

void turnInPlaceL(){
  //double scalingFactor = 1; //  can be changed to change the speed (0-1)
  MotorB(1, (int) turningSpeed * 1.1);
  MotorA(-1, (int) turningSpeed * 1.1);
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

//assumes T-section, call lastTime = millis(); before using this TODO
void complete90TurnR(){
  lastTime = millis();
  while(lastTime - millis() >= 200){
    break;
  }
}



// returns the turning direction (0 == straight, 1== right, -1 == left), if it shoud keep going (0 == stop, 1 == go, -1 == back up) and if there is a T-section
void lineDetection(){
      // when the digitalRead is LOW, no signal is returning to the sensor. aka the sensor is detecting the black line.
      // the middle sensor is opposite, (LOW/HIGH)
  if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)== HIGH and digitalRead(IR_R)==HIGH){
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
  if(digitalRead(IR_L)==HIGH and digitalRead(IR_C)==LOW and digitalRead(IR_R)==HIGH) 
  {
    //back it up
    lineDetectArray[0]= 0;
    lineDetectArray[1]= -1;
    lineDetectArray[2]= 0;
  }
  if(digitalRead(IR_L)==LOW and digitalRead(IR_C)==HIGH and digitalRead(IR_R)==LOW) 
  {
    // Detecting a T section, currently just continuing
    lineDetectArray[0]= 0;
    lineDetectArray[1]= 1;
    lineDetectArray[2]= 1;
  }
  //lastDirBuff();
}



void simpleFollowLine(){
  lineDetection();
   //if(lineDetectArray[2]==0){
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
  
    // else if(lineDetectArray[1]== 0){ //stopping
    //   stop(); 
    // }
    
    if(lineDetectArray[1]== -1){ //reversing
      goStraight(-1, 1);
    }
    
   //}
  //else if(lineDetectArray[2]==1){   //this is implemented in the main code now
  //  stop();
  //}

}

//.----------. not in use yet .----------.

//stores the last non-zero value of detected turning direction // not yet in use
// void lastDirBuff(){
//   if(lineDetectArray[0]!=0){
//     dirBuffer = lineDetectArray[0];
//   }
// }


//------ buuffer for a fortsette en sving selv om man er out of bounds, ikke ferdig

// void outOfBoundsBuffer(){
//   int maxTime =300; //milliseconds
//   time = millis();
//   deltaT += time-prevTime;
//   if( deltaT < maxTime){
//     if(lineDetectArray[0]==0)

   
//     prevTime=  time;
//   }
  
// }


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


//returns true if there is a T-section
bool ifT(){
  if(lineDetectArray[2]==1){
    return true;
  }
  else{ 
    return false;
  }
}


// -------------------- Turning functions -----------------------

void turn90L(){
  while(millis() - lastTime <= turn90Time){ //turns to the correct side to fetch the cup
    turnInPlaceL(); //same as turn180 but with directional controll
  }
  stop();
}

void turn90R(){
	while(millis() - lastTime <= turn90Time){ //turns to the correct side to fetch the cup
    turnInPlaceR(); //same as turn180 but with directional controll
  }
  stop();
}

void startingSpeed(unsigned int Time, int endSpeed){
  if(millis() - Time <= 100){
    speed =140;
  }
  else{ speed = endSpeed; }
}

void stepTurnL(){
  speed = 200;
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= 50){ // edit the time-constants
    turnInPlaceL();
  }
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= 30){ // edit the time-constants
    stop();
  }
}

void stepTurnR(){
 speed = 200;
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= 50){ // edit the time-constants
    turnInPlaceL();
  }
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= 30){ // edit the time-constants
    stop();
  }
}

// ------------------------------------------- startup -------------------------------------------------
unsigned int cameraTurningTime = 400; // 400 when semil-low voltage, 
unsigned int cameraDetectionTime = 7000;
void startup(){
  while(cupSide == 0){
    lastTime = millis();
    while(millis() - lastTime <= cameraTurningTime){ // the constant may need to be changed based on camera angle
        turnInPlaceL();
    }
      stop();
    lastTime = millis();
    while(millis()- lastTime <= cameraDetectionTime){ // 500 is the assumed time it takes for the rb-pi to detect the cup, with some slack
      if (digitalRead(Camera) != LOW){
        cupSide = -1;
        break;
      }
    }
		lastTime = millis();
    while(millis() - lastTime <= cameraTurningTime){ // the constant may need to be changed based on camera angle
      turnInPlaceR();
    }
    stop();
    if(cupSide != 0){
      break;
    }


		lastTime = millis();
    while(millis() - lastTime <= cameraTurningTime){ // the constant need to be changed based on camera angle
      turnInPlaceR();
    }
    stop();
    lastTime = millis();
    while(millis()- lastTime <= cameraDetectionTime){
      if (digitalRead(Camera) != LOW){
        cupSide = 1;
        break;
      }
    }
		lastTime = millis();
    while(millis() - lastTime <= cameraTurningTime){ // the constant need to be changed based on camera angle
      turnInPlaceL();
    }
    stop();
  }
}


// ---- case switches for the stages
int stage1Fin=0; // this didnt seem to work

void Stage1(){
  switch(current_firstStage){
    case (Straight):
      if(millis() - lastTime <= 50){
        speed =150;
      }
      speed = 100;
      simpleFollowLine();
      if (lineDetectArray[2]== 1){
        stop();
        current_firstStage = TSection;
        lastTime = millis();
        break;
      }
      break;

    case(TSection):

      if(cupSide == -1){
			  while(millis() - lastTime <= turn90Time){ //turns to the correct side to fetch the cup
          turnInPlaceL(); //same as turn180 but with directional controll
        }
      }
      else if(cupSide == 1){
        while(millis() - lastTime <= turn90Time){
          turnInPlaceR();
        }
      } 
      // potential problem if there for some reason is another value than 1 or -1 .......
      stop();
			current_firstStage = Grabber_prepare;
			lastTime = millis();
      break;


		case (Grabber_prepare):
      Release(); // probably have this
      if(millis() - lastTime >= 500){
			  lastTime = millis();
			  current_firstStage = Go_To_Cup;
        break;
      }
		  break;


    case (Go_To_Cup):
      startingSpeed(lastTime, 100);
      simpleFollowLine();
      if(digitalRead(CupDist) == LOW){ //LOW = close
        stop();
        current_firstStage = Grab_state;
        lastTime = millis();
        break;
      }
      break;
		
		case (Grab_state):
			Grab();
			if (millis() - lastTime >= 1500){
				lastTime = millis();
				current_firstStage = Turn_state;
				break;
			}
		break;

		case (Turn_state):
      turningSpeed = turningSpeed180;
			while(millis() - lastTime <= turn180Time){ //asumption of 180 turn
			  turnInPlaceR(); //left side-truning seems to be the most consistent
        }
      turningSpeed = turningSpeed90;

			stop();
			lastTime = millis();
			current_firstStage = Drive_state1;
		break;
    
    case (Drive_state1): // make this a searchin algorithm??
      simpleFollowLine();
      if(millis() - lastTime >= 3000){
        current_firstStage = Drive_state2;
        lastTime = millis();
      break;
      }
    break;

    case(Drive_state2):
      //goStraight(1, 1.0); // change this back if it dosent work
      simpleFollowLine();
      //lineDetection();
      //simpleFollowLine();
      if(lineDetectArray[1] == -1){ // checks if its out of bounds (end of tape). mulig vi må definere noe mer for T-sections og for å plassere helt riktig
        stop();
        current_firstStage = Lower_state;
        lastTime = millis();
        break;
      }
    break;

		case (Lower_state):
      while(millis() - lastTime <= 500)
      {
        Lower();
      }
      while (millis() - lastTime <= 1000)
      {
        Release();
        current_firstStage = getOut;
        lastTime = millis();
        break;
      }
    break;
      /*Lower();
      if (millis() - lastTime >= 800){
				lastTime = millis();
        current_firstStage = Release_state;
        break;
      }
      
      break;

    case (Release_state):
      Release();    // probobly edit this
      if( millis()- lastTime >= 300){
        current_firstStage = getOut;
        lastTime = millis();
      }
      break;
    */

    case (getOut):
        startingSpeed((lastTime -300), 80);
			  goStraight(-1, 1.0); // whatch this closely now
        lineDetection();

			  if(ifT()){ // if T_section detected)
          stop();
          current_firstStage = TSection2;
          lastTime= millis();
          break;
        }
			
      break;

    case (TSection2):
      if(cupSide == -1){ 
        turn90L();
        //stop();
        //break;
      }
      if(cupSide == 1){
        turn90R();
        //stop();
        //break;
      }
      Grab(); // closing the grabber to get it out of the way
      current_firstStage = Go_Next_state;
      lastTime = millis();
      break;

    case (Go_Next_state):
      if(millis()- lastTime <=400){
      goStraight(1,1.0);
      }
      else{ // no new time-stamp this time, might be nice to have tho
        simpleFollowLine();
        if(millis() - lastTime >= 3000){
          current_stage = SECOND;
          stage1Fin = 1; // aditional atempt at getting out of stage1, dosent seem to work
          lastTime = millis();
          break;
        }
      }
      break;

    default: //this might break the code, or fix it, Might be only used at the beginning if nothing else is defined...
      current_stage = SECOND;
      break;

    }
}



void Stage2(){
  speed= 150; // get this to as much as possible, maybe adjust for entering and exiting the stage?
  simpleFollowLine();
}

void stage3(){
switch(current_thirdStage){
  case (T1):
    turn90L();
    //turnInPlaceL();
    //if(millis()- lastTime >= turn90Time){
    current_thirdStage = OBSTACLES;
    lastTime = millis();
      //break;
    //}
    break;
  case (OBSTACLES):
    speed = 170; // testing this value
    // add different turning??--------------------------------
    simpleFollowLine();
    if(millis()- lastTime >= 3500){ // increasing speed in the "end" of stage 3
      speed = 200;
      turnRate = 1.3; // this is to get past the second beam and turn afterwards, it might be bad for the sand-pit
    }
    if(lineDetectArray[2] ==1){
      current_thirdStage = T2;
      break;
    }
  break;
  
  case (T2):
    turnInPlaceL();
    if(millis()- lastTime >= turn90Time){
      current_thirdStage = to_Fourth;
      break;
    }
    break;
  case (to_Fourth):
    simpleFollowLine();
    break;
  
  }
}

int counter = 0;
void stageFour(){
  switch (current_fourthStage){
    case (Begin):
      simpleFollowLine();
      if(ifT()){
        stop();

        current_fourthStage = Search;
        break;
      }
      break;

    case(Search):
      if(millis()- lastTime <= 3000){
        if(analogRead(A1) == HIGH){
          current_fourthStage = Fetch;
          lastTime = millis();
          break;
        }
        break;
      }
      else{
        stepTurnL();
        counter += 1;
        lastTime = millis();
        if(counter >= 10){
          turn90R();
        }
      }


      stepTurnL();
      break;
  }
}

// ----------------------------|||||||||||||| main loop |||||||||||||------------------------

// main loop, for now just for testing
void loop(){
  switch(current_stage){
    case (INIT): // only runs this one time
      //delay(2000); // This is now in the Setup-part
      speed = 80; // shouldnt matter
      startup();
      lastTime = millis(); //timestamp
      current_stage = FIRST;
      break;

    case (FIRST):

      Stage1();
      if(stage1Fin==1){ 
        current_stage=SECOND;
      }
      break;
    
    case(SECOND):
      Stage2();
      if(ifT()){
        stop();
        current_stage = THIRD;
        lastTime = millis();
        break;
      }
      break;

    case (THIRD):
      stage3();
      break;

    case (FOURTH):

      break;
  }
}