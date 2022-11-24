#include <Arduino.h>
#include <Servo.h>
//for the different modes in the different stages
int speed = 90; // ----------------------------------------------

enum STAGE{INIT, FIRST, SECOND, THIRD, FOURTH};
STAGE current_stage{INIT};




// enum FirstStage{Straight, TSection, Cup};
// FirstStage current_part_firstStage{Straight};

enum FirstStage {Straight, TSection, Grabber_prepare, Go_To_Cup , Grab_state, Turn_state, Drive_state1, Drive_state2, Lower_state, Release_state, getOut, TSection2 ,Go_Next_state}; // Enumerator for system states
FirstStage current_firstStage{Straight};



enum ThirdStage{T1, OBSTACLES, T2, to_Fourth};
ThirdStage current_thirdStage{T1};

enum FourthStage{GoToleft, Search, Fetch, FindFirstCupPlacement, PutDown, FindSecondCupPlacement, End};
FourthStage current_fourthStage{GoToleft};
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
#define CupDist A0
// camera
const int Camera = 2;
const int Camera_center = 10;

//GRABBER
const int ServoM = 9;

// ----------- OTHER CONSTANTS DECLARATIONS

// turning in place speed
const int turningSpeed90 =140;
int turningSpeed = 140;
const int turningSpeed180 = 160;


// time constant top turn 90 degrees
const int turn90Time = 800;

//time constant to turn 180 degrees
const int turn180Time = 1600;


// cup-side variable
int cupSide = 0; 

// grabber
Servo myservo;
const int posClosed = 100;
const int posOpen = 5;
const int posLower = 90;

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
  while(millis() < 2000){}
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
// Normally set to 1.1 : this one amplifies the backwards component of the turning. actual turning is also ependent on the speed. 1.5 is the current amplification for the 
double turnRate = 1.1;

void turnRight(){
  MotorA(1, (int) speed*1.5);
  MotorB(-1, (int) speed* turnRate);
}

void turnLeft(){
  MotorB(1, (int) speed*1.5);
  MotorA(-1, (int) speed* turnRate);
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

// void bacwardsFollow(){
//   lineDetection();
//    //if(lineDetectArray[2]==0){
//     if(lineDetectArray[1]==1){

//       if(lineDetectArray[0] == 0){
//         goStraight(-1, 1.0);
//       }
//       if(lineDetectArray[0]== 1){
//         turnLeft();
//       }
//       if(lineDetectArray[0]== -1){
//         turnRight();
//       }
//     }
  
//     // else if(lineDetectArray[1]== 0){ //stopping
//     //   stop(); 
//     // }
    
//     if(lineDetectArray[1]== -1){ //reversing
//       goStraight(1, 1);
//     }
// }
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
  lineDetection();
  if(lineDetectArray[2]==1){
    return true;
  }
  else{ 
    return false;
  }
}


// -------------------- Turning functions -----------------------
const int startTurn= 180;
const int goPastT = 40;

void turn90L(){
  lastTime = millis();
  while(millis()- lastTime <= goPastT){ // go a little further than the current T-section so that we have the turning-centre above the T-section(aproximated)
    goStraight(1,1.0);
  }
  lastTime= millis();
  stop();
  while(millis() - lastTime <= startTurn){ //turns for a small amount of time to get out of the current line.
    turnInPlaceL(); //turn in place
  }
  lastTime = millis();
  while(millis()-lastTime <= 2*turn90Time){ // might not want a time-constraint here, but keeping it for now.
    turnInPlaceL();         
    if(digitalRead(IR_L)== LOW){
      stop(); //might want to add correction for the codse, or a more rob ust way of finding the line again
      break;
    }
  }
  lastTime = millis();
}

void turn90R(){
  lastTime = millis();
  while(millis()- lastTime <= goPastT){ // go a little further than the current T-section so that we have the turning-centre above the T-section(aproximated)
    goStraight(1,1.0);
  }
  lastTime= millis();
  stop();
  while(millis() - lastTime <= startTurn){ //turns for a small amount of time to get out of the current line.
    turnInPlaceR(); //turn in place
  }
  lastTime = millis();
  while(millis()-lastTime <= 2*turn90Time){ // might not want a time-constraint here, but keeping it for now. // might create a 
    turnInPlaceR();         
    if(digitalRead(IR_R)== LOW){
      stop(); //might want to add correction for the codse, or a more rob ust way of finding the line again
      break;
    }
  }
  lastTime = millis();
}

//-------------
// this moght not work if it ends up on the T-section
void turn180(){ 
    //lastTime = millis();
  // while(millis()- lastTime <= goPastT){ // Back upp a little bit, Might be bad
  // goStraight(-1, 1.0);
  // }
  lastTime= millis();
  stop();

  while(millis() - lastTime <= (int)startTurn*1.4){ //turns for a small amount of time to get out of the current line, added a factor due to added weight 
    turnInPlaceL(); //turn in place
  }

  lastTime = millis();
  while(millis()-lastTime <= 2*turn180Time){ // using 180-time here instead of 90-time, gives more freedom
    turnInPlaceL();         
    if(digitalRead(IR_L)== LOW){
      stop(); //might want to add correction for the code, or a more rob ust way of finding the line again
      break;
    }
  }
  lastTime = millis();

}

// ------------------------
void startingSpeed(unsigned int Time, int endSpeed){
  if(millis() - Time <= 40){
    speed =120;
  }
  else{ speed = endSpeed; }
}

//-------------Step-Turning-----------------------
const int stepDrivingTime = 100;
const int stepStoppingTime = 500;
const int stepTurningSpeed = 220;
//currently takes 50 + 250 = 300 ms for one use/step, uses its own time-stamp
void stepTurnL(){
  turningSpeed = stepTurningSpeed; 
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= stepDrivingTime){ // edit the time-constants
    turnInPlaceL();
  }
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= stepStoppingTime){ // edit the time-constants
    stop();
  }
  turningSpeed = turningSpeed90;
}

//currently takes 50 + 250 = 300 ms for one use/step, uses its own time-stamp
void stepTurnR(){
  turningSpeed = stepTurningSpeed; 
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= stepDrivingTime){ // edit the time-constants
    turnInPlaceR();
  }
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= stepStoppingTime){ // edit the time-constants
    stop();
  }
  turningSpeed = turningSpeed90;
}

// test these:
void stepTurn90L(){
  //int offCenter =0;
  lastTime = millis();
  while(digitalRead(IR_C ==HIGH)){
    stepTurn90L();
  }
  stepTurnL(); // do one more step to get a little distance
  while(digitalRead(IR_C ==LOW)){ // start searching for the line again
  stepTurnL();
  }
  stop();//you are now above the centerline again

}
// test these:
void stepTurn90R(){
  //int offCenter =0;
  lastTime = millis();
  while(digitalRead(IR_C !=LOW)){
    stepTurn90R();
  }
  stepTurnR(); // do one more step to get a little distance
  while(digitalRead(IR_C !=HIGH)){ // start searching for the line again
  stepTurnR();
  }
  stop();//you are now above the centerline again
}

int prevSpeed= 0;
//dir HAS to be 1 or -1 (or 0 but dosent make sense)
void stepStraight(int dir){
  prevSpeed = speed;
  speed =stepTurningSpeed; // logic seems wierd, but we only use the values from the step turning
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= stepDrivingTime){ // edit the time-constants
    goStraight(dir, 1.0);
  }
  stepTurnTimestamp = millis();
  while(millis() - stepTurnTimestamp <= stepStoppingTime){ // edit the time-constants
    stop();
  }
  speed = prevSpeed;
}

void turnL()
{
  MotorB(1, 150);
  MotorA(-1, 90);
}
void turnR()
{
  MotorA(1, 150);
  MotorB(-1, 90);
}


// void Stop()
// {
//   MotorB(0, 0);
//   MotorA(0, 0);
// }


// ------------------------------------------- startup -------------------------------------------------
unsigned int cameraTurningTime = 300; // 400 when semil-low voltage, 300 with full batteries(11.5 voldts and up to 1.5 amps)
unsigned int cameraDetectionTime = 7000; // this is a little big, but has a "buffer" se we wont miss anything
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



// void DialInCup(){
//   if (digitalRead(Camera) == HIGH){ // everything within this one ? might not be needed/can lead to some mistakes?

//   } 
// }
// ---- case switches for the stages:

//int stage1Fin=0; // this didnt seem to work

const int maksSearchingStpes = 10; //the amount of steps we want to stop at going in one direction
const int cameraCenterDetectionTime= 3000; // find the "Lower Limit"

// code for getting the cup in the centre of the camera, ends standing in the correct direction
enum SearchingAlgorithm{startFindingCup, iterateCloser, minDist, steppingLeft, steppingRight, checkSenter};
SearchingAlgorithm current_iteration{startFindingCup};



void searchCupCentre(){
  unsigned long ITDrivingTime = 1500;
  int doneOneIT =0;
  int wasInCentre =0; // remember if it was correct
  //int inCentre = 0; // update more frequently //maybe not needed?
  int isClose =0;
  lastTime = millis();
  //while(digitalRead(Camera)== HIGH){ // keeps searching for as long as it takes( )
  switch(current_iteration){
    case (startFindingCup):
      while(digitalRead(Camera_center) == LOW){ // when it dosent see anny cups
        stepTurnR(); // we potentially need a puase here to allow camera to detect
        if(digitalRead(Camera_center) != LOW){ // checks if the object detected is/has been in the center due to camera-lag
          wasInCentre = 1;
        }
      }

      if(digitalRead(Camera_center) != LOW){
        //current_iteration = iterateCloser;
        stepStraight(1);
        break;
      }
      /*
      if((digitalRead(Camera_center)==LOW) && wasInCentre == 0){
        current_iteration = steppingRight;
        break;
      }
      if((digitalRead(Camera_center)== LOW) && wasInCentre ==1){
        current_iteration = steppingLeft;
        break;
      }
      */
    break;

    case (steppingRight):
      stepTurnR();
      lastTime = millis();
      if(digitalRead(Camera_center) != LOW){ // we might have to add a delay in here to allow for the camera to catch up, and have a slower stepping-function to call afterwards
        stop();
        lastTime = millis();
        current_iteration = iterateCloser;
        break;
      }

      //loosing sight of the cup
      if(digitalRead(Camera) == LOW){ //if it looses the cup in picture // steps left three times to get back to the lost 
        for(int i =0; i <=4; i++){ //steps four times to the Left (back)
          stepTurnL();
          lastTime = millis();
        }
        while(millis() - lastTime <=4000){ //waiting for camera to catch up.
          if(digitalRead(Camera) != LOW){
            current_iteration = steppingLeft;
            break;
          }
        }
      }
      if(digitalRead(IR_C) == HIGH && doneOneIT ==0){ // doneONEIT is here to stop it from getting wierd signals and just start spinning 
        turn180();
        current_iteration = startFindingCup; //back to start :) -----------This could be a problem with errors from the sensors
        break;
      }

    break;

    case (steppingLeft):
      stepTurnL();
      lastTime = millis();
      if(digitalRead(Camera_center) != LOW){ // we might have to add a delay in here to allow for the camera to catch up, and have a slower stepping-function to call afterwards
        stop();
        lastTime = millis();
        current_iteration = iterateCloser;
        break;
      }

      //loosing sight of the cup
      if(digitalRead(Camera) == LOW){ //if it looses the cup in picture // steps left three times to get back to the lost 
        for(int i =0; i <=4; i++){ //steps four times to the Left (back)
          stepTurnR();
          lastTime = millis();
        }
        while(millis() - lastTime <=4000){ //waiting for camera to catch up.
          if(digitalRead(Camera) != LOW){
            current_iteration = steppingRight;
            break;
          }
        }
      }
      if(digitalRead(IR_C == HIGH) && doneOneIT ==0){ // doneONEIT is here to stop it from getting wierd signals and just start spinning 
        current_iteration = startFindingCup; //back to start :) -----------This could be a problem with errors from the sensors
        break;
      }
    break;

    case (iterateCloser):
      doneOneIT ++;
      lastTime = millis();
      while(millis() - lastTime <= ITDrivingTime){ // the time here has to be adjusted, might make it a big number, then split it in two one or two times !!!!
        stepStraight(1);
        if(digitalRead(Camera) == LOW || digitalRead(CupDist) == LOW){ // cant see the cupp annymore (too close) OR the cup_dist sensor detects it
          stop();
          current_iteration = minDist; // not shure it this will break out completely, i thinkj it only will exit the loop
          isClose =1;
          break;
        }
      }
      if(isClose >=1){
        lastTime = millis();
        current_iteration = minDist;
        break;
      }
      ITDrivingTime = ITDrivingTime/1.5; // this might be stupid, but might work realy well also, the static value might have to be adjusted
      lastTime = millis();
      current_iteration = checkSenter;
    break;

    case (checkSenter):
      while(millis() - lastTime <= 500){ // waiting to check actual footage from camera
        //digitalRead(Camera_center); // does not matter
      }
      lastTime = millis();
      if(digitalRead(Camera_center) != LOW){
        lastTime = millis();
        current_iteration = iterateCloser;
        break;
      }


      if(digitalRead(Camera_center) == LOW && digitalRead(Camera) != LOW){ // checking if the cup is stil centered, and that we are not too close
        for(int j = 0; j <= 3; j++){ // step left 4 times
          stepTurnL();
        }
        lastTime = millis();
        while(millis() - lastTime <= 2000){// waiting for camera
        }

        if(digitalRead(Camera_center) == LOW && digitalRead(Camera) != LOW){
          lastTime = millis();
          current_iteration = steppingRight;
          break;
        }
        // else
        else{
          for(int j = 0; j <= 1; j++){
            stepTurnR();
          }
          break;// stay in checkSenter
        }
      }

      if(digitalRead(Camera_center) == LOW && digitalRead(Camera)== LOW){ // assumes that we are closer than we thought // might also have the cup...
        lastTime = millis();
        current_iteration = minDist;
        break;
      }

    break;

    case (minDist): // only to have a common exit-point 
      if(digitalRead(CupDist) == LOW){ // its close
        stop();
      }
      while(digitalRead(CupDist) == HIGH){
        stepStraight(1);
      }
      stop();
      lastTime = millis();
      current_fourthStage = Fetch;
    break;
  }
}






void Stage1(){
  switch(current_firstStage){
    case (Straight):
      lastTime = millis();
      //tartingSpeed(lastTime, 80);
      speed = 90;
      simpleFollowLine();
      if (ifT()){
        stop();
        current_firstStage = TSection;
        lastTime = millis();
        while(millis()- lastTime <= 400){
          stop();
        }
        lastTime = millis();
        break;
      }
      break;

    case(TSection):

      if(cupSide == -1){
        turn90L();
      }
      else if(cupSide == 1){
        turn90R();
      } 
      // potential problem if there for some reason is another value than 1 or -1 .......
      stop();

			current_firstStage = Go_To_Cup;
			lastTime = millis();
      break;

    case (Go_To_Cup):
      startingSpeed(lastTime, 80);
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
      //------------- test and choose between two 90-degrees or one 180;- last test used two 90-degrees
      turn90L(); // first 90, might have to make some changes due to cup-weight, perhaps going back a little or increasing the turning-speed
      // add small "go back" part to counteract the small forward drive? its intended to just use the momenum, so it might not matter ----------------------
      turn90L();
      //turn180();

			stop();
			lastTime = millis();
			current_firstStage = Drive_state1;
		break;
    
    case (Drive_state1): // make this a searchin algorithm??
      simpleFollowLine();
      if(millis() - lastTime >= 3000){ // time delay to allow it to get back on track
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
      }
      current_firstStage = getOut;
      lastTime = millis();
    break;

    case (getOut):
        //startingSpeed((lastTime), 80);
        while(millis() - lastTime <= 200){ // denne var litt feil  :))
          goStraight(-1,1.0);
        }
			  goStraight(-1, 1.0); // whatch this closely now 
        lineDetection();
        
			  if(ifT()){ // if T_section detected)
          lastTime = millis();
          while(millis() - lastTime <= 1000){
            stop();
          }
          //stop(); // might have to correct the placement there----------
          current_firstStage = TSection2;
          lastTime= millis();
          break;
        }
			
      break;

    case (TSection2):
    while(millis() - lastTime <=300){ //completely stopping after detecting T-section
      stop();
    }

    if (cupSide == -1)
    {
      while(millis() - lastTime <= 1000)
      {
        turnL();
      }
      stop();
      Grab();
      current_firstStage = Go_Next_state;
      lastTime = millis();
      break;
    }

    if (cupSide == 1)
    {
      while(millis() - lastTime <= 1000)
      {
        turnR();
      }
      stop();
      Grab();
      current_firstStage = Go_Next_state;
      lastTime = millis();
      break;
    }

    /*
    lastTime = millis();

    while(millis() - lastTime <=200){ //going a little forward to get beyond the T-section ---- might cause problems with knocking over the cup..... adjust value
      simpleFollowLine();
    }
      if(cupSide == -1){ 
        turn90L();
      }
      if(cupSide == 1){
        turn90R();
      }
      Grab(); // closing the grabber to get it out of the way
      current_firstStage = Go_Next_state;
      lastTime = millis();
      break;
      */
    case (Go_Next_state):

     // if(millis()- lastTime <=400){
     // goStraight(1,1.0);
     // }
     // else{ // no new time-stamp this time, might be nice to have tho

        simpleFollowLine();
        if(millis() - lastTime >= 2000){ // simply timing when stage 2 begins
          current_stage = SECOND;
          //stage1Fin = 1; // aditional atempt at getting out of stage1, dosent seem to work
          lastTime = millis();
          break;
        }
      //}
      break;

    default: //this might break the code, or fix it, Might be only used at the beginning if nothing else is defined...
      current_stage = SECOND;
      break;

    }
}



//void Stage2(){
//  speed= 150; // get this to as much as possible, maybe adjust for entering and exiting the stage?
//  simpleFollowLine();
//}

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
    speed = 170; // testing this value // it seems to work
    // add different turning??--------------------------------
    simpleFollowLine();
    if(millis()- lastTime >= 3500){ // increasing speed in the "end" of stage 3
      speed = 200;
      turnRate = 1.3; // this is to get past the second beam and turn afterwards, it might be bad for the sand-pit //seems to work
    }
    if(lineDetectArray[2] ==1){
      current_thirdStage = T2;
      break;
    }
  break;
  
  case (T2):
    turnRate= 1.1; // standard turnRate
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

unsigned long LastT_Time =0; // starting with a high value to 
const int GetPastT_Time = 200;
int CupCounter = 0;
int TCounter = 0;
int celebCounter = 0;
// code for stage 4
void stage4(){
  switch (current_fourthStage){
    case (GoToleft): //should work both times
      lastTime = millis();
      Release(); // making shure the grabber is ready, will be called 
      while(millis() - lastTime <= 300) 
      {} //simply waiting for arm to open up
      while(TCounter < 3){
        simpleFollowLine();
        lastTime = millis();
        if(ifT() && (millis() - LastT_Time >= GetPastT_Time+ 400)){  // keeps it from reading the same T-section as more than one // might have to be recoded !!!!!!!
          TCounter ++;
          while(millis() - lastTime <=400){
            stop();
          }
        }
      }
      lastTime= millis();

      // // turn90L(); // go to left side when at the final T-section
      // turn90L();
       while(millis() - lastTime <= 800)
      {
        turnL();
      }
      while(millis() - lastTime <= 1000){
        stop();
      }
      lastTime = millis();
      while(millis() - lastTime <= 50){ // time to get x amount to the left side of the "pit"
        simpleFollowLine();
      }
      while(millis() - lastTime <= 3050){
        stop();
      }
      lastTime = millis();
      while (millis()- lastTime <= 500){ // turning about 90 degrees to the right // scetchy due to only using timing!!
       turnInPlaceR();
      }
      lastTime = millis();
      while(millis() - lastTime <= 100){ // traveling a little bit forward to be able to search the whole area without hitting the Line again
        goStraight(1, 1.0);
      }
      stop();
      lastTime = millis();
      current_fourthStage = Search;
    break;

//----------------------
    case(Search):
      lastTime = millis();
      searchCupCentre(); // Now we are right in front of the cup
      //sends us to Ftch;
    break;
// ----------------------
    case (Fetch):
      lastTime = millis();
      Grab();
      while(millis() - lastTime <= 400)
      {} // simply wait for the grabber
      lastTime = millis();
      speed = 90; // this might have to be adjusted
      while(digitalRead(IR_C)!= HIGH){ // waiting for it to detect
        goStraight(-1, 1.0);
        //stepStraight(-1); // CHOOSE ONE OF THESE!!!!!!!!!
      }
      lastTime = millis();
      while(millis() - lastTime <= 200){
        stop();
      }
      while(digitalRead(IR_C)!= HIGH){
        stepStraight(1); 
      }
      stop();
      lastTime = millis();
      while(millis() - lastTime <= 500){ // assuming we are on a angle to the rigth based on our re-entery on the line:  // maybe adjust the timing
        simpleFollowLine(); // follow it for 0.5 sec to re-allign the bot
      }
      if(CupCounter == 0){
        lastTime = millis();
        current_fourthStage = FindFirstCupPlacement;
        break;
      }
      if(CupCounter >= 1){
        lastTime = millis();
        current_fourthStage = FindSecondCupPlacement;
        break;
      }
    break;

    case (FindFirstCupPlacement):
      while(TCounter == 3){
        simpleFollowLine();
        if(ifT()){
          turn90R();
          TCounter-- ; // decrementing (TCounter -= 1 )
          //break; // this is actually extra
        }
        lastTime = millis();
      }
      while(millis() - lastTime <= 800){ //added time to get past T-section
        simpleFollowLine();
      }
      lastTime = millis();
      while(TCounter == 2){ // this might be a little "brittle" compared to the rest of the code
        simpleFollowLine();
        if(ifT()){
          TCounter --; //keeping track of where we are for later use
          turn90L(); // turn into the correct T-section
        }
      }
      lastTime = millis();
      while(millis() - lastTime <= 200){
        stop();
      }
      lastTime = millis();
      while(! ifT()){ // this should work, but might be wierd
        simpleFollowLine();
      }
      stop();
      lastTime = millis();
      current_fourthStage = PutDown;
    break;


    case (PutDown):
       while(millis() - lastTime <= 500)
      {
        Lower();
      }
      while (millis() - lastTime <= 1000)
      {
        Release();
      }
      lastTime = millis();
      CupCounter ++; //this one is really important!!!

      while(millis()- lastTime <= 3000){ // has a max time of 3 seconds to go back
        while(millis() - lastTime <= 200){ // letting it have a little time before it tries to detect the T-section again
          goStraight(-1, 1.0);
        }
        goStraight(-1, 1.0);
        lineDetection(); // this should not be required anymore, but better safe than sorry
        if(ifT()){
          stop();
          lastTime = millis();
          TCounter ++;
          break;
        }
      }
      if(CupCounter >= 2){ // checking if we are finished!!
        lastTime = millis();
        current_fourthStage = End;
        break;
      }

      lastTime = millis();
      while(millis() - lastTime <= 500){
      stop();
      }
      lastTime = millis();

      while(millis() - lastTime <= 500) // dont like that there is only timing tho
      {
        turnL();
        if(digitalRead(IR_R)== LOW){ //detects
          break;
        }
      }
      stop();
      Grab();
      lastTime = millis();
      current_fourthStage = GoToleft;
    break;

//-------

    case (FindSecondCupPlacement):
      while(TCounter == 3){
        simpleFollowLine();
        if(ifT()){
          turn90R();
          TCounter-- ; // decrementing (TCounter -= 1 )
          //break; // this is actually extra
        }
        lastTime = millis();
      }
      while(millis() - lastTime <= 800){ //added time to get past T-section
        simpleFollowLine();
      }
      lastTime = millis();
      while(TCounter == 2){ // this might be a little "brittle" compared to the rest of the code
        simpleFollowLine();
        if(ifT()){ 
          TCounter --; // should only be possib le to call this once //keeping track of where we are fo
        }
      }
      // goes to the next T
      while(TCounter == 1){ // this might be a little "brittle" compared to the rest of the code
        simpleFollowLine();
        if(ifT()){
          TCounter --; //keeping track of where we are
          turn90L(); // turn into the correct T-section
        }
      }
      lastTime = millis();
      while(millis() - lastTime <= 200){
        stop();
      }
      lastTime = millis();
      while(! ifT()){ // this should work, but might be wierd
        simpleFollowLine();
      }
      stop();
      lastTime = millis();
      current_fourthStage = PutDown;
    break;

    // finitos
    case (End): //que celebrations!!
    turningSpeed = 200;
    turnInPlaceL();
    if(millis() - lastTime <=  400){
      Grab();
    }
    if(millis()- lastTime >=400){
      Release();
    }
    if(millis() - lastTime >= 800){
      lastTime = millis();
      celebCounter ++;
    }

    if(celebCounter >= 4){ // infinite loop for after we get an A :)
      Grab();
      while(true){
        stop();
      }
    }
    break;
  }

}

// ----------------------------|||||||||||||| main loop |||||||||||||------------------------

// int a =0; // used for testing

void loop(){
  current_stage = FOURTH;
  // while(true){

  // if(digitalRead(Camera_center) != LOW){
  //   turnInPlaceL();
  // }
  // if(digitalRead(Camera_center) == LOW){
  //   stop();
  // }
  // }
  // while(true){
  //   speed = 80;
  //   simpleFollowLine();
  //   lastTime = millis();
  //   if(ifT()){
  //       while(millis()- lastTime <=2000){
  //     stop();
  //     }
  //     while(millis()-lastTime <= 3000){
  //       simpleFollowLine();
  //     }
  //   }

  // }
  
  switch(current_stage){

    case (INIT): // only runs this one time
      //delay(2000); // This is now in the Setup-part
      speed = 80; // shouldnt matter
      //cupSide = -1; //only for testing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      startup();
      lastTime = millis(); //timestamp
      current_stage = FIRST;
      break;

    case (FIRST):


      Stage1();
      //if(stage1Fin==1){ 
      //  current_stage=SECOND;
      //}
      break;
    
    case(SECOND):
      //Stage2();
      speed= 150; // get this to as much as possible, maybe adjust for entering and exiting the stage?
      // WE MIGHT WANT TO ADD SHARPER TURNING AFTER A CERTAIN AMOUNT OF TIME HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!------------
      while(millis()-lastTime <= 4000){
        simpleFollowLine();
      }
      simpleFollowLine();
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
      stage4();
      break;
  }
}