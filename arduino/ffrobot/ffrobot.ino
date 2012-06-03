//#############################################################################################
//Fire Fighting Robot Code - Drexel Freshman Design Project 2012 Group 5
//Michael Dornisch, Matthew Lieve, Jim Moran
//#############################################################################################
#include "Timer.h" //use custom timer function. see http://arduino.cc/playground/Code/Timer
//#############################################################################################
//Initial Variables/Constants
//#############################################################################################
int gameMode = 0;
int roomMode = 0;
//#####################################
//Reflectivity Sensor
int reflectPin =9;
int reflectVal;
//#####################################
//Fan
int fanPin = 10;


//#####################################
//Ultrasonic Variables
int frontUSpin=12; //pin assignment
int frontUSdist; //distance var in cm
int leftUSpin=13;//pin assignment
int leftUSdist;//distance var in cm
int rightUSpin=4;//pin assignment
int rightUSdist;//distance var in cm
int USdist;//distance var for measureDistance function
unsigned long pulseduration=0;//used in calculating distance
//#####################################
//Multiplexer/flamesensor variables
int multiPin=A2; //Analog in pin to be multiplexed
int selectApin=0; //the following 3 pins are used for selecting the input by driving high/low
int selectBpin=1;
int selectCpin=2;

int flameFLpin=0; //abstract front left sensor pin
int flameFCpin=1; //abstract front center sensor pin
int flameFRpin=2; //abstract front right sensor pin
int flameBRpin=3; //abstract back right sensor pin
int flameBCpin=4; //abstract back center sensor pin
int flameBLpin=5; //abstract back left sensor pin
int flameval; //general variable used in function before writing to final var
int flameFLval; //These are the cooresponding values written to with sensor data
int flameFCval;
int flameFRval;
int flameBRval;
int flameBCval;
int flameBLval;
int fireThreshold = 500; //This is the threshold for something seen to be considered a fire
//#####################################
//Gyro vars
Timer t;
int gyroPin = 4;//Gyro is connected to analog pin 0
float gyroZeroVoltage = 2.43;   //Gyro is zeroed at 2.5V
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = 1;   //Minimum deg/sec to keep track of - helps with gyro drifting
float gyroVoltage = 5.00;
float currentAngle = 0;   //Keep track of our current angle
int roundedAngle;
//#####################################
//Encoder Vars
int avgCount;
int encoderPinL=A0;
int encoderPinR=A1;
int currentEncoderL=0;
int currentEncoderR=0;
int prevEncoderL=0;
int prevEncoderR=0;
int encoderCountL=0;
int encoderCountR=0;
float distanceTraveled;
String Left, Right;
//#####################################
//movement Vars
int leftSpeedPin=5, rightSpeedPin=6;
int leftDirPin=7, rightDirPin=8;
int leftSpeed, rightSpeed, leftDir, rightDir;
String CW, CCW, FWD, REV, STOP, setdir;














//#############################################################################################
//Setup
//#############################################################################################
void setup()
{

  Serial.begin(9600); //Setup serial for debugging
  pinMode(5, OUTPUT); //Setup all digital pins that are permanant I/O
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(10, HIGH); //Turn fan off. it defaults to on when LOW
  t.every(10, readHeading); //custom timer, every 10ms, readHeading();
}



//#############################################################################################
//Measure Ultrasonic Distance Function
//#############################################################################################
int measureDistance(int USside) //I want to be able to put measureDistance(frontUSpin) or measureDistance(leftUSpin) or measureDistance(rightUSpin)
//and have it give me the distance by replacing the var USside with one of the 3 vars I put in
{

  // set pin as output so we can send a pulse
  pinMode(USside, OUTPUT);
  // set output to LOW
  digitalWrite(USside, LOW);
  delayMicroseconds(5);

  // now send the 5uS pulse out to activate Ping)))
  digitalWrite(USside, HIGH);
  delayMicroseconds(5);
  digitalWrite(USside, LOW);

  // now we need to change the digital pin
  // to input to read the incoming pulse
  pinMode(USside, INPUT);

  // finally, measure the length of the incoming pulse
  pulseduration=pulseIn(USside, HIGH);
  // divide the pulse length by half
  pulseduration=pulseduration/2; 

  // now convert to centimetres. We're metric here people...
  USdist = int(pulseduration/29);
  return USdist;
}

//#############################################################################################
//Reflector Sensor
//#############################################################################################
long reflectSensor(){
  long result = 0;
  pinMode(reflectPin, OUTPUT);       // make pin OUTPUT
  digitalWrite(reflectPin, HIGH);    // make pin HIGH to discharge capacitor - study the schematic
  delay(1);                       // wait a  ms to make sure cap is discharged

  pinMode(reflectPin, INPUT);        // turn pin into an input and time till pin goes low
  digitalWrite(reflectPin, LOW);     // turn pullups off - or it won't work
  while(digitalRead(reflectPin)){    // wait for pin to go low
    result++;
  }

  return result;                   // report results   
}   

//#############################################################################################
//Flame Sensor Code / Multiplexer Function
//#############################################################################################
/*Please read connections.xslx for more information on the 4051 multiplexer used. 
 This allows for 1 analog pin and 3 digital pins to be used in order to switch between up to 8 sensors. 
 this code will activate the right switches, then read the sensor, allowing it to be written to the proper variable.
 Inside the spreadsheet is the binary conversion table.*/

int senseFlame( int flamepin)
{
  if (flamepin == flameFLpin) { //If the front left pin is selected
    digitalWrite(selectApin,LOW); //set binary to 000
    digitalWrite(selectBpin,LOW);
    digitalWrite(selectCpin,LOW);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }
  else if (flamepin == flameFCpin) { //If the front center pin is selected
    digitalWrite(selectApin,HIGH); //set binary to 001
    digitalWrite(selectBpin,LOW);
    digitalWrite(selectCpin,LOW);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }
  else if (flamepin == flameFRpin) { //If the front right pin is selected
    digitalWrite(selectApin,LOW); //set binary to 010
    digitalWrite(selectBpin,HIGH);
    digitalWrite(selectCpin,LOW);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }
  else if (flamepin == flameBRpin) { //If the back right pin is selected
    digitalWrite(selectApin,HIGH); //set binary to 011
    digitalWrite(selectBpin,HIGH);
    digitalWrite(selectCpin,LOW);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }
  else if (flamepin == flameBCpin) { //If the back center pin is selected
    digitalWrite(selectApin,LOW); //set binary to 100
    digitalWrite(selectBpin,LOW);
    digitalWrite(selectCpin,HIGH);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }
  else if (flamepin == flameBLpin) { //If the back left pin is selected
    digitalWrite(selectApin,HIGH); //set binary to 101
    digitalWrite(selectBpin,LOW);
    digitalWrite(selectCpin,HIGH);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }

  return flameval;
}


//#############################################################################################
//Basic Drive Function
//#############################################################################################
void drive( String dir, int howfast) //call with drive(FWD/REV/CW/CCW, 0-255) for direction and speed
{
  if(dir == CW)
  {
    digitalWrite(leftDirPin, LOW);
    digitalWrite(rightDirPin, HIGH);
    analogWrite(leftSpeedPin, howfast);
    analogWrite(rightSpeedPin, howfast);
    gyroVoltage = 5.01; //this compensates for voltage drop to stop gyro drift
  }
  if(dir == CCW)
  {
    digitalWrite(leftDirPin, HIGH);
    digitalWrite(rightDirPin, LOW);
    analogWrite(leftSpeedPin, howfast);
    analogWrite(rightSpeedPin, howfast);
    gyroVoltage = 5.01;
  }
  if(dir == FWD)
  {
    digitalWrite(leftDirPin, LOW);
    digitalWrite(rightDirPin, LOW);
    analogWrite(leftSpeedPin, howfast);
    analogWrite(rightSpeedPin, howfast);
    gyroVoltage = 5.01;
  }
  if(dir == REV)
  {
    digitalWrite(leftDirPin, LOW);
    digitalWrite(rightDirPin, HIGH);
    analogWrite(leftSpeedPin, howfast);
    analogWrite(rightSpeedPin, howfast);
    gyroVoltage = 5.01;
  }
  if(dir == STOP)
  {
    digitalWrite(leftDirPin, LOW);
    digitalWrite(rightDirPin, LOW);
    analogWrite(leftSpeedPin, 0);
    analogWrite(rightSpeedPin, 0);
    gyroVoltage = 5.00;
  }
  return;
}

//#############################################################################################
//Advanced Drive Functions
//#############################################################################################
//This allows for turning to a specific angle
void turnAngle(String LR, int heading) //input= CW/CCW, 0-360 degrees
{
  drive(LR, 255);

  while(roundedAngle != heading)
  {
    t.update();
  }
  drive(STOP, 0);
  delay(250);
  return;
}
//this allows for driving a specific distance, in cm.
//diamater is 37.5mm / 3.75 cm. There are 16 partitions on the encoder wheel(8 white/black), for 22.5 degrees per tick
//this breaks down to .735cm per tick
void driveEnc(String FR, int dist)
{
  encoderCountL=0;
  encoderCountR=0;
  distanceTraveled=0;
  drive(FR, 255);  
  while(distanceTraveled < dist)
  {
    checkSensors();
    t.update();
    leftEncoder();
    rightEncoder();
    distanceTraveled = ((encoderCountL + encoderCountR)/2 * 0.736); //This gets the average of the two encoders, and multiplies into distance(cm)
    if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
    {//if a fire is seen, stop and break from this immediatley.
      gameMode = 1;
      delay(250);
      drive(STOP, 0);
      delay(250);
      return;
    }
  }
  drive(STOP, 0);
  delay(250);
  return;
}




//#############################################################################################
//Encoder code
//#############################################################################################

void leftEncoder()
{
  int raw;
  prevEncoderL=currentEncoderL; //Previous current value becomes current previous value
  raw = analogRead(encoderPinL);
  if(raw < 600) //check if encoder is on white
  {
    currentEncoderL = 1; //if so, make 1
  }
  else
  {
    currentEncoderL = 0; //if not, make 0
  }
  if(currentEncoderL != prevEncoderL) //if the previous and current arent the same (AKA transition)
  {
    encoderCountL ++; //add one to the count
  }
  return;
}

void rightEncoder()
{
  int raw;
  prevEncoderR=currentEncoderR; //Previous current value becomes current previous value
  raw = analogRead(encoderPinR);
  if(raw < 600) //check if encoder is on white
  {
    currentEncoderR = 1; //if so, make 1
  }
  else
  {
    currentEncoderR = 0; //if not, make 0
  }
  if(currentEncoderR != prevEncoderR)
  {
    encoderCountR ++;
  }
  return;
}



//#############################################################################################
//Gyro Code
//#############################################################################################

void readHeading() {
  float gyroRate = ((analogRead(gyroPin) * gyroVoltage) / 1023);//- 5.0 + mappedVoltage;

  //This line finds the voltage offset from sitting still
  gyroRate -= gyroZeroVoltage;

  //This line divides the voltage we found by the gyro's sensitivity
  gyroRate /= gyroSensitivity;

  //Ignore the gyro if our angular velocity does not meet our threshold
  if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) 
  {
    //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
    gyroRate /= 100 / 3.6;
    currentAngle += gyroRate;
  }

  //Keep our angle between 0-359 degrees
  if (currentAngle < 0)
    currentAngle += 360;
  else if (currentAngle > 359)
    currentAngle -= 360;
  roundedAngle = 360 - currentAngle;
  return;
}


void checkSensors()
{
  leftEncoder(); //get encoder ticks
  rightEncoder();
  flameFLval = senseFlame(flameFLpin); //check all flame sensors
  flameFCval = senseFlame(flameFCpin);
  flameFRval = senseFlame(flameFRpin);
  flameBLval = senseFlame(flameBLpin);
  flameBCval = senseFlame(flameBCpin);
  flameBRval = senseFlame(flameBRpin);
  //readHeading(); // check gyro
  frontUSdist = measureDistance(frontUSpin); // check all US sensors
  leftUSdist = measureDistance(leftUSpin);
  rightUSdist = measureDistance(rightUSpin);
  reflectVal = reflectSensor(); //check reflectivity sensor
  return;
}






//#############################################################################################
//Loop Code
//#############################################################################################
void loop() {
  switch(gameMode) {
  case 0:
    {
      if(roomMode==0)
      {
        //intelligent navigation
        //This is how to search each room.
        //0 = first hallway
        //1 = 
        //2 =
        //3 =
        t.update(); //update timer function
        checkSensors(); //update all sensors
        drive(FWD, 255);
        Serial.println("Start");
        while(roomMode ==0 && frontUSdist > 10) //before reaching the wall
        {
          t.update();
          checkSensors(); //update all sensors
          Serial.println(roundedAngle);
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
          if(roundedAngle >= 4 && roundedAngle < 180) //if veering clockwise
          {
            digitalWrite(leftDirPin, LOW);
            digitalWrite(rightDirPin, LOW);
            analogWrite(leftSpeedPin, 200); //slow down left side to compensate
            analogWrite(rightSpeedPin, 255);

          }
          else if(roundedAngle <= 360-4 && roundedAngle > 180) //if veering c-clockwise
          {
            digitalWrite(leftDirPin, LOW);
            digitalWrite(rightDirPin, LOW);
            analogWrite(leftSpeedPin, 255); //slow down right side to compensate
            analogWrite(rightSpeedPin, 200);
          }
          else //if on track
          {
            drive(FWD, 255);
          }

        }
      }
      drive(STOP, 0);
      roomMode=1;
      Serial.print(roomMode);
      //########################################################################################################################### Room 1
      if(roomMode==1) // turn towards top right room, drive into and out of it
      {
        turnAngle(CW,90); //turn 90 degrees towards first room

        if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
        {//if a fire is seen, stop and break from this immediatley.
          gameMode = 1;
          delay(250);
          drive(STOP, 0);
          delay(250);
          break;
        }

        driveEnc(FWD, 50);//drive 50 cm into room

        if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
        {//if a fire is seen, stop and break from this immediatley.
          gameMode = 1;
          delay(250);
          drive(STOP, 0);
          delay(250);
          break;
        }
        driveEnc(REV, 50);

        if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
        {//if a fire is seen, stop and break from this immediatley.
          gameMode = 1;
          delay(250);
          drive(STOP, 0);
          delay(250);
          break;
        }
        drive(STOP, 0);
        roomMode = 2;
      }//end roomMode1
      //########################################################################################################################### Room 2

        if(roomMode == 2) //turn to go back down hallway, travel along west wall of room 2, turn towards opening, drive in and out.
      {
        turnAngle(CW, 180); //turn down hallway

        if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
        {//if a fire is seen, stop and break from this immediatley.
          gameMode = 1;
          delay(250);
          drive(STOP, 0);
          delay(250);
          break;
        }
        drive(FWD, 255); 
        checkSensors();
        while(rightUSdist < 30) //while driving along the west wall of room 2
        {
          t.update();
          checkSensors();
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
        }
        driveEnc(FWD, 20); //drive south a little bit further, ultrasonic sensor is not on back of this 
        if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
        {//if a fire is seen, stop and break from this immediatley.
          gameMode = 1;
          delay(250);
          drive(STOP, 0);
          delay(250);
          break;
        }
        turnAngle(CW,270); //turn to enter room2
        driveEnc(FWD,60);
        if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
        {//if a fire is seen, stop and break from this immediatley.
          gameMode = 1;
          delay(250);
          drive(STOP, 0);
          delay(250);
          break;
        }
        driveEnc(REV,60);
        if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
        {//if a fire is seen, stop and break from this immediatley.
          gameMode = 1;
          delay(250);
          drive(STOP, 0);
          delay(250);
          break;
        }
        drive(STOP,0);
        roomMode=3;
      }//end roomMode2
      //########################################################################################################################### Room 3
      if(roomMode==3)
      {
        turnAngle(CCW,180); //turn south in hallway
        drive(FWD, 255);
        checkSensors();
        while(frontUSdist <= 118) //drive south until in 4-way
        {
          t.update();
          checkSensors();
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
        }
        
        turnAngle(CW,270); //turn towards room3
        drive(FWD,255);
        checkSensors();
        while(leftUSdist > 100 ) //drive until near that room opening
        {
          t.update();
          checkSensors();
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
        }
driveEnc(FWD,20); //go a little farther to be sure.
if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
          drive(REV, 255);
          while(leftUSdist < 80)//drive out to 4 way again
          {
            t.update();
          checkSensors();
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
      }
      driveEnc(REV, 10); //just a little more
      if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
          drive(STOP,0);
        roomMode=4;
      }//end room3
       //########################################################################################################################### Room 4
       if(roomMode=4)
       {
         checkSensors();
         turnAngle(CCW,180);//turn south
         if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
          drive(FWD,255);
          while(leftUSdist < 80)//move south until by hallway for room 4
          {
            t.update();
          checkSensors();
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
          }
          driveEnc(FWD,10);
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
          turnAngle(CCW,90);
          drive(FWD,255);
          checkSensors();
           while(frontUSdist > 10)//move east towards the room 4 doorway
          {
            t.update();
          checkSensors();
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
          }
          turnAngle(CW,180);
          driveEnc(FWD,40);
          if(flameFLval >= fireThreshold || flameFCval >= fireThreshold || flameFRval >= fireThreshold || flameBLval >= fireThreshold || flameBCval >= fireThreshold || flameBRval >= fireThreshold)
          {//if a fire is seen, stop and break from this immediatley.
            gameMode = 1;
            delay(250);
            drive(STOP, 0);
            delay(250);
            break;
          }
          drive(STOP,0);
          checkSensors;
          roomMode=5;
       }//end room4
 //########################################################################################################################### Room 5
if(roomMode=5)
gameMode=2;
break;
    } //end case 0


  case 1:
    // Fire is detected, closing in!
    break;
  case 2:
    // something must've went wrong, brute force method
    break;
  case 3: 
    // time to go home
    break;
  default:
    //Found home! yay!
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
    break;
  }
}










