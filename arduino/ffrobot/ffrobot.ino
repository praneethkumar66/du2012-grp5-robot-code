//#############################################################################################
//Fire Fighting Robot Code - Drexel Freshman Design Project 2012 Group 5
//Michael Dornisch, Matthew Lieve, Jim Moran
//#############################################################################################
#include <avr/interrupt.h>  //include interrupts to use timer 2 overflow as internal interrrupt
#include <avr/io.h>
//#############################################################################################
//Initial Variables/Constants
//#############################################################################################
String CW, CCW, FWD, REV, setdir;
int gameMode = 0;
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
//#####################################
//Gyro vars
int gyroPin = A4;               //Gyro is connected to analog pin 0
float gyroVoltage = 5;         //Gyro is running at 5V
float gyroZeroVoltage = 2.425;   //Gyro is zeroed at 2.423V
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = 1;   //Minimum deg/sec to keep track of - helps with gyro drifting

volatile float currentAngle = 0;          //Keep track of our current angle
volatile int roundedAngle; //rounded angle for whole number use
volatile float gyroRate;
unsigned int count = 0;
volatile float ISRvalue;
volatile boolean canCheck = false;
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
int distanceTraveled;
String Left, Right;
//#####################################
//movement Vars
int leftSpeedPin=5, rightSpeedPin=6;
int leftDirPin=7, rightDirPin=8;
int leftSpeed, rightSpeed, leftDir, rightDir; 














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

  //Setup Timer2 to fire every 1ms
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
  TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128
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
  }
  if(dir == CCW)
  {
    digitalWrite(leftDirPin, HIGH);
    digitalWrite(rightDirPin, LOW);
    analogWrite(leftSpeedPin, howfast);
    analogWrite(rightSpeedPin, howfast);
  }
  if(dir == FWD)
  {
    digitalWrite(leftDirPin, LOW);
    digitalWrite(rightDirPin, LOW);
    analogWrite(leftSpeedPin, howfast);
    analogWrite(rightSpeedPin, howfast);
  }
  if(dir == REV)
  {
    digitalWrite(leftDirPin, LOW);
    digitalWrite(rightDirPin, HIGH);
    analogWrite(leftSpeedPin, howfast);
    analogWrite(rightSpeedPin, howfast);
  }

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
}

//#############################################################################################
//Interrupt Service Routine
//#############################################################################################
ISR(TIMER2_OVF_vect) { //Timer2 overflow interrupt vector, called every 1 ms
  count++;
  if(count > 99) { //how many ms to wait before doing this(0 based)
    // readHeading();  - don't use this, too much going on...
    ISRvalue = analogRead(gyroPin);
    count = 0; 
    canCheck = true; 
  }
  TCNT2 = 130; //set to 130 out of 255 (125 cycles)
  TIFR2 = 0x00; //restart timer
};

//#############################################################################################
//Gyro Code
//#############################################################################################

void readHeading()
{
  if(canCheck)
  {
    gyroRate = (ISRvalue * gyroVoltage) / 1023; //This line converts the 0-1023 signal to 0-5V
    gyroRate -= gyroZeroVoltage;  //This line finds the voltage offset from sitting still
    gyroRate /= gyroSensitivity;   //This line divides the voltage we found by the gyro's sensitivity
    if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) { //Ignore the gyro if our angular velocity does not meet our threshold
      gyroRate /= 10; //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
      currentAngle += gyroRate;
    }
    if (currentAngle < 0) {  //Keep our angle between 0-359 degrees
      currentAngle += 360;
    }
    else if (currentAngle > 359) {
      currentAngle -= 360;
    }
    roundedAngle = currentAngle;
    roundedAngle = 360 - roundedAngle;
    //Serial.println(" ");
    canCheck = false;
  }
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
  readHeading(); // check gyro
  frontUSdist = measureDistance(frontUSpin); // check all US sensors
  leftUSdist = measureDistance(leftUSpin);
  rightUSdist = measureDistance(rightUSpin);
  reflectVal = reflectSensor(); //check reflectivity sensor
  return;
}






//#############################################################################################
//Loop Code
//#############################################################################################
//void loop() {
//  switch(gameMode) {
//  case 0:
//    {
//      //intelligent navigation
//      int roomMode=0; //This is how to search each room.
//      //0 = first hallway
//      //1 = 
//      //2 =
//      //3 =
//
//      checkSensors(); //update all sensors
//      digitalWrite(leftDirPin, LOW);
//      digitalWrite(rightDirPin, LOW);
//      analogWrite(leftSpeedPin, 255); 
//      analogWrite(rightSpeedPin, 255);
//      Serial.println("Start");
//      while(roomMode ==0 && frontUSdist > 10) //before reaching the wall
//      {
//        checkSensors(); //update all sensors
//        Serial.println(roundedAngle);
//        if(roundedAngle >= 4 && roundedAngle < 180) //if veering clockwise
//        {
//          digitalWrite(leftDirPin, LOW);
//          digitalWrite(rightDirPin, LOW);
//          analogWrite(leftSpeedPin, 200); //slow down left side to compensate
//          analogWrite(rightSpeedPin, 255);
//         // Serial.println("compensate for clockwise");
//
//        }
//        else if(roundedAngle <= 360-4 && roundedAngle > 180) //if veering c-clockwise
//        {
//          digitalWrite(leftDirPin, LOW);
//          digitalWrite(rightDirPin, LOW);
//          analogWrite(leftSpeedPin, 255); //slow down right side to compensate
//          analogWrite(rightSpeedPin, 200);
//          //Serial.println("compensate for clockwise");
//        }
//        else //if on track
//        {
//          digitalWrite(leftDirPin, LOW);
//          digitalWrite(rightDirPin, LOW);
//          analogWrite(leftSpeedPin, 255); 
//          analogWrite(rightSpeedPin, 255);
//          //Serial.println("going straignt");
//        }
//
//      }
//    }
//    digitalWrite(leftDirPin, LOW);
//    digitalWrite(rightDirPin, LOW);
//    analogWrite(leftSpeedPin, 0); 
//    analogWrite(rightSpeedPin, 0);
//
//
//  case 1:
//    // Fire is detected, closing in!
//    break;
//  case 2:
//    // something must've went wrong, brute force method
//    break;
//  case 3: 
//    // time to go home
//    break;
//  default:
//    //Found home! yay!
//    digitalWrite(2, HIGH);
//    delay(100);
//    digitalWrite(2, LOW);
//    delay(100);
//    break;
//  }
//}
void loop()
{
  checkSensors();
  Serial.println(roundedAngle);
}


