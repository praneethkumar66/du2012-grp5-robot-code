//#############################################################################################
//Fire Fighting Robot Code - Drexel Freshman Design Project 2012 Group 5
//Michael Dornisch, Matthew Lieve, Jim Moran
//#############################################################################################


//#############################################################################################
//Initial Variables/Constants
//#############################################################################################
int CW, CCW, FWD, REV, setdir;
//#####################################
//Reflectivity Sensor
int reflectPin =A3;
int reflectVal;
//#####################################
//Ultrasonic Variables
int frontUSpin=2; //pin assignment
int frontUSdist; //distance var in cm
int leftUSpin=3;//pin assignment
int leftUSdist;//distance var in cm
int rightUSpin=4;//pin assignment
int rightUSdist;//distance var in cm
int USdist;//distance var for measureDistance function
unsigned long pulseduration=0;//used in calculating distance
//#####################################
//Multiplexer/flamesensor variables
int multiPin=A2; //Analog in pin to be multiplexed
int selectApin=11; //the following 3 pins are used for selecting the input by driving high/low
int selectBpin=12;
int selectCpin=13;

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
float gyroZeroVoltage = 2.423;   //Gyro is zeroed at 2.423V
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = 1;   //Minimum deg/sec to keep track of - helps with gyro drifting
float currentAngle = 0;          //Keep track of our current angle
int roundedAngle; //rounded angle for whole number use
float gyroRate;
//#####################################
//Encoder Vars
 int avgCount;
 int encoderPinL=A0;
 int encoderPinR=A1;
 int sensorcount1L=0;
 int sensorcount1R=0;
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

 Serial.begin(9600);
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
    digitalWrite(selectApin,LOW); //set binary to 001
    digitalWrite(selectBpin,LOW);
    digitalWrite(selectCpin,HIGH);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }
    else if (flamepin == flameFRpin) { //If the front right pin is selected
    digitalWrite(selectApin,LOW); //set binary to 010
    digitalWrite(selectBpin,HIGH);
    digitalWrite(selectCpin,LOW);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }
  else if (flamepin == flameBRpin) { //If the back right pin is selected
    digitalWrite(selectApin,LOW); //set binary to 011
    digitalWrite(selectBpin,HIGH);
    digitalWrite(selectCpin,HIGH);
    flameval = analogRead(multiPin); //Read the pin through the multiplexer, set to flame val
  }
else if (flamepin == flameBCpin) { //If the back center pin is selected
    digitalWrite(selectApin,HIGH); //set binary to 100
    digitalWrite(selectBpin,LOW);
    digitalWrite(selectCpin,LOW);
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
//Reflectivity Sensor Function
//############################################################################################# 
//  int senseReflection(int pinval)
//  {
//    reflectVal = analogRead(pinval);
//    return reflectVal;
//  }











//#############################################################################################
//Basic Drive Function
//#############################################################################################
void drive( int dir, int howfast) //call with drive(FWD/REV/CW/CCW, 0-255) for direction and speed
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
//Code for encoders
//#############################################################################################
int encodeDistance(int whichEncoder)
{
  int rawsensorValue=0;
int encoderVarSelect;
 int sensorcount0 = 0;
 if which
  rawsensorValue = analogRead(whichEncoder);

if(whichEncoder == A0)
{
  encoderVarSelect = sensorcount1L;
}
else
{
  encoderVarSelect = sensorcount1R;
}

if (rawsensorValue < 600){ //Min value is 400 and max value is 800, so state chance can be done at 600.
encoderVarSelect = 1;
}
else {
sensorcount1 = 0;
}
if (encoderVarSelect != sensorcount0){
count ++;
}








//#############################################################################################
//Go Distance at Heading Code
//#############################################################################################
int goplace( int radius, int angle)
{
  int originalHeading= roundedAngle; //makes current heading variable

  if(angle - originalHeading <=180) // if the destination is less than 180 degrees clockwise from the orginal angle
{
setdir = CW;
}
else if (angle - originalHeading >180) //this part makes it turn the shortest way possible
{
 setdir = CCW;
}

while(roundedAngle != angle) // while it isn't facing the right direction. 
{
  readHeading();
drive(setdir, 255); //spin the direction set above
}
drive(FWD, 0); //stop

}
  
   





















//#############################################################################################
//Gyro Code
//#############################################################################################

void readHeading()
{
gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1023; //This line converts the 0-1023 signal to 0-5V
  gyroRate -= gyroZeroVoltage;  //This line finds the voltage offset from sitting still
   gyroRate /= gyroSensitivity;   //This line divides the voltage we found by the gyro's sensitivity
   if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) { //Ignore the gyro if our angular velocity does not meet our threshold
    gyroRate /= 100; //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
    currentAngle += gyroRate;
   }
   if (currentAngle < 0) {  //Keep our angle between 0-359 degrees
    currentAngle += 360;
}
  else if (currentAngle > 359) {
    currentAngle -= 360;
}
   roundedAngle = currentAngle;
return;
}










































//#############################################################################################
//Loop Code
//#############################################################################################
void loop()
{
 //Here is some basic code for sensing 
 leftUSdist = measureDistance(leftUSpin);
 rightUSdist = measureDistance(rightUSpin);
 frontUSdist = measureDistance(frontUSpin);
 
 flameFLval = senseFlame(flameFLpin);
 flameFCval = senseFlame(flameFCpin);
 flameFRval = senseFlame(flameFRpin);
 flameBRval = senseFlame(flameBRpin);
 flameBCval = senseFlame(flameBCpin);
 flameBLval = senseFlame(flameBLpin);
 readHeading();
 //This code is used for keeping the gyroscope heading
  //round to whole number
 

}
