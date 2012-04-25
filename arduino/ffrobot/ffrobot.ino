//#############################################################################################
//Fire Fighting Robot Code - Drexel Freshman Design Project 2012 Group 5
//Michael Dornisch, Matthew Lieve, Jim Moran
//#############################################################################################


//#############################################################################################
//Initial Variables/Constants
//#############################################################################################


//Ultrasonic Variables
int frontUSpin=2; //pin assignment
int frontUSdist; //distance var in cm
int leftUSpin=3;//pin assignment
int leftUSdist;//distance var in cm
int rightUSpin=4;//pin assignment
int rightUSdist;//distance var in cm

int USdist;//distance var for measureDistance function
unsigned long pulseduration=0;//used in calculating distance


//#############################################################################################
//Setup
//#############################################################################################

void setup()
{

 Serial.begin(9600);
}

//#############################################################################################
//Measure Ultrasonic Distance
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
//Loop Code
//#############################################################################################
void loop()
{
 
 //The following should be a test, where it should print the number of cm the distance is for the leftUSpin ultrasonic sensor
 Serial.print("Distance - ");
 Serial.print(measureDistance(leftUSpin));
 Serial.println(" cm");
 delay(500);
}
