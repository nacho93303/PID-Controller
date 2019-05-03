#include "TimerOne.h"
#include <PID_v1.h>

int counter=0;
//Polarity
const int IN1 = 11;
const int IN2 = 10;

volatile float pot=0;

//PWM Signal and Analog from the Potentiometer
const int POT = 0;
const int ENA = 6;
const int SET = 12;

bool PIDset = false;

float savedrotation, currentrotation, last;

double Setpoint, Input, Output;
double Kp=1.2, Ki=8.5, Kd=2.5;


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void docount()  // counts from the speed sensor
{
  counter++;  // increase +1 the counter value
} 

//This takes the docout() counter and calculates how many holes is read to give rotations per second
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  currentrotation = counter / 31*60/(millis()-last);  // divide by number of holes in Disc
  Serial.print(currentrotation,DEC); 
  
  if ( digitalRead(SET) == HIGH && PIDset == false)
  {
    savedrotation = currentrotation;
    PIDset = true;
  }
  else if (digitalRead(SET) == LOW)
  {
    PIDset = false;
  }
   
  //Serial.println(" Rotation per second"); 
  counter=0;  //  reset counter to zero
  last = millis();
  
  Timer1.attachInterrupt( timerIsr );  //enable the timer
  Serial.print(", ");
  Serial.print(savedrotation);
  
  Serial.print("\n");

  //Serial.print(Input);
  //Serial.print("\n");

}

void setup() 
{
  Serial.begin(9600);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  pinMode(SET, INPUT);
  Timer1.initialize(500000); // set timer for 90ms
  attachInterrupt(0, docount, CHANGE);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer

  myPID.SetMode(AUTOMATIC);
} 

void loop()
{
  // This bit is how you convert the pot values to manually control the fan
  pot=analogRead(POT)/4.01569;
  analogWrite(ENA, pot);
  
  // This basically acts as a push button. If the SET pin is HIGH it initiates the PID and whatever
  // the current rotation and pot setting are, they're used to send to the myPID function above to 
  // maintain current operating values
  while (digitalRead(SET) == HIGH)
  {
      Setpoint = savedrotation;
      Input = currentrotation;
      myPID.Compute();
      analogWrite(ENA, Output);
      digitalWrite(10, HIGH);  // set rotation of motor to Clockwise
      digitalWrite(11, LOW);
      
  }
  digitalWrite(10, HIGH);  // set rotation of motor to Clockwise
  digitalWrite(11, LOW);


  
  delay (25);
}
