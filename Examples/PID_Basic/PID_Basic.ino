/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <arduino_template_pid.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PIDd myPID(Input, Output, Setpoint,2,5,1, PIDd::DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(0);
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(PIDd::AUTOMATIC);
}

void loop()
{
  Input = analogRead(0);
  myPID.Compute();
  analogWrite(3,Output);
}


