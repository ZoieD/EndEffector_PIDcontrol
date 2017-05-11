/*
  Force Contorl using PID tuning algorithm to perform precise contorl compliance force actuator

  created 14 April 2017
  Last modified 21 April 2017 
  by Dou Zi

  This code is the on-board code for MEGA 1280 that
  acts as an controller to adjust PWM value output to
  motor in order to achieve fast reactive and stable control

  This code has four main functions :
  1. Two limited switch CW and CCW state operate reset and emergency to back to a fixed place
  2. Auto-control algorithm with the concept of PID Tuning 

  More can be found on:


  Coder in-charge: Dou Zi
  Contributor : Qiu Chen and Conghui
*/

/* header files */
#include <PID_v1.h> 
#include <PinChangeInterrupt.h> // include arduino hardware interrupt (pin change interrupt) library 
#include "TimerOne.h"
#include "TimerThree.h"

/* define the Input and output pinMode */
const byte hall_pin[] = {2, 3, 21}; // we use pin 2,3,21 for hall sensor Input
const byte LS_pin[] = {44, 46};
const byte dir_pin[] = {24, 26};
int PWM = 8;//speed control (IN1 in motor driver)

/* this array is the global variables needed to get and store PWM values */ 
volatile unsigned long hall_state[] = {0, 0, 0}; // 
volatile unsigned long pulse_length[] = {0, 0, 0}; //
volatile unsigned long rising_start[] = {0, 0, 0};
volatile unsigned long LS_state[] = {0, 0}; 

int reset_speed = 10;
int last_PWM = 0;
double sensor_value = 0;
double first_sensor_value = 0;
boolean start_check = false;
boolean pid_run = false;
boolean stop_check = false;


/* PID parameter set */
double Kp = 0.6; // Proportional gain constant, to observe whether response reaction speed is within our request
double Ki = 0.02; // Integral gain constant, to contorl the pressure fluctuations
double Kd = 0; // Derivative gain constant, to observe whether stress reaction speed is within our request
double Setpoint = 0; // Target relative distance 
double decre_distance = 30; // initial position + decre_distance = setpoint, decre_distance is the decrement of spring which is target force
double Input; // Error, in this case is "sensor_value", we want it to be equal to Setpoint
double Output; // the PID adjusted PWM value to achieve Input to be equal to Setpoint
double tuning_speed = 20;
double output_final; // store speed_default_value + Output PWM value
//int diff = 50; // threshold bewteen Input and Setpoint
int input_error = 0;// error bewteen Input and Setpoint
int error_tolerance = 3;
unsigned long time0;
unsigned long time1;
double interval = 6000;

/*Construct class of PID and passing it several pointers */
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
void setup() 
{
  // the setup routine runs once when you press reset:
  Serial.begin(9600);
  pinMode(LS_pin[0], Input);  // set pin 50 as CWS Input  
  pinMode(LS_pin[1], Input);  // set pin 52 as CCWS Input
  pinMode(PWM, OUTPUT); //PWM speed control
  pinMode(dir_pin[0], OUTPUT);// direction control
  pinMode(dir_pin[1], OUTPUT);

  /* set default state */
  analogWrite(PWM, 0); // initial set pwm = 0
  digitalWrite(dir_pin[0], LOW); // "LOW" is to activate limit switch, as "HIGH" is default statue
  digitalWrite(dir_pin[1], HIGH);

  digitalWrite(LS_pin[0], LOW);
  digitalWrite(LS_pin[1], LOW);// "LOW" is to activate limit switch, as "HIGH" is default statue
  
  modeSet();
  reset();

  /* initialize timer3 */
  Timer3.initialize(10000);
  Timer3.attachInterrupt(firstReading);
  /*  initialize timer1 */
  Timer1.initialize(100000);         // initialize timer1, and set a 0.1 second period
  Timer1.attachInterrupt(pidCompute);  // attaches callback() as a timer overflow interrupt

  /*Setup needed for PID used for pressure adjust*/
  myPID.SetMode(AUTOMATIC); // set the mode of output PID to be automatic
  myPID.SetSampleTime(5);  // set sample time of PID to be 0.05s
  myPID.SetOutputLimits(0, tuning_speed); // set max,min limit of output PID
}


/* main function*/
void loop() 
{ 

 


 /* recieve ROS stop command, and deattchinterrupt */ 
//  Timer1.detachInterrupt();
   


}

/* set setpoint = first sensor_value + decrement of spring (target force respective)*/
double firstReading()
{
  /* interrupt of Timer1 to check sensor value at different time */
  time0 = 0;
  time1 = millis();
  if(time1 - time0 < interval)
  {
    first_sensor_value = map(analogRead(A0), 0, 1023, 0, 255);
    return Setpoint = first_sensor_value + decre_distance;
  }
  
//  last_sensor_value = sensor_value;  
}

/* PID algorithm apply to do a feedback control for speed to reach target force */
void pidCompute()
{
  LS_state[0] = digitalRead(LS_pin[0]);
  LS_state[1] = digitalRead(LS_pin[1]);
  
//  Serial.println("CW  |  CCW");
//  Serial.println(LS_state[0]);
//  Serial.println(LS_state[1]);

  if(LS_state[1] == 1)
  {
    analogWrite(PWM, 0);
    delay(5000);
    Timer1.detachInterrupt();
  }
  else
  { 
     firstReading();
     Serial.println(Setpoint);
     Input = map(analogRead(A0), 0, 1023, 0, 255);

    Serial.println("relative distance");
    Serial.println(Input);
     
   /*PID Tuning to make motor close to target position */
    myPID.Compute(); //PID will compute adjusted PWM value and store it to Output Speed
    output_final =  Output; // adjust final PWM value that will be outputted

//  input_error = Input - Setpoint;
//  Serial.println("Input error");
//  Serial.println(input_error);

   if ( (Setpoint - error_tolerance) < Input && Input < (Setpoint + error_tolerance) )
   {
    //TODO: maintain the position in a duration and move to the reset position
    output_final = 0;
    Serial.println(" right force now");
   }
   else if ( Input >= Setpoint )
   { 
    Serial.println("up");
    digitalWrite(dir_pin[0], HIGH);  
   }
   else
   {
    Serial.println("down");
    digitalWrite(dir_pin[0], LOW);
    }
   
   /*To make sure our motor is turning within our desired turning speed limit*/
   if (output_final >  tuning_speed)
   {
     output_final =  tuning_speed;
   }
   else if (output_final < 1)
   {
     output_final = 0;
   }
   Serial.println("output speed");
   Serial.println(Output);
   // output pwm
   analogWrite(PWM, output_final);  
   Serial.println("output final");
   Serial.println(output_final);
  }
    /*stop when touch limit switch2 (SQ2) || receive stop sigal from ROS */
//    stopExecute();
}

/* interrupt of Timer1 to check sensor value at different time */
//void valueChange()
//{
//  last_sensor_value = sensor_value;
//  sensor_value = analogRead(A0);
//
//  pidCompute();
//  
//}

/* set mode0, to make sure motor go back to start position */
void modeSet()
{
   LS_state[0] = digitalRead(LS_pin[0]);
   LS_state[1] = digitalRead(LS_pin[1]);

   Serial.println("go up");
   Serial.println(LS_state[0]);
   Serial.println(LS_state[1]);
   digitalWrite(dir_pin[0], HIGH);
   analogWrite(PWM, reset_speed);
}

/* go to a consistent posisition everytime start the program */
void reset()
{
    LS_state[0] = digitalRead(LS_pin[0]);
    LS_state[1] = digitalRead(LS_pin[1]);

 
     Serial.println("go down");
     Serial.println(LS_state[0]);
     Serial.println(LS_state[1]);
     digitalWrite(dir_pin[0], LOW);
     analogWrite(PWM, reset_speed);
     delay(1000);
     analogWrite(PWM, 0);
     delay(5000);    
}


/* stop function */
//void stopExecute()
//{
//  LS_state[0] = digitalRead(LS_pin[0]);
//  LS_state[1] = digitalRead(LS_pin[1]);
//  if (LS_state[0] == 1)
//  {
//    analogWrite(PWM, 0);
//  }
//  else if (stop_check)
//  {
//    analogWrite(PWM, 0);
//    reset();
//  }
//  else
//  {
//    
//  }
//}



 








