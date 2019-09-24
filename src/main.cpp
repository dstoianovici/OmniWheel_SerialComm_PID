////////////////////////////////////////////////////////////////////////////////
/*

Motor Control code for Nyku Omni Wheel Base

Takes in roll pitch yaw string from serial input from python controller.
Parses string into int so that PID controller can use it as a set point.


Dan Stoianovici 9/21/19

*/
////////////////////////////////////////////////////////////////////////////////


//Header Files
#include <Arduino.h>
#include <CytronMotorDriver.h>
#include <Analog_Pot.h>
#include <Serial_Parser.h>

//Serial Comms
#define BAUD_RATE 115200
#define DELIM ','
#define NUM_PARAMS 4


//Motor Pins
#define mot0_dir 2
#define mot0_en 3

#define mot1_dir 4
#define mot1_en 5

#define mot2_dir 7
#define mot2_en 6

#define mot3_dir 8
#define mot3_en 9

//Pot Pins
#define pot_0 A0
#define pot_1 A1
#define pot_2 A2
#define pot_3 A3

#define range_M 1023 //Max val of pot
#define range_m 0 //Min Val of pot

//Create Serial Parser object
Serial_Parser parser(DELIM);

//Create motor objects
CytronMD motor0(PWM_DIR, mot0_en, mot0_dir);
CytronMD motor1(PWM_DIR, mot1_en, mot1_dir);
CytronMD motor2(PWM_DIR, mot2_en, mot2_dir);
CytronMD motor3(PWM_DIR, mot3_en, mot3_dir);

//Potetntiometer
Analog_Pot pot0(pot_0,range_m,range_M);
Analog_Pot pot1(pot_1,range_m,range_M);
Analog_Pot pot2(pot_2,range_m,range_M);
Analog_Pot pot3(pot_3,range_m,range_M);

//Global Vals for PID control
int setpoints[4];

//PID Vars
double kP[4];
double kI[4];
double kD[4];

unsigned long currentTime, previousTime;
double elapsedTime, error,cumError,rateError;




void setup() {
  Serial.begin(BAUD_RATE);

  //Set mode for motor pins
  // pinMode(mot0_dir, OUTPUT);
  // pinMode(mot0_en, OUTPUT);
  // pinMode(mot1_dir, OUTPUT);
  // pinMode(mot1_en, OUTPUT);
  // pinMode(mot2_dir, OUTPUT);
  // pinMode(mot2_en, OUTPUT);
  // pinMode(mot3_dir, OUTPUT);
  // pinMode(mot3_en, OUTPUT);

  Serial.println("Initialized");\
}

void loop() {

  int setpoint[NUM_PARAMS]; //array of 100 intergers
  int param_check = parser.GetParams(setpoint);






  // Serial.println();
  // Serial.print(pot0.GetVal());
  // Serial.print(",");
  // Serial.print(pot1.GetVal());
  // Serial.print(",");
  // Serial.print(pot2.GetVal());
  // Serial.print(",");
  // Serial.println(pot3.GetVal());
  //
  //
  // motor0.setSpeed(pot0.GetVal2PWM());
  // motor1.setSpeed(pot1.GetVal2PWM());
  // motor2.setSpeed(pot2.GetVal2PWM());
  // motor3.setSpeed(pot3.GetVal2PWM());




}

int computePID(int setpoint, int state, int channel ){
  currentTime[channel] = millis();
  elapsedTime[channel] = currentTime[channel]-previousTime[channel];

  error[channel] = setpoint - state;
  cumError[channel] += error[channel]*







  previousTime = currentTime;

  return out;
}
