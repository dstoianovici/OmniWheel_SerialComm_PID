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
#define NUM_MOTORS 4

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
Serial_Parser parser(DELIM,range_M,range_m);

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
int setpoints[4], setpoints_old[4] = {0};



//PID Vars
float kP[4] = {1.5,1,1,1};
float kI[4] = {0.2,0,0,0};
float kD[4] = {0.7,0,0,0};

float deadband = 5.0;


float volatile currentTime[NUM_MOTORS], previousTime[NUM_MOTORS], elapsedTime[NUM_MOTORS];
float volatile error[NUM_MOTORS]={0}, cumError[NUM_MOTORS]={0}, rateError[NUM_MOTORS]={0}, lastError[NUM_MOTORS]={0};


//PID function Prototype
float computePID(int setpoint, int state, int channel,float deadband);


void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("Initialized");\

}

void loop() {
  int PID_flag = 1;
  int setpoint[NUM_PARAMS]; //array of intergers corresponting to number of parameters neededs
  int* param_checks = parser.GetParams(setpoint); //Pull in setpoints from serial parser

   if(param_checks[1] != 0){
     Serial.println("params out of range");
     PID_flag = 0;
   }

   else if(param_checks[0] != NUM_PARAMS){
     if(param_checks[0] == 0) Serial.println("nothing recieved");
     else{
        Serial.println("params do not match");
        PID_flag = 0;
      }
   }
   else;

   if (PID_flag == 1){//If serial Value are ok
      for(int i=0;i<4;i++){
           setpoints_old[i] = setpoints[i];
      }


      int out0 = computePID(setpoint[0], pot0.GetVal(), 0, deadband);
      // int out1 = computePID(setpoint[1], pot1.GetVal(), 1, DEAD_BAND);
      // int out2 = computePID(setpoint[2], pot2.GetVal(), 2, DEAD_BAND);
      // int out3= computePID(setpoint[3], pot3.GetVal(), 3, DEAD_BAND);

      motor0.setSpeed(out0);
      // motor1.setSpeed(out1);
      // motor2.setSpeed(out2);
      // motor3.setSpeed(out3);

      Serial.print("Elapsed Time: ");
      Serial.println(elapsedTime[0]);
      Serial.print("Setpoint and State: ");
      Serial.print(setpoint[0]);
      Serial.print(",");
      Serial.println(pot0.GetVal());
      Serial.print("Error: ");
      Serial.println(error[0]);
      Serial.print("Cumulative Error: ");
      Serial.println(cumError[0]);
      Serial.print("Output: ");
      Serial.println(out0);
      Serial.println();
      delay(500);
    }
    else;
  }




///////////////////PID Function////////////////////////////////////////
float computePID(int setpoint, int state, int channel,float _deadband){

  currentTime[channel] = millis();
  elapsedTime[channel] = (currentTime[channel]-previousTime[channel])/1000;

  error[channel] = float(setpoint - state);
  if(abs(error[channel]) <= _deadband){ error[channel] = 0;}
  else;

  cumError[channel] += error[channel]*elapsedTime[channel];
  // if (abs(cumError[channel]) > 3000){ cumError[channel] = 0;}
  // else;

  rateError[channel] = (error[channel]-lastError[channel])/elapsedTime[channel];

  float out = kP[channel]*error[channel] + kI[channel]*cumError[channel] + kD[channel]*rateError[channel];

  lastError[channel] = error[channel];
  previousTime[channel] = currentTime[channel];

  return -out;
}
