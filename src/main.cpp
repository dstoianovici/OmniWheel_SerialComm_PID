////////////////////////////////////////////////////////////////////////////////
/*

Motor Control code for Nyku Omni Wheel Base

Takes in roll pitch yaw string from serial input from python controller.
Parses string into int so that PID controller can use it as a set point.

Hardware: Cytron 4D04A Motor Driver, 4x Pololu #3480 Motor,
          Arduino Uno, 3548S-1AA-103A-ND Potentiometer from Digikey

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
#define PID_FREQ 60

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
#define MIDPOINT 512

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
int setpoints[NUM_PARAMS] = {MIDPOINT,MIDPOINT,MIDPOINT,MIDPOINT}, setpoints_old[NUM_PARAMS] = {MIDPOINT,MIDPOINT,MIDPOINT,MIDPOINT};

//PID Vars
float kP[4] = {2.2,2.2,2.2,2.2};
// float kI[4] = {0,0,0,0};
float kI[4] = {0.2,.2,.2,.2};
float kD[4] = {0.8,0.8,0.8,0.8};

float deadband = 7.0;


float volatile currentTime[NUM_MOTORS], previousTime[NUM_MOTORS], elapsedTime[NUM_MOTORS];
float volatile error[NUM_MOTORS]={0,0,0,0}, cumError[NUM_MOTORS]={0,0,0,0}, rateError[NUM_MOTORS]={0,0,0,0}, lastError[NUM_MOTORS]={0,0,0,0};

const int hist_length = 60;
int iter = 0;

float  cumError_hist0[hist_length];
float  cumError_hist1[hist_length];
float  cumError_hist2[hist_length];
float  cumError_hist3[hist_length];

// float volatile cumError_hist0[hist_length];
// float volatile cumError_hist1[hist_length];
// float volatile cumError_hist2[hist_length];
// float volatile cumError_hist3[hist_length];

//PID function Prototype
float computePID(int setpoint, int state, int channel,float _deadband);
float bound(float val, float range);
int Error_Hist(float Error, float* Error_Hist, int hist_size, int _iter, float _deadband);
float average(float* arr, int len);

void setup() {
  // Serial.begin(BAUD_RATE);
  // pinMode(13,OUTPUT);
  // delay(5000);
  // digitalWrite(13,HIGH);
  // delay(500);
  // digitalWrite(13,LOW);
  // delay(100);
  // digitalWrite(13,HIGH);
  // delay(100);
  // digitalWrite(13,LOW);
  // delay(800);
  // digitalWrite(13,HIGH);
  // delay(500);
  // digitalWrite(13,LOW);
  //Serial.println("Initialized");

}

void loop() {
  int PID_flag = 1;
  //int setpoint[NUM_PARAMS]; //array of intergers corresponting to number of parameters neededs
  int* param_checks = parser.GetParams(setpoints); //Pull in setpoints from serial parser

   if(param_checks[1] != 0){ // Parameters out of range
     //Serial.println("params out of range");
     for(int i=0;i<NUM_PARAMS;i++){
       setpoints[i] = setpoints_old[i]; // Will reset to midpoint if these
     }
     PID_flag = 0;
   }

   else if(param_checks[0] != NUM_PARAMS){
     if(param_checks[0] == 0) PID_flag = 1; //Serial.println("nothing recieved");
     else{
        //Serial.println("params do not match");
        PID_flag = 0;
      }
   }

   if (PID_flag == 1){//If serial Value are ok
      if(param_checks[0] == NUM_PARAMS){
        for(int i=0;i<NUM_PARAMS;i++){
             cumError[i] = 0;
        }
      }

      for(int i=0;i<NUM_PARAMS;i++){
           setpoints_old[i] = setpoints[i];
      }


      int out0 = computePID(setpoints[0], pot0.GetVal(), 0, deadband);
      // int out1 = bound(computePID(setpoints[1], pot1.GetVal(), 1, deadband),35); //Bounts output to 0 if between +-35
      int out1 = computePID(setpoints[1], pot1.GetVal(), 1, deadband);
      int out2 = computePID(setpoints[2], pot2.GetVal(), 2, deadband);
      int out3= computePID(setpoints[3], pot3.GetVal(), 3, deadband);

      int out0_flag = Error_Hist(cumError[0], cumError_hist0, hist_length, iter, deadband);
      int out1_flag = Error_Hist(cumError[1], cumError_hist1, hist_length, iter, deadband);
      int out2_flag = Error_Hist(cumError[2], cumError_hist2, hist_length, iter, deadband);
      int out3_flag = Error_Hist(cumError[3], cumError_hist3, hist_length, iter, deadband);

      iter++;
      if(hist_length<=iter) iter = 0;

      int output0;
      int output1;
      int output2;
      int output3;

      if(out0_flag == 1) output0 = 0;
      else output0 = out0;
      if(out1_flag == 1) output1 = 0;
      else output1 = out1;
      if(out2_flag == 1) output2 = 0;
      else output2 = out2;
      if(out3_flag == 1) output3 = 0;
      else output3 = out3;

      motor0.setSpeed(output0);
      motor1.setSpeed(output1);
      motor2.setSpeed(output2);
      motor3.setSpeed(output3);

      // Serial.print("output flag: ");
      // Serial.println(out0_flag);
      Serial.print("Elapsed Time: ");
      Serial.println(elapsedTime[2]);
      Serial.print("Setpoint and State: ");
      Serial.print(setpoints[2]);
      Serial.print(",");
      Serial.println(pot2.GetVal());
      Serial.print("Error: ");
      Serial.println(error[2]);
      Serial.print("Cumulative Error: ");
      Serial.println(cumError[2]);
      Serial.print("Output: ");
      Serial.println(output2);
      Serial.println();
      delay(1000/PID_FREQ);
    }
  }




///////////////////PID Function////////////////////////////////////////
float computePID(int setpoint, int state, int channel,float _deadband){

  currentTime[channel] = millis();
  elapsedTime[channel] = (currentTime[channel]-previousTime[channel])/1000;

  error[channel] = float(setpoint - state);
  if(abs(error[channel]) <= _deadband){ error[channel] = 0;}
  //else;

  cumError[channel] += error[channel]*elapsedTime[channel];
  rateError[channel] = (error[channel]-lastError[channel])/elapsedTime[channel];

  float out = kP[channel]*error[channel] + kI[channel]*cumError[channel] + kD[channel]*rateError[channel];

  lastError[channel] = error[channel];
  previousTime[channel] = currentTime[channel];

  return -out;
}

float bound(float val, float range){
  float out = val;
  if(abs(val)< range){
    out = 0;
  }
  return out;
}
float average(float* arr, int len){
  float sum = 0;
  for(int i=0; i<len; i++){
    sum += arr[i];
  }
  float avrg = sum/len;
  return avrg;
}

int Error_Hist(float Error, float* Error_Hist, int hist_size, int _iter, float _deadband){
    int flag = 0;
    Error_Hist[_iter] = Error;
    float avg = average(Error_Hist, hist_size);

    if(abs(avg - Error)<deadband) flag = 1;
    else flag = 0;

    return flag;
}
