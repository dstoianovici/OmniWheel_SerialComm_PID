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

#define DEAD_BAND 5 //deadband for PID

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
double kP[4] = {1,1,1,1};
double kI[4] = {0,0,0,0};
double kD[4] = {.2,.2,.2,.2};



unsigned long currentTime[NUM_MOTORS], previousTime[NUM_MOTORS], elapsedTime[NUM_MOTORS];
double  error[NUM_MOTORS],cumError[NUM_MOTORS],rateError[NUM_MOTORS], lastError[NUM_MOTORS];


//PID function Prototype
int computePID(int setpoint, int state, int channel,int deadband);


void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("Initialized");\

}

void loop() {

  int setpoint[NUM_PARAMS]; //array of 100 intergers
  int param_check = parser.GetParams(setpoint);
  //if (param_check == NUM_PARAMS){
    int out0 = computePID(setpoint[0], pot0.GetVal(), 0, DEAD_BAND);
    int out1 = computePID(setpoint[1], pot1.GetVal(), 1, DEAD_BAND);
    int out2 = computePID(setpoint[2], pot2.GetVal(), 2, DEAD_BAND);
    int out3= computePID(setpoint[3], pot3.GetVal(), 3, DEAD_BAND);

    motor0.setSpeed(int16(out0));
    // motor1.setSpeed(out1);
    // motor2.setSpeed(out2);
    // motor3.setSpeed(out3);

    Serial.println(out0);
    Serial.println(setpoint[0]);
    Serial.println(pot0.GetVal());
    Serial.println(error[0]);
    delay(500);

  //}
  //else{
    // motor0.setSpeed(0);
    // motor1.setSpeed(0);
    // motor2.setSpeed(0);
    // motor3.setSpeed(0);
//  }
}


double computePID(int setpoint, int state, int channel,int deadband){

  currentTime[channel] = millis();
  elapsedTime[channel] = currentTime[channel]-previousTime[channel];
  error[channel] = setpoint - state;

  if(error[channel] >= deadband){
    cumError[channel] += error[channel]*elapsedTime[channel];
    rateError[channel] = (error[channel]-lastError[channel])/elapsedTime[channel];

    double out =int(kP[channel]*error[channel] + kI[channel]*cumError[channel] + kD[channel]*rateError[channel]);


    lastError[channel] = error[channel];
    previousTime[channel] = currentTime[channel];

    return out;
  }

  else{
    return 0; //Do not power motor for small error
  }
}
