//Reference: https://github.com/ToanTech/DengFOC_Lib
//This code incorporates a integral separated PID control to improve the performance

#include "AS5600.h"

#include "PID.c"
#include "PID.h"

int pwmA = 32;
int pwmB = 33;
int pwmC = 25;

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float voltage_limit=12.6;
float voltage_power_supply=12.6;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
#define _3PI_2 4.71238898038f

pid_struct_t FOC = {0};

int PP=7,DIR=1;
float _electricalAngle(){
  return  _normalizeAngle((float)(DIR *  PP) * getAngle_Without_track()-zero_electric_angle);
}


float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   
  return a >= 0 ? a : (a + 2*PI);  
}



void setPwm(float Ua, float Ub, float Uc) {
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  ledcWrite(0, dc_a*255);
  ledcWrite(1, dc_b*255);
  ledcWrite(2, dc_c*255);
}

void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el);
  // Inverse Park
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // Inverse Clark
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}



float motor_target;
float FOC_OUT = 0;
float switch_angle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  ledcAttachPin(pwmA, 0);
  ledcAttachPin(pwmB, 1);
  ledcAttachPin(pwmC, 2);
  ledcSetup(0, 30000, 8);  
  ledcSetup(1, 30000, 8);  
  ledcSetup(2, 30000, 8); 
  Serial.println("Initialize");
  BeginSensor();
  setPhaseVoltage(3, 0,_3PI_2);
  delay(3000);
  zero_electric_angle=_electricalAngle();
  motor_target = getAngle();
  switch_angle = motor_target;
  setPhaseVoltage(0, 0,_3PI_2);
  Serial.print("Zero electric angle：");Serial.println(zero_electric_angle);
  Serial.println(switch_angle);
  
  pid_init(&FOC, 2, 0.02, 0, 1023, 1023, 5);
}



float shift_switch_position(){
  switch_angle = getAngle();
    if (switch_angle > motor_target + PI/4){
      motor_target = motor_target + PI/2;
      return motor_target;}
    if (switch_angle < motor_target - PI/4){
      motor_target = motor_target - PI/2;
      return motor_target;}
    else{return motor_target;}
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(getAngle());
  Serial.print(",");
  float Sensor_Angle =  getAngle();
  float shift_switch_target = shift_switch_position();
  FOC_OUT = pid_general(&FOC, DIR*Sensor_Angle,shift_switch_target);
  setPhaseVoltage(_constrain(FOC_OUT,-6,6),0,_electricalAngle());
  Serial.print(_normalizeAngle(getAngle()));
  Serial.print(",");
  Serial.print(shift_switch_target);
  Serial.print(",");
  Serial.print(_electricalAngle());
  Serial.print(",");
  Serial.println(FOC_OUT);
  delay(3);
}