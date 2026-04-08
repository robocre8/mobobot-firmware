#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include <Wire.h>
#include <Led.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>
#include <movingAvg.h>  
// #include "gripper.h"
#include <EPMC_I2C_Client.h>
// #include <EIMU_I2C_Client.h>
#include "buzzer.h"
#include "ir_sensor.h"

//------------ Communication Command IDs --------------//
enum CommandID : uint8_t {
  START_BYTE = 0xAA,
  WRITE_SERVO1_ANGLE = 0x01,
  WRITE_SERVO2_ANGLE = 0x02,
  WRITE_BUZZER = 0x03,
  WRITE_RGB = 0x04,
  WRITE_MOTOR_VEL = 0X05,
  WRITE_MOTOR_PWM = 0X06,
  WRITE_CMD_VEL = 0x07,
  SET_WHEEL_ODOM_PARAMS = 0X08,
  GET_WHEEL_ODOM_PARAMS = 0X09,
  READ_SONAR = 0x0A,
  READ_LINE_SENSOR1 = 0x0B,
  READ_LINE_SENSOR2 = 0x0C,
  READ_MOTOR_STATES = 0X0D,
  READ_ODOM_DATA = 0X0E,
  READ_ALL_SENSORS = 0X0F,
  CLEAR_CONTROLLER_DATA = 0X10,
  SET_CONTROLLER_CMD_TIMEOUT = 0X11,
  SET_UDP_CONN_TIMEOUT = 0X12,
  GET_UDP_CONN_TIMEOUT = 0X13,
  UDP_HEART_BEAT = 0X14,
};
//---------------------------------------------------//


//--------------- global variables -----------------//
static_assert(sizeof(float) == 4, "Float must be 32-bit");

float readFloat(const uint8_t* data, uint8_t offset) {
  float v;
  memcpy(&v, &data[offset], sizeof(float));
  return v;
}

uint64_t udp_comm_timeout_us = 2500000, udp_conn_time_check;
bool udp_started = false;
bool allow_system_function = true;
bool emergency_stop = true;


//-------------------------
const int LED_PIN = 2;
Led led(LED_PIN);
//-------------------------


//--------------------------
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2450;
const int SERVO1_PIN = 19;
const int SERVO2_PIN = 18;
Servo servo1;
int servo1_angle_deg = 0;
Servo servo2;
int servo2_angle_deg = 0;

void servo_init() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);    // Standard 50 Hz servo frequency
  servo1.attach(SERVO1_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo2.setPeriodHertz(50);    // Standard 50 Hz servo frequency
  servo2.attach(SERVO2_PIN, SERVO_MIN_US, SERVO_MAX_US);
}

int servoAngle(int angle_deg) {
  int a = constrain(angle_deg, -90, 90);
  int angle = map(a, -90, 90, 0, 180);
  return angle;
}
//--------------------------


//--------------------------
// Gripper gripper(18);
// int gripper_dist_mm = gripper.GRIPPER_MAX_DIST;
// int gripper_dist_mm = 5;
//--------------------------


//--------------------------
const int BUZZER_PIN = 14;
Buzzer buzzer(BUZZER_PIN);

bool is_beep = false;
int beep_period = 1000;
//--------------------------


//--------------------------
const int R_LED_PIN = 27;
const int B_LED_PIN = 26;
const int G_LED_PIN = 25;

void rgb_init() {
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);

  analogWrite(R_LED_PIN, 255);
  analogWrite(G_LED_PIN, 255);
  analogWrite(B_LED_PIN, 255);
}

void rgb_on(int r_pwm, int g_pwm, int b_pwm) {
  analogWrite(R_LED_PIN, 255-r_pwm);
  analogWrite(G_LED_PIN, 255-g_pwm);
  analogWrite(B_LED_PIN, 255-b_pwm);
}

void rgb_off() {
  analogWrite(R_LED_PIN, 255);
  analogWrite(G_LED_PIN, 255);
  analogWrite(B_LED_PIN, 255);
}
//--------------------------


//---------------------------
uint8_t epmc_i2c_address = 0x55;
EPMC_I2C_Client controller(epmc_i2c_address);

bool epmc_connected = false;

float R = 0.03; //wheel radius
float L = 0.195; //wheel seperation

int motor_cmd_type = 0;

float motor_vel[2] = {
  0.0, // wl
  0.0 // wr
};

int motor_pwm[2] = {
  0, // l_pwm
  0 // r_pwm
};

float cmd_vel[2] = {
  0.0, // v
  0.0 // w
};

float motor_states[4] = {
  0.0, // tl
  0.0, // wl
  0.0, // tr
  0.0 // wr
};

float odom_data[6] = {
  0.0, // x
  0.0, // y
  0.0, // theta
  0.0, // v
  0.0, // w
  0.0 // d - total dist travelled
};

float computeWL(float v, float w) {
  return ((2.0*v - w*L)/(2.0*R));
}

float computeWR(float v, float w) {
  return ((2.0*v + w*L)/(2.0*R));
}
//-----------------------------


//-----------------------------
int SONAR_TIMEOUT_MS = 15;

const int TRIGGER_PIN = 33;
const int ECHO_PIN = 35;
Ultrasonic sonar(TRIGGER_PIN, ECHO_PIN, SONAR_TIMEOUT_MS);
movingAvg sonar_filter(10);
int sonar_dist_mm = 0;

int read_sonar(){
  int dist_mm = (int)(sonar.convert(sonar.timing(), Ultrasonic::CM)*10);
  int dist_filter = sonar_filter.reading(dist_mm);
  return dist_filter;
}
// ---------------------------




// ---------------------------
const int LINE_SENSOR1_PIN = 32;
const int LINE_SENSOR2_PIN = 34;

IRSensor lineSensor1(LINE_SENSOR1_PIN);
IRSensor lineSensor2(LINE_SENSOR2_PIN);

int line_sensor1_read = 0;
int line_sensor2_read = 0;
//-------------------------------------------------//




//--------------- global functions ----------------//
float writeServo1Angle(int angle_deg)
{
  servo1_angle_deg = angle_deg;
  return 1.0;
}

float writeServo2Angle(int angle_deg)
{
  servo2_angle_deg = angle_deg;
  return 1.0;
}

// float writeGripperDist(int dist_mm)
// {
//   gripper_dist_mm = dist_mm;
//   return 1.0;
// }

float writeBuzzer(int value)
{
  int val = abs(value);
  if (val>1) {
    if (val>=100 && val<=5000) {
      is_beep = true;
      beep_period = constrain(val, 100, 5000);
    } else {
      return 0.0;
    }
  } else {
    is_beep = false;
    if (val == 1) buzzer.on();
    else buzzer.off();
  }

  return 1.0;
}

float writeRGB(int r_pwm, int g_pwm, int b_pwm)
{
  rgb_on(r_pwm, g_pwm, b_pwm);
  return 1.0;
}

float writeMotorVel(float wl, float wr)
{
  motor_cmd_type = 1;
  motor_vel[0] = wl;
  motor_vel[1] = wr;
  return 1.0;
}

float writeMotorPwm(int l_pwm, int r_pwm)
{
  motor_cmd_type = 0;
  motor_pwm[0] = l_pwm;
  motor_pwm[1] = r_pwm;
  return 1.0;
}

float writeCmdVel(float v, float w)
{
  motor_cmd_type = 2;
  cmd_vel[0] = v;
  cmd_vel[1] = w;
  return 1.0;
}

float setWheelOdomParams(int R_mm, int L_mm)
{
  R = (float)R_mm/1000.0;
  L = (float)L_mm/1000.0;
  return 1.0;
}

float getWheelOdomParams(float &R_mm, float &L_mm)
{
  R_mm = R*1000.0;
  L_mm = L*1000.0;
  return 1.0;
}

// float readTofSensor()
// {  
//   return (float)tof_dist_mm;
// }

float readSonar()
{  
  return (float)sonar_dist_mm;
}

float readLineSensor1()
{ 
  return (float)lineSensor1.read();
}

float readLineSensor2()
{ 
  return (float) lineSensor2.read();
}

float readMotorStates(float &tl, float &wl, float &tr, float &wr)
{  
  tl = motor_states[0];
  wl = motor_states[1];
  tr = motor_states[2];
  wr = motor_states[3];

  return 1.0;
}

float readOdomData(float &x, float &y, float &theta, float &v, float &w, float &d)
{  
  x = odom_data[0];
  y = odom_data[1];
  theta = odom_data[2];
  v = odom_data[3];
  w = odom_data[4];
  d = odom_data[5];

  return 1.0;
}

// float readAllSensors(float &sonar, float &sonar2, float &tl, float &wl, float &tr, float &wr, float &x, float &y, float &theta, float &v, float &w, float &d)
// {
//   sonar = sonar_dist_mm;
//   sonar2 = sonar2_dist_mm;

  // tl = motor_states[0];
  // wl = motor_states[1];
  // tr = motor_states[2];
  // wr = motor_states[3];

//   x = odom_data[0];
//   y = odom_data[1];
//   theta = odom_data[2];
//   v = odom_data[3];
//   w = odom_data[4];
//   d = odom_data[5];

//   return 1.0;
// }

float readAllSensors(float &sonar, float &line_sensor1, float &line_sensor2, float &tl, float &tr, float &theta, float &dist)
{
  sonar = sonar_dist_mm;

  line_sensor1 = lineSensor1.read();
  line_sensor2 = lineSensor2.read();

  tl = motor_states[0];
  tr = motor_states[2];

  theta = odom_data[2];
  dist = odom_data[5];

  return 1.0;
}

float clearControllerData()
{ 
  controller.clearDataBuffer();
  controller.writeSpeed(0.0, 0.0);
  return 1.0;
}

float setControllerCmdTimeout(int timeout_ms)
{ 
  controller.setCmdTimeout(timeout_ms);
  return 1.0;
}

float setUdpConnTimeout(int timeout_ms)
{ 
  if(timeout_ms < 1000){
    return 0.0;
  }
  udp_comm_timeout_us = 1000*timeout_ms;
  return 1.0;
}

float getUdpConnTimeout()
{ 
  return (float)(udp_comm_timeout_us/1000);
}

float udpHeartBeat()
{ 
  udp_conn_time_check = esp_timer_get_time();
  allow_system_function = true;
  emergency_stop = true;
  return 1.0;
}
//-------------------------------------------------------------------//


#endif