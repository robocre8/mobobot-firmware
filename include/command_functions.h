#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <Led.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>
#include <movingAvg.h>  
#include "gripper.h"
#include <EPMC_I2C_Client.h>
// #include <EIMU_I2C_Client.h>
#include "buzzer.h"
#include "ir_sensor.h"
#include "tof_sensor.h"


//------------ WIFI CONFIG --------------
const char* WIFI_SSID = "mobobot1234";
const char* WIFI_PASS = "mobobot1234";
//---------------------------------------


//------------ Communication Command IDs --------------//
enum CommandID : uint8_t {
  START_BYTE = 0xAA,
  READ_DATA = 0x01,
  WRITE_SERVO1_ANGLE = 0x02,
  WRITE_SERVO2_ANGLE = 0x03,
  WRITE_BUZZER = 0x04,
  WRITE_RGB = 0x05,
  WRITE_MOTOR_VEL = 0x06,
  WRITE_MOTOR_PWM = 0x07,
  WRITE_CMD_VEL = 0x08,
  SET_WHEEL_ODOM_PARAMS = 0x09,
  CLEAR_CONTROLLER_DATA = 0x0A,
  SET_CONTROLLER_CMD_TIMEOUT = 0x0B,
  SET_UDP_CONN_TIMEOUT = 0x0C,
  UDP_HEART_BEAT = 0x0D,
  RESET_PARAMS = 0x0E,
  SET_WHEEL_RADIUS = 0x0F,
  SET_WHEEL_DISTANCE = 0x10,
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

float R = 0.034; //wheel radius
float L = 0.165; //wheel seperation

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






//--------------- storage variables -----------------//
bool firstLoad = false;

Preferences storage;

const char * R_key = "wheelRadius";

const char * L_key = "wheelDistance";

const char * firstLoad_key = "firstLoad";

const char * params_ns = "params"; // preference namespace

void resetParamsInStorage(){
  storage.begin(params_ns, false);

  storage.putFloat(R_key, 0.034);
  storage.putFloat(L_key, 0.165);

  storage.end();
}

void initParams(){
  //check for firstLoad
  storage.begin(params_ns, true);
  firstLoad = storage.getBool(firstLoad_key);
  storage.end();
  // if firsLoad -> reset all params and set firstLoad to false
  if(firstLoad == true){
    resetParamsInStorage();
    firstLoad = false;
    storage.begin(params_ns, false);
    storage.putBool(firstLoad_key, firstLoad);
    storage.end();
  }

}

void loadStoredParams(){
  initParams();
  // load each parameter form the storage to the local variables
  storage.begin(params_ns, true);

  R = storage.getFloat(R_key, 0.034);
  L = storage.getFloat(L_key, 0.165);

  storage.end();
}
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

// float getUdpConnTimeout()
// { 
//   return (float)(udp_comm_timeout_us/1000);
// }

float udpHeartBeat()
{ 
  udp_conn_time_check = esp_timer_get_time();
  allow_system_function = true;
  emergency_stop = true;
  return 1.0;
}

float setWheelRadius(float r)
{
  R = r;

  storage.begin(params_ns, false);
  storage.putFloat(R_key, R);
  storage.end();

  return 1.0;
}

float setWheelDistance(float l)
{
  L = l;

  storage.begin(params_ns, false);
  storage.putFloat(L_key, L);
  storage.end();

  return 1.0;
}

float triggerResetParams()
{
  firstLoad = true;

  storage.begin(params_ns, false);
  storage.putBool(firstLoad_key, firstLoad);
  storage.end();
  // reload to reset
  // loadStoredParams();

  return 1.0;
}

//---------------------- READ DATA -------------------------//

float readData(float &sonar, float &tof, float &line_sensor, 
              float &tl, float &tr, float &yaw, float &dist, 
              float &color_sensor, float &wheelRadiusParam, 
              float &wheelDistanceParam, float &maxWheelSpeedParam)
{
  sonar = sonar_dist_mm;
  tof = 0.0;
  line_sensor = 0.0;
  tl = motor_states[0];
  tr = motor_states[2];
  yaw = odom_data[2];
  dist = odom_data[5];
  color_sensor = 0.0;
  wheelRadiusParam = R;
  wheelDistanceParam = L;
  maxWheelSpeedParam = 0.0;

  return 1.0;
}

//-------------------------------------------------------------------//


#endif