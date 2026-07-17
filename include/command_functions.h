#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <EPMC_I2C_Client.h>
// #include <EIMU_I2C_Client.h>
#include "buzzer.h"
#include "sonar_sensor.h"
#include "tof_sensor.h"
#include "servo_control.h"
#include "linear_gripper_control.h"
#include <led.h>
#include "rgb_led.h"
#include "line_sensor_5_channel.h"

//------------ WIFI CONFIG --------------
const char* WIFI_SSID = "mobobot1234";
const char* WIFI_PASS = "mobobot1234";
//---------------------------------------


//------------ Communication Command IDs --------------//
enum CommandID : uint8_t {
  START_BYTE = 0xAA,
  READ_DATA = 0x01,
  WRITE_SERVO_ANGLE = 0x02,
  WRITE_GRIPPER_ANGLE = 0x03,
  WRITE_GRIPPER_DIST = 0x04,
  WRITE_BUZZER = 0x05,
  WRITE_LED = 0x06,
  WRITE_RGB_LED = 0x07,
  WRITE_MOTOR_VEL = 0x08,
  WRITE_MOTOR_PWM = 0x09,
  WRITE_CMD_VEL = 0x0A,
  SET_WHEEL_ODOM_PARAMS = 0x0B,
  CLEAR_CONTROLLER_DATA = 0x0C,
  SET_CONTROLLER_CMD_TIMEOUT = 0x0D,
  SET_UDP_CONN_TIMEOUT = 0x0E,
  UDP_HEART_BEAT = 0x0F,
  RESET_PARAMS = 0x10,
  SET_WHEEL_RADIUS = 0x11,
  SET_WHEEL_DISTANCE = 0x12,
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
const int SERVO_PIN = 19;
ServoControl servo(SERVO_PIN);

const int GRIPPER_PIN = 18;
LinearGripperControl gripper(GRIPPER_PIN);
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

RGBLed rgb_led(R_LED_PIN, G_LED_PIN, B_LED_PIN);
//--------------------------


//---------------------------
uint8_t epmc_i2c_address = 0x55;
EPMC_I2C_Client controller(epmc_i2c_address);

bool epmc_connected = false;

float R = 0.034; //wheel radius
float L = 0.173; //wheel seperation

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
const int TRIGGER_PIN = 33;
const int ECHO_PIN = 35;
int SONAR_TIMEOUT_MS = 15;

SonarSensor sonar(TRIGGER_PIN, ECHO_PIN, SONAR_TIMEOUT_MS);

TOFSensor tof;
bool tof_connected = false;

int sonar_read_dist_mm;
int tof_read_dist_mm;
// ---------------------------




// ---------------------------
const int pin0 = 36;
const int pin1 = 39;
const int pin2 = 34;
const int pin3 = 32;
const int pin4 = 13;

LineSensor5Channel line_sensor(pin0, pin1, pin2, pin3, pin4);

int line_sensor_read_val;
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
float writeServoAngle(int angle_deg)
{
  servo.write(angle_deg);
  return 1.0;
}

float writeGripperAngle(int angle_deg)
{
  gripper.gripAngle(angle_deg);
  return 1.0;
}

float writeGripperDist(int dist_mm)
{
  gripper.gripDist(dist_mm);
  return 1.0;
}

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

float writeLed(int val)
{
  int led_val = constrain(val, 0, 1);
  if (led_val) led.on();
  else led.off();
  return 1.0;
}

float writeRGB(int r_pwm, int g_pwm, int b_pwm)
{
  rgb_led.on(r_pwm, g_pwm, b_pwm);
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

float readData(float &sonar_val, float &tof_val, float &line_sensor_val, 
              float &tl, float &tr, float &yaw, float &dist, 
              float &color_sensor, float &wheelRadiusParam, 
              float &wheelDistanceParam, float &maxWheelSpeedParam)
{
  sonar_val = sonar_read_dist_mm;
  tof_val = tof_read_dist_mm;
  line_sensor_val = line_sensor_read_val;
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