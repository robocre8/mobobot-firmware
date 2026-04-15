#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

WiFiUDP Udp;
const int UDP_PORT = 8888;

#include "command_functions.h"
#include "udp_comm.h"



//------------ WIFI CONFIG --------------
const char* WIFI_SSID = "mobobot1234";
const char* WIFI_PASS = "mobobot1234";
//---------------------------------------



//---------------------------------------------------------
// Timing variables in esp_timer_get_timeeconds
// please do not adjust any of the values as it can affect important operations
uint64_t udpCommTime, udpCommTimeInterval = 5000;
uint64_t sensorReadTime, sensorReadTimeInterval = 50000;
uint64_t ctrlTime, ctrlTimeInterval = 50000;

uint64_t wifiConnCheckTime, wifiConnCheckTimeInterval = 1000000;
//---------------------------------------------------------



//--------------- WIFI CONNECTION FUNCTION ------------------
void connect_wifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");

  uint32_t start_attempt = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    // led.off();
    // buzzer.off();
    // delay(250);
    // led.on();
    // buzzer.on();
    // delay(250);

    led.blink(250);
    buzzer.beep(250);
    Serial.print(".");

     // optional timeout
    if (millis() - start_attempt > 10000)
    {
      Serial.println("\nRetry WiFi...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      start_attempt = millis();
    }
  }
  led.off();
  buzzer.off();

  Serial.println();
  Serial.print("Connected. ESP32 IP: ");
  // Serial.println(WiFi.localIP());
  // Initialize mDNS
  if (!MDNS.begin("mobobot")) {   // Set the hostname to "esp32.local"
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
}

void check_wifi_connection()
{
  if (WiFi.status() == WL_CONNECTED)
    return;

  Serial.println("WiFi lost. Reconnecting...");

  led.off();
  buzzer.off();

  udp_started = false;

  switch (motor_cmd_type)
  {
  case 0:
    controller.writePWM(0, 0);
    break;

  case 1:
    controller.writeSpeed(0.0, 0.0);
    break;

  case 2:
    controller.writeSpeed(0.0, 0.0);
    break;
  }

  motor_vel[0] = 0.0;
  motor_vel[1] = 0.0;

  motor_pwm[0] = 0;
  motor_pwm[1] = 0;

  cmd_vel[0] = 0.0;
  cmd_vel[1] = 0.0;

  connect_wifi();
}
//-------------------------------------------------------






void setup()
{
  // Serial.begin(57600);
  Serial.begin(115200);
  // Serial.begin(460800);
  // Serial.begin(921600);
  // Serial.setTimeout(2);
  Wire.begin();

  // buzzer_init();
  rgb_init();
  sonar_filter.begin();

  // gripper.begin();
  servo_init();

  rgb_off();
  servo1.write(servoAngle(0));
  servo2.write(servoAngle(0));

  epmc_connected = controller.begin(); // takes about 3 to 4 secs to connect
  if (!epmc_connected) {
    Serial.println("Error Connecting to EPMC. Probably due to Supported NumOfMotor Mismatch");
    buzzer.on();
    delay(1000);
    buzzer.off();
  }

  // ---------------- WIFI CONNECT ----------------
  connect_wifi();
  // ---------------- UDP START ----------------
  Udp.begin(UDP_PORT);
  udp_started = true;
  // -------------------------------------------  

  led.off();
  delay(500);
  led.on();
  delay(1500);
  led.off();

  // gripper.open();

  if(epmc_connected){
    controller.clearDataBuffer();
    controller.writeSpeed(0.0, 0.0);
    controller.setCmdTimeout(0); // set motor command velocity timeout
  }

  // Initialize timing markers
  uint64_t now_us = esp_timer_get_time();
  udpCommTime = now_us;
  sensorReadTime = now_us;
  ctrlTime = now_us;
  wifiConnCheckTime = now_us;
  udp_conn_time_check = now_us;
}

void loop()
{
  uint64_t now_us = esp_timer_get_time();

  // wifi conn/disconnect check
  if( (now_us - wifiConnCheckTime) > wifiConnCheckTimeInterval ) {
    // Reconnect WiFi if needed
    check_wifi_connection();

    // Restart UDP if needed
    if (WiFi.status() == WL_CONNECTED && !udp_started)
    {
      Serial.println("Restarting UDP server...");
      Udp.begin(UDP_PORT);
      udp_started = true;
    }
    wifiConnCheckTime = now_us;
  }

  // udp comm/disconnect check
  if( (now_us - udp_conn_time_check) > udp_comm_timeout_us ) {
    if(emergency_stop) {

      if (epmc_connected){
        switch (motor_cmd_type)
        {
        case 0:
          controller.writePWM(0, 0);
          break;

        case 1:
          controller.writeSpeed(0.0, 0.0);
          break;

        case 2:
          controller.writeSpeed(0.0, 0.0);
          break;
        }
      }

      motor_vel[0] = 0.0;
      motor_vel[1] = 0.0;

      motor_pwm[0] = 0;
      motor_pwm[1] = 0;

      cmd_vel[0] = 0.0;
      cmd_vel[1] = 0.0;

      buzzer.off();
      is_beep = false;

      rgb_off();
      servo1.write(servoAngle(0));
      servo2.write(servoAngle(0));
      // gripper.open();
    
      emergency_stop = false;
    }
    allow_system_function = false;
  }

  // udp comm loop
  if( (now_us - udpCommTime) > udpCommTimeInterval ) {
    recieve_and_send_data();
    udpCommTime = now_us;
  }

  if (allow_system_function) {
    if(is_beep) buzzer.beep(beep_period);

    // sensor read loop
    if( (now_us - sensorReadTime) > sensorReadTimeInterval ) {
      sonar_dist_mm = read_sonar();

      // line_sensor1_read = lineSensor1.read();
      // line_sensor2_read = lineSensor2.read();

      if (epmc_connected){
        float tl=0.0, tr=0.0;
        // float wl=0.0, wr=0.0;
        // float x=0.0, y=0.0 v=0.0, w=0.0;
        float t=0.0, d=0.0;
        controller.readPos(tl, tr);
        // controller.readVel(wl, wr);

        motor_states[0] = tl;
        // motor_states[1] = wl;
        motor_states[2] = tr;
        // motor_states[4] = wr;

        d = R*(tr+tl)/2.0;
        t = R*(tr-tl)/L;
        // x = d*cos(t);
        // y = d*sin(t);
        // v = R*(wr+wl)/2.0;
        // w = R*(wr-wl)/2.0;

        // odom_data[0] = x;
        // odom_data[1] = y;
        odom_data[2] = t;
        // odom_data[3] = v;
        // odom_data[4] = w;
        odom_data[5] = d;
      }
  
      sensorReadTime = now_us;
    }

    // gripper control loop
    if( (now_us - ctrlTime) > ctrlTimeInterval ) {
      // gripper.open(gripper_dist_mm);

      servo1.write(servoAngle(servo1_angle_deg));
      servo2.write(servoAngle(servo2_angle_deg));

      if (epmc_connected) {
        switch (motor_cmd_type)
        {
        case 0:
          controller.writePWM(motor_pwm[0], motor_pwm[1]);
          break;

        case 1:
          controller.writeSpeed(motor_vel[0], motor_vel[1]);
          break;

        case 2:
          float wl=0.0, wr=0.0;
          wl = computeWL(cmd_vel[0], cmd_vel[1]);
          wr = computeWR(cmd_vel[0], cmd_vel[1]);
          controller.writeSpeed(wl, wr);
          break;
        }
      }

      ctrlTime = now_us;
    }
  }

}
