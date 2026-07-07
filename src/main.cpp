#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

WiFiUDP Udp;
const int UDP_PORT = 8888;

#include "command_functions.h"
#include "udp_comm.h"



//---------------------------------------------------------
// Timing variables in esp_timer_get_timeeconds
// please do not adjust any of the values as it can affect important operations
uint64_t udpCommTime, udpCommTimeInterval = 5000;
uint64_t sensorReadTime, sensorReadTimeInterval = 50000;
uint64_t ctrlTime, ctrlTimeInterval = 50000;

uint64_t wifiConnCheckTime, wifiConnCheckTimeInterval = 1000000;
//---------------------------------------------------------



//--------------- WIFI CONNECTION FUNCTION (ACCESSPOINT MODE) ------------------
void connect_wifi()
{
  Serial.print("Setting up Access Point...");

  // Set WiFi mode to Access Point
  WiFi.mode(WiFiMode_t::WIFI_AP); 
  
  // Start the Access Point
  bool success = WiFi.softAP(WIFI_SSID, WIFI_PASS);

  if (success) {
    Serial.println(" Success!");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP()); // Usually 192.168.4.1
  } else {
    Serial.println(" Failed to start AP!");
    return;
  }

  // Visual/Audio confirmation of successful AP boot
  led.off();
  buzzer.off();

  // Initialize mDNS (So you can still use mobobot.local)
  if (!MDNS.begin("mobobot")) {   
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
}

void check_wifi_connection()
{
  // In AP Mode, the ESP32 is hosting the network, so it doesn't "lose connection" 
  // the same way a station does. We just make sure the AP interface is still active.
  if (WiFi.getMode() == WiFiMode_t::WIFI_AP) {
    return;
  }

  Serial.println("AP mode lost. Restarting AP...");

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




// //--------------- WIFI CONNECTION FUNCTION (STATION MODE) ------------------
// void connect_wifi()
// {
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(WIFI_SSID, WIFI_PASS);

//   Serial.print("Connecting to WiFi");

//   uint32_t start_attempt = millis();

//   while (WiFi.status() != WL_CONNECTED)
//   {
//     // led.off();
//     // buzzer.off();
//     // delay(250);
//     // led.on();
//     // buzzer.on();
//     // delay(250);

//     led.blink(250);
//     buzzer.beep(250);
//     Serial.print(".");

//      // optional timeout
//     if (millis() - start_attempt > 10000)
//     {
//       Serial.println("\nRetry WiFi...");
//       WiFi.disconnect();
//       WiFi.begin(WIFI_SSID, WIFI_PASS);
//       start_attempt = millis();
//     }
//   }
//   led.off();
//   buzzer.off();

//   Serial.println();
//   Serial.print("Connected. ESP32 IP: ");
//   // Serial.println(WiFi.localIP());
//   // Initialize mDNS
//   if (!MDNS.begin("mobobot")) {   // Set the hostname to "esp32.local"
//     Serial.println("Error setting up MDNS responder!");
//     while(1) {
//       delay(1000);
//     }
//   }
//   Serial.println("mDNS responder started");
// }

// void check_wifi_connection()
// {
//   if (WiFi.status() == WL_CONNECTED)
//     return;

//   Serial.println("WiFi lost. Reconnecting...");

//   led.off();
//   buzzer.off();

//   udp_started = false;

//   switch (motor_cmd_type)
//   {
//   case 0:
//     controller.writePWM(0, 0);
//     break;

//   case 1:
//     controller.writeSpeed(0.0, 0.0);
//     break;

//   case 2:
//     controller.writeSpeed(0.0, 0.0);
//     break;
//   }

//   motor_vel[0] = 0.0;
//   motor_vel[1] = 0.0;

//   motor_pwm[0] = 0;
//   motor_pwm[1] = 0;

//   cmd_vel[0] = 0.0;
//   cmd_vel[1] = 0.0;

//   connect_wifi();
// }
// //-------------------------------------------------------






void setup()
{
  loadStoredParams();

  // Serial.begin(57600);
  Serial.begin(115200);
  // Serial.begin(460800);
  // Serial.begin(921600);
  // Serial.setTimeout(2);

  Wire.begin();

  // buzzer aleady initialized
  buzzer.off();

  // LED already initialized
  led.off();

  // RGB LED already initialized
  rgb_led.off();

  // sonar already initialzed

  // initialize Servo and Gripper
  servo.begin();
  gripper.begin();

  // initilize tof sensor
  bool is_initialized = tof.initialize();
  if (!is_initialized) {
    Serial.println("Error Initializing TOF Sensor. Pls Check Connection");
    buzzer.on();
    led.on();
    delay(1500);
    buzzer.off();
    led.off();
  };

  // initialize EPMC Motor Contoller
  epmc_connected = controller.begin(); // takes about 3 to 4 secs to connect
  if (!epmc_connected) {
    Serial.println("Error Connecting to EPMC. Probably due to Supported NumOfMotor Mismatch");
    buzzer.on();
    led.on();
    delay(1500);
    buzzer.off();
    led.off();
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


  servo.write(0);
  gripper.gripAngle(75);


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

      rgb_led.off();
      gripper.gripAngle(75);
      servo.write(0);
    
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
      sonar_read_dist_mm = sonar.readDist();

      tof_read_dist_mm = tof.readDist();

      line_sensor_read_val = line_sensor.readFullValue();

      // TBD - would add that of color sensor later

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


    if( (now_us - ctrlTime) > ctrlTimeInterval ) {

      // servo write command already called during udp communication
      // gripper write command already called during udp communication

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