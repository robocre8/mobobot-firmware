#ifndef UDP_COMM_H
#define UDP_COMM_H

#include <WiFiUdp.h>

extern WiFiUDP Udp;
extern const int UDP_PORT;

#include "command_functions.h"

IPAddress remoteIP;
uint16_t remotePort;

static inline void udpSend(uint8_t* data, size_t len) {
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(data, len);
  Udp.endPacket();
}

static inline void processCommand(uint8_t cmd, uint8_t* data) {

  gpio_set_level((gpio_num_t)LED_PIN, 1);

  switch (cmd) {

    // case WRITE_GRIPPER_DIST: {
    //   float value = readFloat(data, 0);
    //   writeGripperDist((int)value);
    //   break;
    // }

    case WRITE_SERVO1_ANGLE: {
      float value = readFloat(data, 0);
      writeServo1Angle((int)value);
      break;
    }

    case WRITE_SERVO2_ANGLE: {
      float value = readFloat(data, 0);
      writeServo2Angle((int)value);
      break;
    }

    case WRITE_BUZZER: {
      float value = readFloat(data, 0);
      writeBuzzer((int)value);
      break;
    }

    case WRITE_RGB: {
      float r_value = readFloat(data, 0);
      float g_value = readFloat(data, 4);
      float b_value = readFloat(data, 8);
      writeRGB((int)r_value, (int)g_value, (int)b_value);
      break;
    }

    case WRITE_MOTOR_VEL: {
      float wl = readFloat(data, 0);
      float wr = readFloat(data, 4);
      writeMotorVel(wl, wr);
      break;
    }

    case WRITE_MOTOR_PWM: {
      float l_pwm = readFloat(data, 0);
      float r_pwm = readFloat(data, 4);
      writeMotorPwm((int)l_pwm, (int)r_pwm);
      break;
    }

    case WRITE_CMD_VEL: {
      float v = readFloat(data, 0);
      float w = readFloat(data, 4);
      writeCmdVel(v, w);
      break;
    }

    case SET_WHEEL_ODOM_PARAMS: {
      float R_mm = readFloat(data, 0);
      float L_mm = readFloat(data, 4);
      setWheelOdomParams((int)R_mm, (int)L_mm);
      break;
    }

    case GET_WHEEL_ODOM_PARAMS: {
      float R_mm, L_mm;
      getWheelOdomParams(R_mm, L_mm);

      uint8_t tx[8];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &R_mm, sizeof(R_mm)); tx_len += 4;
      memcpy(&tx[tx_len], &L_mm, sizeof(L_mm)); tx_len += 4;

      udpSend(tx, tx_len);
      break;
    }

    // case READ_TOF_SENSOR: {
    //   float res = readTofSensor();
    //   udpSend((uint8_t*)&res, sizeof(res));
    //   break;
    // }

    case READ_SONAR: {
      float res = readSonar();
      udpSend((uint8_t*)&res, sizeof(res));
      break;
    }

    case READ_LINE_SENSOR1: {
      float res = readLineSensor1();
      udpSend((uint8_t*)&res, sizeof(res));
      break;
    }

    case READ_LINE_SENSOR2: {
      float res = readLineSensor2();
      udpSend((uint8_t*)&res, sizeof(res));
      break;
    }

    case READ_MOTOR_STATES: {
      float tl, wl, tr, wr;
      readMotorStates(tl, wl, tr, wr);

      uint8_t tx[16];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &tl, sizeof(tl)); tx_len += 4;
      memcpy(&tx[tx_len], &wl, sizeof(wl)); tx_len += 4;
      memcpy(&tx[tx_len], &tr, sizeof(tr)); tx_len += 4;
      memcpy(&tx[tx_len], &wr, sizeof(wr)); tx_len += 4;

      udpSend(tx, tx_len);
      break;
    }

    case READ_ODOM_DATA: {
      float x, y, t, v, w, d;
      readOdomData(x, y, t, v, w, d);

      uint8_t tx[28];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &x, sizeof(x)); tx_len += 4;
      memcpy(&tx[tx_len], &y, sizeof(y)); tx_len += 4;
      memcpy(&tx[tx_len], &t, sizeof(t)); tx_len += 4;
      memcpy(&tx[tx_len], &v, sizeof(v)); tx_len += 4;
      memcpy(&tx[tx_len], &w, sizeof(w)); tx_len += 4;
      memcpy(&tx[tx_len], &d, sizeof(d)); tx_len += 4;

      udpSend(tx, tx_len);
      break;
    }

    // case READ_ALL_SENSORS: {
    //   float sonar, sonar2, tl, wl, tr, wr, x, y, theta, v, w, dist;
    //   readAllSensors(sonar, sonar2, tl, wl, tr, wr, x, y, theta, v, w, dist);

    //   uint8_t tx[48];
    //   size_t tx_len = 0;
    //   memcpy(&tx[tx_len], &sonar, sizeof(sonar)); tx_len += 4;
    //   memcpy(&tx[tx_len], &sonar2, sizeof(sonar2)); tx_len += 4;
      
    //   memcpy(&tx[tx_len], &tl, sizeof(tl)); tx_len += 4;
    //   memcpy(&tx[tx_len], &wl, sizeof(wl)); tx_len += 4;
    //   memcpy(&tx[tx_len], &tr, sizeof(tr)); tx_len += 4;
    //   memcpy(&tx[tx_len], &wr, sizeof(wr)); tx_len += 4;

    //   memcpy(&tx[tx_len], &x, sizeof(x)); tx_len += 4;
    //   memcpy(&tx[tx_len], &y, sizeof(y)); tx_len += 4;
    //   memcpy(&tx[tx_len], &theta, sizeof(theta)); tx_len += 4;
    //   memcpy(&tx[tx_len], &v, sizeof(v)); tx_len += 4;
    //   memcpy(&tx[tx_len], &w, sizeof(w)); tx_len += 4;
    //   memcpy(&tx[tx_len], &dist, sizeof(dist)); tx_len += 4;

    //   udpSend(tx, tx_len);
    //   break;
    // }

    case READ_ALL_SENSORS: {
      float sonar, line_sensor1, line_sensor2, tl, tr, theta, dist;
      readAllSensors(sonar, line_sensor1, line_sensor2, tl, tr, theta, dist);

      uint8_t tx[28];
      size_t tx_len = 0;
      memcpy(&tx[tx_len], &sonar, sizeof(sonar)); tx_len += 4;

      memcpy(&tx[tx_len], &line_sensor1, sizeof(line_sensor1)); tx_len += 4;
      memcpy(&tx[tx_len], &line_sensor2, sizeof(line_sensor2)); tx_len += 4;
      
      memcpy(&tx[tx_len], &tl, sizeof(tl)); tx_len += 4;
      memcpy(&tx[tx_len], &tr, sizeof(tr)); tx_len += 4;

      memcpy(&tx[tx_len], &theta, sizeof(theta)); tx_len += 4;
      memcpy(&tx[tx_len], &dist, sizeof(dist)); tx_len += 4;

      udpSend(tx, tx_len);
      break;
    }

    case SET_CONTROLLER_CMD_TIMEOUT: {
      float value = readFloat(data, 0);
      setControllerCmdTimeout((int)value);
      break;
    }

    case CLEAR_CONTROLLER_DATA: {
      float res = clearControllerData();
      udpSend((uint8_t*)&res, sizeof(res));
      break;
    }

    case SET_UDP_CONN_TIMEOUT: {
      float value = readFloat(data, 0);
      setUdpConnTimeout((int)value);
      break;
    }

    case GET_UDP_CONN_TIMEOUT: {
      float timeout_ms = getUdpConnTimeout();
      udpSend((uint8_t*)&timeout_ms, sizeof(timeout_ms));
      break;
    }

    case UDP_HEART_BEAT: {
      // float value = readFloat(data, 0);
      udpHeartBeat();
      break;
    }

    default: {
      float error = 0.0;
      udpSend((uint8_t*)&error, sizeof(error));
      break;
    }
  }

  gpio_set_level((gpio_num_t)LED_PIN, 0);
  // udpPingCheck();
}







static inline void recieve_and_send_data() {

  int packetSize = Udp.parsePacket();
  if (!packetSize) return;

  remoteIP = Udp.remoteIP();
  remotePort = Udp.remotePort();

  uint8_t packet[64];
  int len = Udp.read(packet, sizeof(packet));

  static uint8_t state = 0;
  static uint8_t cmd, length;
  static uint8_t buffer[64];
  static uint8_t index = 0;
  static uint8_t checksum = 0;

  for (int i = 0; i < len; i++) {

    yield();   // prevent watchdog reset

    uint8_t b = packet[i];

    switch (state) {

      case 0:
        if (b == START_BYTE) {
          state = 1;
          checksum = b;
        }
        break;

      case 1:
        cmd = b;
        checksum += b;
        state = 2;
        break;

      case 2:
        length = b;

        if (length > sizeof(buffer)) {
          state = 0;
          checksum = 0;
          break;
        }

        checksum += b;
        index = 0;
        state = (length == 0) ? 4 : 3;
        break;

      case 3:
        buffer[index++] = b;
        checksum += b;

        if (index >= length) {
          state = 4;
        }
        break;

      case 4:
        if ((checksum & 0xFF) == b) {
          processCommand(cmd, buffer);
        }
        state = 0;
        break;
    }
  }
}

#endif