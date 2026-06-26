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

    case READ_DATA: {
      float sonar, tof, line_sensor, tl, tr, yaw, dist, color_sensor, wheel_radius_param, wheel_dist_param, max_wheel_speed_param;
      readData(sonar, tof, line_sensor, tl, tr, yaw, dist, color_sensor, wheel_radius_param, wheel_dist_param, max_wheel_speed_param);

      uint8_t tx[44];
      size_t tx_len = 0;

      memcpy(&tx[tx_len], &sonar, sizeof(sonar)); tx_len += 4;
      memcpy(&tx[tx_len], &tof, sizeof(tof)); tx_len += 4;
      memcpy(&tx[tx_len], &line_sensor, sizeof(line_sensor)); tx_len += 4;
      memcpy(&tx[tx_len], &tl, sizeof(tl)); tx_len += 4;
      memcpy(&tx[tx_len], &tr, sizeof(tr)); tx_len += 4;
      memcpy(&tx[tx_len], &yaw, sizeof(yaw)); tx_len += 4;
      memcpy(&tx[tx_len], &dist, sizeof(dist)); tx_len += 4;
      memcpy(&tx[tx_len], &color_sensor, sizeof(color_sensor)); tx_len += 4;
      memcpy(&tx[tx_len], &wheel_radius_param, sizeof(wheel_radius_param)); tx_len += 4;
      memcpy(&tx[tx_len], &wheel_dist_param, sizeof(wheel_dist_param)); tx_len += 4;
      memcpy(&tx[tx_len], &max_wheel_speed_param, sizeof(max_wheel_speed_param)); tx_len += 4;

      udpSend(tx, tx_len);
      break;
    }

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

    case UDP_HEART_BEAT: {
      // float value = readFloat(data, 0);
      udpHeartBeat();
      break;
    }

    case RESET_PARAMS: {
      float res = triggerResetParams();
      udpSend((uint8_t*)&res, sizeof(res));
      break;
    }

    case SET_WHEEL_RADIUS: {
      float value = readFloat(data, 0);
      setWheelRadius(value);
      break;
    }

    case SET_WHEEL_DISTANCE: {
      float value = readFloat(data, 0);
      setWheelDistance(value);
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