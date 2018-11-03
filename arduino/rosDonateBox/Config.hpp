#ifndef H_RDB_CONFIG
#define H_RDB_CONFIG

// HINT: pins_arduino.h doesn't have stdint.h include
#include <stdint.h>
#include "pins_arduino.h"

namespace Config
{
  // Firmware version constants
  const int fwVersionMajor = 0;
  const int fwVersionMinor = 8;
  
  // Maximum WiFi SSID length
  const int maxSSIDLength = 30;
  // Maximum WiFi PSK length
  const int maxPSKLength = 30;
  
  // WiFi SSID in the AP state
  const char* const APModeSSID = "rosDonateBox";
  // WiFi PSK in the AP state
  const char* const APModePSK = "mufFins4D";
  
  // DG600F INHIBIT pin (after the logic level converter)
  constexpr int inhibitPin = D0;
  
  // DG600F SIGNAL (after logic converter)
  constexpr int signalPin = D6;
  // DG600F SIGNAL UART RX buffer size
  const unsigned int signalRxBufferSize = 512;
  
  // SSD1306 display I2C address
  const uint8_t displayI2CAddress = 0x3C;
  // Display geometry in pixels
  const int displayWidth = 128;
  const int displayHeight = 64;
  //

  // ROS topic name
  const char* const rosTopicName = "donations";
}

#endif
