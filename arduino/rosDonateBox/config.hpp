#ifndef H_RDB_CONFIG
#define H_RDB_CONFIG

#include <stdint.h>
#include "pins_arduino.h"

namespace config
{
  // Firmware version constants
  static const int fwVersionMajor = 0;
  static const int fwVersionMinor = 4;
  
  // Maximum WiFi SSID length
  static const int maxSSIDLength = 30;
  // Maximum WiFi PSK length
  static const int maxPSKLength = 30;
  
  // WiFi SSID in the AP state
  static const char *APModeSSID = "rosDonateBox";
  // WiFi PSK in the AP state
  static const char *APModePSK = "mufFins4D";
  
  // Maximum command arguments number in a command
  static const unsigned int commandMaxArguments = 3;
  
  // DG600F INHIBIT pin (after the logic level converter)
  static constexpr int inhibitPin = D0;
  
  // DG600F SIGNAL (after logic converter)
  static constexpr int signalPin = D6;
  // DG600F SIGNAL UART RX buffer size
  static const unsigned int signalRxBufferSize = 512;
  
  // SSD1306 display I2C address
  static const uint8_t displayI2CAddress = 0x3C;
  // Display geometry in pixels
  static const int displayWidth = 128;
  static const int displayHeight = 64;
  //
}

#endif
