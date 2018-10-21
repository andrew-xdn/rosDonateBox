#ifndef H_RDB_CONFIG
#define H_RDB_CONFIG

namespace config
{
  // Firmware version constants
  const int fwVersionMajor = 0;
  const int fwVersionMinor = 4;
  
  // Maximum WiFi SSID length
  const int maxSSIDLength = 30;
  // Maximum WiFi PSK length
  const int maxPSKLength = 30;
  
  // WiFi SSID in the AP state
  const char *APModeSSID = "rosDonateBox";
  // WiFi PSK in the AP state
  const char *APModePSK = "mufFins4D";
  
  // Maximum command arguments number in a command
  const unsigned int commandMaxArguments = 3;
  
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
}

#endif
