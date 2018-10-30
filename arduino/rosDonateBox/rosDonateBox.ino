
// stdlib
#include <limits.h>
#include <math.h>
// EPS8266
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
// rosserial
#include <ros.h>
#include <rosdonatebox_msgs/Donation.h>
// OLED display
#include <brzo_i2c.h>
#include <SSD1306Brzo.h>
// Minimal custom font with currency symbols
#include "fonts/OpenSansPlain40Minimal.h"
// Other
#include <TimeLib.h>
#include <uCRC16Lib.h>

#include "Config.hpp"
#include "DG600FMode3.hpp"
#include "CommandScanner.hpp"
#include "TelnetServer.hpp"

// Device FSM states
enum DeviceState
{
  DS_ERROR = 0,  // A critical error occured (no exit from this state, no device activity)
  DS_INIT,  // Device initialization state
  DS_WIFI_CONNECTING,  // Connecting to the target WiFi network
  DS_WIFI_AP_SETUP,  // Configuring WiFi AP
  // WARNING: No delays from this state!
  DS_TELNET_SETUP,  // Configuring TELNET server
  DS_TELNET_ONLY,  // Only TELNET server avalible (no exit from this state)
  DS_ROSSERIAL_SETUP,  // Configuraing rosserial
  DS_ROSSERIAL_CONNECTING,  // Connecting to the rosserial server
  // WARNING: Display overlay enabled from this state
  DS_ROSSERIAL_READY,  // rosserial ready (main device state, coin acceptor enabled)
  DS_GRATITUDE  // Displaying something nice for a user (coin acceptor enabled)
};

// The device is in the AP mode
// TODO: Replace this flag with a new FSM state (?)
bool wifiApMode;

// Current device state
DeviceState deviceState;

// String currency code constants for every currency in the ROS donation message enum
// HINT: NONE is the exception, this value is used only to display the empty currency by the list command
const char* const currencyCodes[rosdonatebox_msgs::Donation::CUR_LAST]
{
  "-", "USD", "EUR", "RUR"
};

// Font symbols bindings for every currency in the ROS donation message enum
// HINT: NONE character is not used at all, needed just for the correct offset
const char currencyFontBindings[rosdonatebox_msgs::Donation::CUR_LAST]
{
  '\0', '\x24', '\x3C', '\x3D'
};

// Pack the following structure (saves some space on EEPROM)
#pragma pack(push, 1) 
// Device EEPROM record
struct DeviceEeprom
{
  // Target WiFi network SSID (max length + '\0')
  char targetSSID[Config::maxSSIDLength + 1];
  // Target WiFi network PSK (max length + '\0')
  char targetPSK[Config::maxPSKLength + 1];

  // rosserial server IP (integer representation)
  uint32_t rosserialIP;
  // rosserial server port
  uint16_t rosserialPort;
  // Time zone UTC offset (hours * 60 + minutes)
  int16_t timeZoneOffset;

  // Coins value divider (1, 10, 100)
  uint8_t coinValueDivider;
  // Currency according to the ROS donation message enum
  uint8_t donationCurrency;

  // WARNING: Do not place any data fields after the checksum!
  // Checksum (CRC16)
  uint16_t crc16;
};
#pragma pack(pop)
//

// Device EEPROM parameters RAM copy
DeviceEeprom deviceEeprom;

// Default device EEPROM parameters
const DeviceEeprom deviceEepromDefault = 
{
  .targetSSID = { '\0' },  // No target WiFi SSID
  .targetPSK = { '\0' },  // No target WiFi PSK
  .rosserialIP = 0,  // No rosserial IP
  .rosserialPort = 11411,  // No rosserial port
  .timeZoneOffset = 0, // UTC+00:00
  .coinValueDivider = 1,  // No fraction coins are allowed
  .donationCurrency = rosdonatebox_msgs::Donation::CUR_NONE // No currency
  // No checksum
};

// ROS handle
ros::NodeHandle nh;

// Buffer for a sprintf function call ("00:00\0")
constexpr unsigned int timeZoneOffsetBufferSize = 2 * 2 + 1 + 1;

// Parse a TELNET command
//  Arguments:
//    caller - a colling TELNET server,
//    tokens - command tokens array after scanning,
//    tokensNumber - a number of tokens the scanner has found.
void parseCommand(TelnetServer &caller, String (&tokens)[CommandScanner::commandMaxTokens], unsigned int tokensNumber)
{
  // The user has passed an empty tokens array
  if (!tokensNumber)
    return;
  
  // Convert the command to the lower case
  tokens[0].toLowerCase();

  // Exit command and it's alias "quit"
  if ((tokens[0] == "exit") or (tokens[0] == "quit"))
  {
    // No arguments for this command
    if (tokensNumber > 1)
    {
      Serial.printf("[TELNET] Exit command doesn't accept any arguments!\n");
      caller.print("Exit command doesn't accept any arguments!\r\n");
    }
    else
    {
      Serial.printf("[TELNET] Exit command, disconnecting the client...\n");

      // Close the user TELNET session
      caller.disconnectClient();
    }
  }
  // User requested to print all the parameters
  else if (tokens[0] == "list")
  {       
    // No arguments for this command
    if (tokensNumber > 1)
    {
      Serial.printf("[TELNET] List command doesn't accept any arguments!\n");
      caller.print("List command doesn't accept any arguments!\r\n");
    }
    else
    {
      Serial.printf("[TELNET] List command, printing EEPROM parameters...\n");

      // Time zone offset (HH:MM)
      char timeZoneOffsetBuffer[timeZoneOffsetBufferSize];
      // Positive time zone offset
      unsigned int timeZoneOffsetAbs = abs(deviceEeprom.timeZoneOffset);
      // Format tme zone offset
      snprintf(timeZoneOffsetBuffer, sizeof(timeZoneOffsetBuffer), "%02u:%02u", timeZoneOffsetAbs / 60, timeZoneOffsetAbs % 60);

      // Print all device EEPROM parameter using the string class
      caller.print(String("TARGET_SSID = \"") + deviceEeprom.targetSSID + "\"\r\nTARGET_PSK = " +
        ((deviceEeprom.targetPSK[0]) ? "****" : "-") + "\r\nROSSERIAL_IP = " + 
        ((deviceEeprom.rosserialIP) ? IPAddress(deviceEeprom.rosserialIP).toString() : "-") + "\r\nROSSERIAL_PORT = " +
        ((deviceEeprom.rosserialPort) ? String(deviceEeprom.rosserialPort) : "-") + "\r\nTIME_ZONE = UTC" +
        ((deviceEeprom.timeZoneOffset < 0) ? '-' : '+') + timeZoneOffsetBuffer + "\r\nCOIN_DIVIDER = " + 
        deviceEeprom.coinValueDivider + "\r\nCURRENCY = " + currencyCodes[deviceEeprom.donationCurrency] + "\r\n");
    }
  }
  // User wants to set a new value to the parameter
  else if (tokens[0] == "set")
  {
    // The command accepts exactly 2 arguments (+ command itself)
    if (tokensNumber != 3)
    {
      Serial.printf("[TELNET] Set command accepts 2 arguments!\n");
      caller.print("Set command accepts 2 arguments!\r\n");
    }
    else
    {
      // Convert the parameter name to the upper case (it just looks better in the code)
      tokens[1].toUpperCase();
      Serial.printf("[TELNET] Set command parameter: \"%s\"\n", tokens[1].c_str());
      
      Serial.printf("[TELNET] Set command value: \"%s\"\n", tokens[2].c_str());

      // Target WiFi SSID
      if (tokens[1] == "TARGET_SSID")
      {
        // Empty token, user wants to reset the target WiFi SSID
        if (!tokens[2].length())
        {
          // Clean up the C string
          deviceEeprom.targetSSID[0] = '\0';
          
          Serial.printf("[TELNET] Parameter was reset\n");
          return;
        }

        // Checks all the characters in the value
        for (int i = 0; i < tokens[2].length(); i++)
          // The character must be printable
          // WARNING: This check doesn't fully covers the WiFi SSID integrity
          if (!isPrintable(tokens[2][i]))
          {
            Serial.printf("[TELNET] SSID contains invalid characters!\n");
            caller.print("SSID contains invalid characters!\r\n");
            
            return;
          }

        // SSID is short enought to fit into the EEPROM field
        if (tokens[2].length() <= Config::maxSSIDLength)
        {
          // Copy the token contents inside the C string
          strcpy(deviceEeprom.targetSSID, tokens[2].c_str());
          Serial.printf("[TELNET] New parameter value has been set\n");
        }
        // SSID is too long
        else
        {
          Serial.printf("[TELNET] Too long SSID: %u\n", tokens[2].length());
          caller.print(String("SSID can't be longer than ") + Config::maxSSIDLength + "!\r\n");
        }
      }
      // Target WiFi PSK
      else if (tokens[1] == "TARGET_PSK")
      {
        // Empty token, user wants to reset the target WiFi PSK
        if (!tokens[2].length())
        {
          // Clean up the C string
          deviceEeprom.targetPSK[0] = '\0';
          
          Serial.printf("[TELNET] Parameter was reset\n");
          return;
        }

        // Check all the characters in the value
        for (int i = 0; i < tokens[2].length(); i++)
          // The character must be printables
          if (!isPrintable(tokens[2][i]))
          {
            Serial.printf("[TELNET] PSK contains invalid characters!\n");
            caller.print("PSK contains invalid characters!\r\n");
            
            return;
          }

        // PSK is short enought to fit into the EEPROM field
        if (tokens[2].length() <= Config::maxPSKLength)
        {
          // Copy the token contents inside the C string
          strcpy(deviceEeprom.targetPSK, tokens[2].c_str());
          Serial.printf("[TELNET] New parameter value has been set\n");
        }
        // PSK is too long
        else
        {
          Serial.printf("[TELNET] Too long PSK: %u\n", tokens[2].length());
          caller.print(String("PSK can't be longer than ") + Config::maxPSKLength + "!\r\n");
        }
      }
      // rosserial server IP
      else if (tokens[1] == "ROSSERIAL_IP")
      {
        // Empty token, user wants to reset the rosserial server IP
        if (!tokens[2].length())
        {
          // Set it to 0 (0.0.0.0 is not a valid IP for the TCP client)
          deviceEeprom.rosserialIP = 0;
          
          Serial.printf("[TELNET] Parameter was reset\n");
        }
        // It's not an empty token
        else
        {
          // IP adress object for the conversion
          IPAddress rosserialIP;

          // Valid IP address
          if (rosserialIP.fromString(tokens[2]))
          {
            // IP address object has a method to convert itself to uint32_t
            deviceEeprom.rosserialIP = rosserialIP;
            
            Serial.printf("[TELNET] New parameter value has been set\n");
          }
          // Invalid IP address
          else
          {
            Serial.printf("[TELNET] Invalid IP address!\n");
            caller.print("Invalid IP address!\r\n");
          }
        }
      }
      // rosserial server port
      else if (tokens[1] == "ROSSERIAL_PORT")
      {
        // String object can convert itself to long
        long rosserialPort = tokens[2].toInt();

        // Port number has to be > 0 and <= USHRT_MAX
        // HINT: User can't reset the port number parameter
        if ((rosserialPort > 0) && (rosserialPort <= USHRT_MAX))
        {
          deviceEeprom.rosserialPort = static_cast<uint16_t>(rosserialPort);
          Serial.printf("[TELNET] New parameter value has been set\n");
        }
        // Invalid port number
        else
        {
          Serial.printf("[TELNET] Invalid port!\n");
          caller.print("Invalid port!\r\n");
        }
      }
      // UTC time zone offset
      else if (tokens[1] == "TIME_ZONE")
      {
        // Empty token, user wants to reset a UTC time zone offset
        if (!tokens[2].length())
        {
          // UTC+00:00
          deviceEeprom.timeZoneOffset = 0;
          
          Serial.printf("[TELNET] Parameter was reset\n");
          return;
        }
        
        // HINT: Accepts only "[+-][D]D:[D]D" offset format
        
        // Look for ':' in the string
        int timeDelimiterIndex = tokens[2].indexOf(':');

        // No ':' in the string, ":[D][D]", "[D]D:"
        if ((timeDelimiterIndex < 0) || !timeDelimiterIndex || (timeDelimiterIndex == (tokens[2].length() - 1)))
        {
          Serial.printf("[TELNET] Invalid UTC offset!\n");
          caller.print("Invalid UTC offset!\r\n");
        }
        // Looks like a valid offset format
        else
        {
          // Extract the hours part ("[+-][D]D"), last character is not included
          String hoursString = tokens[2].substring(0, timeDelimiterIndex);
          Serial.printf("[TELNET] UTC offset hours token: \"%s\"\n", hoursString.c_str());

          // Hours sign changes minutes parsing behaviour
          bool positiveOffset;

          // It's a zero value with a positive sign
          // HINT: String object returns 0 if integer has an invalid format
          if ((hoursString == "0") || (hoursString == "00") || (hoursString == "+0")
            || (hoursString == "+00"))
          {
            // Positive offset
            positiveOffset = true;
            // Zero hours
            deviceEeprom.timeZoneOffset = 0;
          }
          // It's a zero value with a negative sign
          // HINT: String object returns 0 if integer has an invalid format
          else if ((hoursString == "-0") || (hoursString == "-00"))
          {
            // Negative offset
            positiveOffset = false;
            // Zero hours
            deviceEeprom.timeZoneOffset = 0;
          }
          // It's not a zero value
          else
          {
            // String object can convert itself to long
            long hours = hoursString.toInt();
  
            // A valid integer
            if (hours)
            {
              // Time zone offset hours integrity check
              if ((hours < -12) || (hours > 14))
              {
                Serial.printf("[TELNET] Invalid UTC offset hours!\n");
                caller.print("Invalid UTC offset hours!\r\n");
                return;
              }
  
              // Check the hours sign
              positiveOffset = hours >= 0; 
  
              // Calculate a time zone offset by hours
              deviceEeprom.timeZoneOffset = hours * 60;
            }
            // It's an invalid integer
            else
            {
              Serial.printf("[TELNET] Invalid UTC offset hours!\n");
              caller.print("Invalid UTC offset hours!\r\n");
              return;
            }
          }

          // Extract the minutes part ("[D]D"), last character is not included
          String minutesString = tokens[2].substring(timeDelimiterIndex + 1, tokens[2].length());
          Serial.printf("[TELNET] UTC offset minutes token: \"%s\"\n", minutesString.c_str());

          // It's not a zero value
          // HINT: String object returns 0 if integer has an invalid format
          if ((minutesString != "0") && (minutesString != "00"))
          {
            // String object can convert itself to long
            long minutes = minutesString.toInt();
  
            // A valid integer
            if (minutes)
            {
              // Time zone offset minutes integrity check
              if ((minutes < 0) || (minutes > 59))
              {
                Serial.printf("[TELNET] Invalid UTC offset minutes!\n");
                caller.print("Invalid UTC offset minutes!\r\n");
                return;
              }
  
              // Positive hours value
              if (positiveOffset)
                deviceEeprom.timeZoneOffset += minutes;
              // Negative hours value
              else
                deviceEeprom.timeZoneOffset -= minutes;
            }
            // It's an invalid integer
            else
            {
              Serial.printf("[TELNET] Invalid UTC offset minutes!\n");
              caller.print("Invalid UTC offset minutes!\r\n");
              return;
            }
          }
          // HINT: No need to handle minutes zero value
          Serial.printf("[TELNET] New parameter value has been set\n");
        }
      }
      // Coin value divider
      else if (tokens[1] == "COIN_DIVIDER")
      {
        // String object can convert itself to long
        long coinValueDivider = tokens[2].toInt();

        // Coin value divider can be 1, 10 or 100
        if ((coinValueDivider == 1) or (coinValueDivider == 10) or (coinValueDivider == 100))
        {
          deviceEeprom.coinValueDivider = static_cast<uint8_t>(coinValueDivider);
          Serial.printf("[TELNET] New parameter value has been set\n");
        }
        // Invalid coin value divider
        else
        {
          Serial.printf("[TELNET] Invalid coin value divider!\n");
          caller.print("Invalid coin value divider!\r\n");
        }
      }
      // Donation currency
      else if (tokens[1] == "CURRENCY")
      {
        // Empty token, user wants to reset currency
        if (!tokens[2].length())
        {
          // Clean up the C string
          deviceEeprom.donationCurrency = rosdonatebox_msgs::Donation::CUR_NONE;
          
          Serial.printf("[TELNET] Parameter was reset\n");
          return;
        }
        
        // Convert the currency code to the upper case (it just looks better in the code)
        tokens[2].toUpperCase();

        Serial.printf("[TELNET] Currency code: \"%s\"\n", tokens[2].c_str());

        // Check every code in the currency codes (except NONE, it was handled above)
        for (uint8_t i = rosdonatebox_msgs::Donation::CUR_NONE + 1; i < rosdonatebox_msgs::Donation::CUR_LAST; i++)
        {
          // The currency codes match
          if (tokens[2] == currencyCodes[i])
          {
            // Index is the correct currency value
            deviceEeprom.donationCurrency = i;
            Serial.printf("[TELNET] Currency index: %u\n", deviceEeprom.donationCurrency);
            
            Serial.printf("[TELNET] New parameter value has been set\n");
            return;
          }
        }
        // Unknown currency code

        Serial.printf("[TELNET] Invalid currency code!\n");
        caller.print("Invalid currency code!\r\n");
      }
      // HINT: You can handle new parameter here
      // Unknown parameter
      else
      {
        Serial.printf("[TELNET] Unknown parameter!\n");
        caller.print("Unknown parameter!\r\n");
      }
    }
  }
  // User requested to save parameters to EEPROM
  else if (tokens[0] == "save")
  {
    // No arguments for this command
    if (tokensNumber > 1)
    {
      Serial.printf("[TELNET] Save command doesn't accept any arguments!\n");
      caller.print("Save command doesn't accept any arguments!\r\n");
    }
    else
    {
      Serial.printf("[TELNET] Saving to EEPROM...\n");
      caller.print("Saving to EEPROM...");

      // Prepare a pointer for the device EEPROM structure in RAM
      uint8_t *pDeviceEeprom = reinterpret_cast<uint8_t *>(&deviceEeprom);

      // Calculate an actual CRC16 checksum for the EEPROM structure in RAM
      deviceEeprom.crc16 = uCRC16Lib::calculate(reinterpret_cast<char *>(pDeviceEeprom), sizeof(DeviceEeprom) - sizeof(uint16_t));

      // Reset EEPROM library
      EEPROM.begin(sizeof(DeviceEeprom));

      // For every byte in the device EEPROM structure in RAM
      for (int i = 0; i < sizeof(DeviceEeprom); i++)
        // Write to the EEPROM buffer
        EEPROM.write(i, pDeviceEeprom[i]);

      // Actually write data to EEPROM (it's EEPROM emulation, ESP8266 doesn't have one)
      EEPROM.commit();

      Serial.printf("[TELNET] Saving to EEPROM has been completed\n");
      caller.print(" done\r\n");
    }
  }
  // User requested to reboot the device
  else if (tokens[0] == "reboot")
  {
    // No arguments for this command
    if (tokensNumber > 1)
    {
      Serial.printf("[TELNET] Reboot command doesn't accept any arguments!\n");
      caller.print("Reboot command doesn't accept any arguments!\r\n");
    }
    else
    {
      // HINT: It's a tricky operation. ESP8266 library allows to reboot the device but this operation doesn't work on all hardware
      // plaforms without a valid BOOT pins setup.
      // HINT: To prevent platform dependent problems we can initiate a virtual reboot by resetting all the objects and setting FSM to
      // the INIT state.
      
      Serial.printf("[TELNET] Reboot command, rebooting...\n");
      // Stop accepting coins
      enableCoinAcceptance(false);
      // Close the TELNET-session if any
      caller.disconnectClient();
      // telnetServer.stop(); There is another close() call at the begin() method. Duplicate stop() call causes server to not start.
      // Give some time for TCP connections to close before WiFi disconnect
      delay(100);
      // WiFi AP mode
      if (wifiApMode)
        // Stop the AP
        WiFi.softAPdisconnect(true);
      // WiFi STA mode
      else
        // Disconnect from the target SSID
        WiFi.disconnect(true);

      // Reset the device FSM
      deviceState = DS_INIT;
    }
  }
  // User requested to print help
  else if (tokens[0] == "help")
  {
    // No arguments for this command
    if (tokensNumber > 1)
    {
      Serial.printf("[TELNET] Help command doesn't accept any arguments!\n");
      caller.print("Help command doesn't accept any arguments!\r\n");
    }
    else
    {
      Serial.printf("[TELNET] Help command, printing help...\n");
      // Print the help
      caller.print(
        "help - prints this output;\r\nlist - prints parameters;\r\n"
        "set <parameter> <value> - sets a new value to the parameter:\r\n"
        "\tTARGET_SSID - a target WiFi SSID;\r\n"
        "\tTARGET_PSK - a target WiFi passhrase;\r\n"
        "\tROS_IP - a target ROS system IP address;\r\n"
        "\tROS_PORT - a target ROS system port.\r\n"
        "\tTIME_ZONE - an UTC time zone offset (set in format +-00:00)"
        "\tCOIN_DIVIDER - divide coin value by this number (1, 10 or 100).\r\n"
        "\tCURRENCY - a donation currency (USD, EUR, RUR)."
        "save - save parameters from RAM to EEPROM;\r\n"
        "reboot - reboot device to apply new parameters or reconnect to the target network;\r\n"
        "exit or quit - exit from the termial.\r\n"
      );
    }
  }
  // HINT: You can implement new command here
  // Unknown command
  else
  {
    Serial.printf("[TELNET] Unknown command!\n");
    caller.print("Unknown command!\r\n");
  }
}

// TELNET server for device configuration
TelnetServer telnetServer("rosDonateBox terminal", "> ", parseCommand);

// Switch DG600F coin acceptance mode
//  enable - accept coins (true); return coins (false).
void enableCoinAcceptance(bool enable)
{
  Serial.printf("[COIN_ACCEPTANCE] %s\n", (enable) ? "Enabled" : "Disabled");
  digitalWrite(Config::inhibitPin, (enable) ? HIGH : LOW);
}

// Software serial object (RX only) connected to DG600F SIGNAL (after logic converter)
// HINT: ESP8266 with Arduino doesn't have any free hardware serial RX. Software serial isn't a good
// solution, but due to a high ESP8266 clock frequency and low DG600F SIGNAL baudrate (up to 9600) it works fine.
SoftwareSerial DG600FSerial(Config::signalPin, SW_SERIAL_UNUSED_PIN, false, Config::signalRxBufferSize);

// DG600F mode 3 driver
DG600FMode3 coinAcceptor(DG600FSerial);

// A donation ROS message
rosdonatebox_msgs::Donation donationMessage;
// A ROS donation publisher
ros::Publisher donations(Config::rosTopicName, &donationMessage);

// Donation timeout timer
Ticker donationTimeoutTicker;
volatile bool donationTimeoutExpired;

void donationTimeoutCallback()
{
  donationTimeoutExpired = true;
}
//

// Donation message stamped flag
// HINT: Helps to avoid stamp updates for the already stamped message that
// hasn't been sent from the first attempt
bool donationStamped;

// Display object
SSD1306Brzo display(Config::displayI2CAddress, SDA, SCL);

// Font heights constants to calculate frame geometry
// HINT: It doesn't look like constexpr can calculate PROGMEM read requests
const unsigned int fontArialMTPlain24Height = 28;
const unsigned int fontArialMTPlain16Height = 19;
const unsigned int fontArialMTPlain10Height = 13;
const unsigned int fontOpenSansRegular40MinimalHeight = 55;
//

// Display the project logo
void displayLogo()
{
  display.clear();
  // Calculate the Y axis offset to center the frame contents vertically
  constexpr unsigned int yOffset = (Config::displayHeight - (fontArialMTPlain16Height + 3 + fontArialMTPlain16Height + 2 + fontArialMTPlain10Height)) / 2;
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  // Project logo
  String logo("ROS Donate Box");
  uint16_t width = display.getStringWidth(logo);
  display.drawString(Config::displayWidth / 2, yOffset, logo);
  //
  display.drawHorizontalLine(0, yOffset + fontArialMTPlain16Height + 1, Config::displayWidth);
  // Firmware version string
  String versionString(String('v') + Config::fwVersionMajor + '.' + Config::fwVersionMinor);
  width = display.getStringWidth(versionString);
  display.drawString(Config::displayWidth / 2, yOffset + fontArialMTPlain16Height + 3, versionString);
  //
  // Add a catch phrase to fill some space at the bottom
  String catchPhrase("Robots need money too...");
  display.setFont(ArialMT_Plain_10);
  display.drawString(Config::displayWidth / 2, yOffset + fontArialMTPlain16Height + 3 + fontArialMTPlain16Height + 2, catchPhrase);
  //
  display.display();
}

// Display "connecting to" frame
// Arguments:
//  target - a connection target string.
void displayConnectingFrame(String target)
{
  display.clear();
  // Calculate the Y axis offset to center the frame contents vertically
  constexpr unsigned int yOffset = (Config::displayHeight - (3 + fontArialMTPlain16Height + fontArialMTPlain16Height + 2)) / 2;
  // Top line
  display.drawHorizontalLine(0, yOffset, Config::displayWidth);
  // "Connecting to" header
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  String header("Connecting to");
  uint16_t width = display.getStringWidth(header);
  display.drawString(Config::displayWidth / 2, yOffset + 3, header);
  // Connection target text with a postfix
  String targetWithPostfix(target + "...");
  width = display.getStringWidth(targetWithPostfix);
  display.drawString(Config::displayWidth / 2, yOffset + 3 + fontArialMTPlain16Height, targetWithPostfix);
  // Bottom line
  display.drawHorizontalLine(0, yOffset + 3 + fontArialMTPlain16Height + fontArialMTPlain16Height + 2, Config::displayWidth);
  //
  display.display();
}

// Displays AP mode frame
void displayAPModeFrame()
{
  display.clear();
  // Calculate the Y axis offset to center the frame contents vertically
  constexpr unsigned int yOffset = (Config::displayHeight - (fontArialMTPlain16Height + 5 + fontArialMTPlain10Height + fontArialMTPlain10Height)) / 2;
  // Header
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  String header = "SETUP MODE";
  uint16_t width = display.getStringWidth(header);
  display.drawString(Config::displayWidth / 2, yOffset, header);
  // Horizontal line to separate the header from connection information
  display.drawLine(0, yOffset + 19 + 3, Config::displayWidth, yOffset + 19 + 3);
  // AP SSID
  display.setFont(ArialMT_Plain_10);
  String ssidInfo(String("SSID: \"") + Config::APModeSSID + '"');
  width = display.getStringWidth(ssidInfo);
  display.drawString(Config::displayWidth / 2, yOffset + fontArialMTPlain16Height + 5, ssidInfo);
  // Device IP address
  String ipInfo("IP: " + WiFi.softAPIP().toString());
  width = display.getStringWidth(ipInfo);
  display.drawString(Config::displayWidth / 2, yOffset + fontArialMTPlain16Height + 5 + fontArialMTPlain10Height, ipInfo);
  //
  display.display();
}

// Clock ready timer
// HINT: There is no way to determine the end of rosserial clock synchronization process
Ticker clockReadyTimeoutTicker;
volatile bool clockReadyTimeout;

void clockReadyTimeoutCallback()
{
  clockReadyTimeout = true;
}
//

// Buffer for a sprintf function call ("00:00:00\0")
constexpr unsigned int clockBufferSize = 2 * 3 + 1 * 2 + 1;

// Device overlay height
constexpr unsigned int overlayHeight = fontArialMTPlain10Height;

// Draw the device overlay with clock and the last IP address octet
void drawOverlay()
{
  // Clock or placeholder on the left side
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  // Clock is ready
  if (clockReadyTimeout)
  {
    // Retrieve the ROS time and convert to a local time
    // WARNING: time_t is platform dependent
    time_t currentTime = nh.now().toSec() + deviceEeprom.timeZoneOffset * 60;
    // Clock (HH:MM:SS)
    char clockTextBuffer[clockBufferSize];
    snprintf(clockTextBuffer, sizeof(clockTextBuffer), "%02u:%02u:%02u", hour(currentTime), minute(currentTime), second(currentTime));
    display.drawString(0, 0, clockTextBuffer);
  }
  // Clock is not ready
  else
    // Print "SYNC" instead
    display.drawString(0, 0, "SYNC");
  //
  // The last IP address octet on the right side
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  String IPLastOctet(WiFi.localIP()[3]);
  display.drawString(Config::displayWidth, 0, IPLastOctet);
}

// Display the donation frame
void displayDonationFrame()
{
  display.clear();
  // Calculate the Y axis offset to center the frame contents vertically
  constexpr unsigned int yOffset = overlayHeight + (Config::displayHeight - (overlayHeight + fontOpenSansRegular40MinimalHeight)) / 2;
  // Draw the overlay
  drawOverlay();
  // The donation has started
  if (donationMessage.sum)
  {
    // Donation sum
    display.setFont(OpenSans_Regular_40_Minimal);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    // Donation sum string format depends on the coin value divider
    String donationSum = (deviceEeprom.coinValueDivider > 1) ?
      String(donationMessage.sum, static_cast<unsigned int>(log10(deviceEeprom.coinValueDivider))) :
      String(static_cast<unsigned int>(donationMessage.sum));
    // Donation sum format also depends on the selected currency
    display.drawString(Config::displayWidth / 2, yOffset,
      (deviceEeprom.donationCurrency == rosdonatebox_msgs::Donation::CUR_NONE) ? donationSum
      : donationSum + ' ' + currencyFontBindings[deviceEeprom.donationCurrency]);
  }
  //
  display.display();
}

// Displays the gratitude frame
void displayGratitudeFrame()
{
  display.clear();
  // Calculate the Y axis offset to center the frame contents vertically
  constexpr unsigned int yOffset = overlayHeight + (Config::displayHeight - (overlayHeight + fontArialMTPlain24Height)) / 2;
  // Draw the overlay
  drawOverlay();
  // Gratitude
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  String donationSum("Thank you!");
  display.drawString(Config::displayWidth / 2, yOffset, donationSum);
  //
  display.display();
}

// Clear the display
void clearDisplay()
{
  display.clear();
  display.display();
}

// Clock update timer
Ticker clockUpdateTicker;
volatile bool clockUpdate;

void clockUpdateCallback()
{
  clockUpdate = true;
}
//

void setup()
{
  // Debug serial setup
  Serial.begin(115200);
  // DG600F SIGNAL pin pull-up setup
  // HINT: The pin direction has been set during the SoftwareSerial object construction
  digitalWrite(Config::signalPin, HIGH);
  // DG600F SIGNAL UART setup
  DG600FSerial.begin(9600);
  // DG600F INHIBIT pin setup
  pinMode(Config::inhibitPin, OUTPUT);
  
  //Serial.setDebugOutput(true);
  Serial.printf("ROS Donate Box v%u.%u\n", Config::fwVersionMajor, Config::fwVersionMinor);

  // Disable coins acceptance
  enableCoinAcceptance(false);
  
  // Reset all WiFi modes
  // HINT: ESP8266 may not reset its WiFi mode even after hardware reset providing a strange bugs
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);

  // Conect a ROS donation publisher to the ROS node handle
  nh.advertise(donations);

  // Set the device FSM to initialisation state
  deviceState = DS_INIT;
}

// User gratitude timer
Ticker gratitudeTimeoutTicker;
volatile bool gratitudeTimeoutExpired;

void gratitudeTimeoutCallback()
{
  gratitudeTimeoutExpired = true;
}
//

// Target WiFi connection time
const unsigned int targetWiFiTimeout = 15000;
// Delay between the next target WiFi connection check attempt
const unsigned int targetWiFiCheckInterval = 500;

// Time to wait for the ROS clock synchronization
const unsigned int rosserialClockSyncTime = 2;

// Clock update interval
const unsigned int clockUpdateTime = 1;

void loop()
{
  // Device FSM
  switch (deviceState)
  {
    // Device initialisation
    case DS_INIT:
      Serial.printf("[STATE] INIT\n");

      // Reset ROS donation message fields
      // HINT: ROS message sequence keeps increasing during the device work session
      donationMessage.header.seq = 0;
      // HINT: It may contain actual sum that hasn't been published. Can't reset from the other states.
      donationMessage.sum = 0;

      // Reset the donation timer
      // HINT: The donation can be interruped by a connection loss. Can't reset from the other states.
      donationTimeoutExpired = false;
      // Reset donation stamped flag
      // HINT: The donation publication may fail due to the connection loss. Can't reset from the other states.
      donationStamped = false;

      Serial.printf("[INIT] Init display\n");
      display.init();

      Serial.printf("[INIT] Display test\n");
      // Display test sequence
      display.fillRect(0, 0, Config::displayWidth, Config::displayHeight);
      display.display();
      delay(1000);
      display.clear();
      display.display();
      //

      Serial.printf("[INIT] Display logo\n");
      displayLogo();
      delay(5000);
    
      Serial.printf("[INIT] Reading EEPROM...\n");
      
      {
        // Prepare a pointer for the device EEPROM structure in RAM
        uint8_t *pDeviceEeprom = reinterpret_cast<uint8_t *>(&deviceEeprom);

        // Reset EEPROM library
        EEPROM.begin(sizeof(deviceEeprom));

        // For every byte in the device EEPROM structure in EEPROM
        for (int i = 0; i < sizeof(DeviceEeprom); i++)
          // Read a byte from EEPROM
          pDeviceEeprom[i] = EEPROM.read(i);

        // Invalid CRC16 checksum
        if (uCRC16Lib::calculate(reinterpret_cast<char *>(pDeviceEeprom), sizeof(DeviceEeprom) - sizeof(uint16_t)) != deviceEeprom.crc16)
        {
          Serial.printf("[EEPROM] Failed. Loading defaults...\n");

          // Copy the default values to the device EEPROM structure in RAM
          memcpy(pDeviceEeprom, &deviceEepromDefault, sizeof(DeviceEeprom));
        }
        // Valid CRC16 checksum
        else
        {
          Serial.printf("[EEPROM] OK\n");

          // To prevent undefined behaviour if some currency support has been removed
          if (deviceEeprom.donationCurrency >= rosdonatebox_msgs::Donation::CUR_LAST)
          {
            Serial.printf("[EEPROM] Invalid currency value in the valid EEPROM structure!\n");

            // Reset the currency value
            deviceEeprom.donationCurrency = rosdonatebox_msgs::Donation::CUR_NONE;
          }
        }
      }

      // Going to the WiFi connecting state
      deviceState = DS_WIFI_CONNECTING;
      break;
    // Connecting to the target SSID
    case DS_WIFI_CONNECTING:
      Serial.printf("[STATE] WIFI_CONNECTING\n");
      displayConnectingFrame("WiFi");

      // Non-empty SSID and PSK have been set in the device EEPROM
      if (deviceEeprom.targetSSID[0] && deviceEeprom.targetPSK[0])
      {
        Serial.printf("[WIFI] Connecting to \"%s\"...\n", deviceEeprom.targetSSID);

        // Enable STA WiFi mode
        WiFi.begin(deviceEeprom.targetSSID, deviceEeprom.targetPSK);

        // Connection check attempt
        int i = 0;
        // Connection timeout flag
        bool timeout = false;

        while (WiFi.status() != WL_CONNECTED)
        {
          // Wait for some time before the next connection state check
          delay(targetWiFiCheckInterval);

          // Connection attempts were unsuccessful too many times 
          if (i == targetWiFiTimeout / targetWiFiCheckInterval)
          {
            // Set the timeout flag
            timeout = true;
            
            break;
          }
          
          i++;
        }

        // Connection timeout
        if (timeout)
        {
          // Turn off STA mode
          WiFi.disconnect(true);
          Serial.printf("[WIFI] Connection failed! Starting AP...\n");

          // Going ti the WiFi AP setup state
          deviceState = DS_WIFI_AP_SETUP;
          break;
        }
        // Connection attempt was successful
        else
        {
          // Reset the AP mode flag
          wifiApMode = false;
          
          Serial.printf("[WIFI] Connected! Device IP: %s\n", WiFi.localIP().toString().c_str());

          // Going to the TELNET setup state
          deviceState = DS_TELNET_SETUP;
        }
      }
      // User hasn't set target WiFi parameters
      else
      {
        Serial.printf("[WIFI] No target SSID. Starting AP...\n");

        // Going to the AP setup state
        deviceState = DS_WIFI_AP_SETUP;
      }
      break;
    // Configuring WiFi AP mode
    case DS_WIFI_AP_SETUP:
      Serial.printf("[STATE] WIFI_AP_SETUP\n");

      // Enable WiFi AP mode
      if (!WiFi.softAP(Config::APModeSSID, Config::APModePSK))
      {
        Serial.printf("[WIFI] AP setup failed.\n");

        Serial.printf("[STATE] ERROR\n");
        // AP setup mode failed, it's a serious problem
        deviceState = DS_ERROR;
        break;
      }

      Serial.printf("[WIFI] AP ready. Device IP: %s.\n", WiFi.softAPIP().toString().c_str());
      displayAPModeFrame();

      // Set AP mode flag
      wifiApMode = true;

      // Going to the TELNET setup state
      deviceState = DS_TELNET_SETUP;
      break;
    // Configuring the TELNET-server
    case DS_TELNET_SETUP:
      Serial.printf("[STATE] TELNET_SETUP\n");
      // Start the server
      telnetServer.begin();
      Serial.printf("[TELNET] Server ready\n");

      // WiFi AP mode
      if (wifiApMode)
      {
        Serial.printf("[TELNET] rosserial is disabled in the AP mode\n");
        Serial.printf("[STATE] TELNET_ONLY\n");
        // Going to the TELNET-only state
        deviceState = DS_TELNET_ONLY;
      }
      else
        // Going to the rosserial setup state
        deviceState = DS_ROSSERIAL_SETUP;
      break;
    // TELNET-only state
    // HINT: The device can leave this state only after reboot
    case DS_TELNET_ONLY:
      break;
    // Configuring rosserial
    case DS_ROSSERIAL_SETUP:
      Serial.printf("[STATE] ROSSERIAL_SETUP\n");
      displayConnectingFrame("rosserial");

      // User has set rosserial server parameters
      if (deviceEeprom.rosserialIP && deviceEeprom.rosserialPort)
      {    
        // Create an IPAddress object from the integer IP address representation
        // HINT: setConnection() call expects a reference to an IPAddress object
        IPAddress rosserialIP(deviceEeprom.rosserialIP);
        // Set the target rosserial server parameters
        nh.getHardware()->setConnection(rosserialIP, deviceEeprom.rosserialPort);
        Serial.printf("[ROSSERIAL] Server at %s:%u\n", rosserialIP.toString().c_str(), deviceEeprom.rosserialPort);
        // Initialise a ROS node
        nh.initNode();
        // Limit maximum spin time
        nh.setSpinTimeout(300);
        Serial.printf("[ROSSERIAL] ROS node ready\n");
        Serial.printf("[ROSSERIAL] Connecting to server...\n");

        Serial.printf("[STATE] ROSSERIAL_CONNECTING\n");
        // Going to the rosserial connecting state
        deviceState = DS_ROSSERIAL_CONNECTING;
      }
      // User hasn't set rosserial server parameters
      else
      {
        Serial.printf("[ROSSERIAL] No rosserial IP or port!\n");
        
        Serial.printf("[STATE] TELNET_ONLY\n");
        // Going to the TELNET-only state
        deviceState = DS_TELNET_ONLY;
      }
      break;
    // Connecting to the rosserial server
    case DS_ROSSERIAL_CONNECTING:
      // Node is connected to the remote ROS server
      if (nh.connected())
      {
        Serial.printf("[ROSSERIAL] Connected to rosserial server\n");
        
        Serial.printf("[STATE] ROSSERIAL_READY\n");
        // Now we can accept coins
        enableCoinAcceptance(true);

        // Start the clock ready timer
        // HINT: No other way to determine the end of the clock synchronisation process ends
        clockReadyTimeoutTicker.once(rosserialClockSyncTime, clockReadyTimeoutCallback);
        Serial.printf("[GUI] Clock enable timer has been started\n");

        // Reset the clock ready flag
        clockReadyTimeout = false;

        // Start the clock update timer
        clockUpdateTicker.attach(clockUpdateTime, clockUpdateCallback);
        Serial.printf("[GUI] Clock update timer has been started\n");

        // Reset the clock update flag
        clockUpdate = false;

        displayDonationFrame();

        Serial.printf("[STATE] ROSSERIAL_READY\n");
        // Going to the rosserial ready state
        deviceState = DS_ROSSERIAL_READY;
      }
      break;
    // rosserial server is ready
    case DS_ROSSERIAL_READY:  
      // The ROS node is dosconnected from the rosserial server
      if (!nh.connected())
      {
        Serial.printf("[ROSSERIAL] Disconnected from rosserial server\n");

        // Stop accepting coins from
        enableCoinAcceptance(false);

        // Stop clock update timer
        // TODO: It's not really important. Still, it's nice to stop regular timers...
        clockUpdateTicker.detach();
        Serial.printf("[GUI] Clock update timer has been stopped\n");
        
        Serial.printf("[ROSSERIAL] Connecting to rosserial server...\n");
        displayConnectingFrame("rosserial");
        
        Serial.printf("[STATE] ROSSERIAL_CONNECTING\n");
        // Going to the rosserial connecting state
        deviceState = DS_ROSSERIAL_CONNECTING;
      }
      break;
    // Device user gratitude
    case DS_GRATITUDE:
      // The gratitude timeout has been expired
      if (gratitudeTimeoutExpired)
      {
        // Reset the gratitude timeout flag
        gratitudeTimeoutExpired = false;

        displayDonationFrame();

        Serial.printf("[STATE] ROSSERIAL_READY\n");
        // Going to the rosserial ready state
        deviceState = DS_ROSSERIAL_READY;
      }
      break;
    // Device error, no way from this state
    case DS_ERROR:
      break;
  }

  // After the TELNET server setup
  // TODO: Integrate this branch into FSM (?)
  if (deviceState >= DS_TELNET_SETUP)
    // WiFi AP mode or the target WiFi network is connected
    if (wifiApMode || WiFi.isConnected())
    {
      // Handle the TELNET server data
      telnetServer.handle();

      // After rosserial setup
      if (deviceState >= DS_ROSSERIAL_SETUP)
      {
        // Handle the rosserial node data
        if (nh.spinOnce() == ros::SPIN_TIMEOUT)
          Serial.printf("[ROSSERIAL] Timeout!\n");

        // It's time to update the clock
        if (clockUpdate)
        {
          // Reset the clock update flag
          clockUpdate = false;

          // Redraw the frame according to the state
          switch(deviceState)
          {
            case DS_ROSSERIAL_READY:
              displayDonationFrame();
              break;
            case DS_GRATITUDE:
              displayGratitudeFrame();
              break;
            default:
              break;
          }
        }

        // The previously started donation has ended
        // HINT: We can get an unprocessed donation after the reconnection here
        if (donationTimeoutExpired)
        {
          // HINT: ESP8266 printf doesn't support %f
          Serial.printf("[DONATION] Total donation sum: %s!\n", String(donationMessage.sum, 2).c_str());

          // The donation hasn't been stamped
          if (!donationStamped)
          {
            // Stamp the ROS time
            donationMessage.header.stamp = nh.now();
            // Update the donation currency
            // HINT: In case it has been changed from the TELNET server
            donationMessage.currency = deviceEeprom.donationCurrency;
            // Set the donation stamped flag
            donationStamped = true;
            // Convert the ROS time to the UNIX time to print the debug output
            // WARNING: time_t is platform dependent
            time_t stampUnix = donationMessage.header.stamp.toSec();
            Serial.printf("[DONATION] Donation has been stamped: %02u:%02u\n", hour(stampUnix), minute(stampUnix));
          }

          // Publish the donation message
          if (donations.publish(&donationMessage) > 0)
          {
            Serial.printf("[ROSSERIAL] Donation has been published\n");

            // Reset the donation timeout flag
            donationTimeoutExpired = false;
            // Reset  the donation stamped flag
            donationStamped = false;

            // Reset the donation sum
            donationMessage.sum = 0;
            // Increase the ROS message sequence number
            donationMessage.header.seq++;

            // Start the gratitude timeout timer
            gratitudeTimeoutTicker.once(3, gratitudeTimeoutCallback);
            
            // Reset the gratitude timeout flag
            gratitudeTimeoutExpired = false;

            displayGratitudeFrame();

            Serial.printf("[STATE] GRATITUDE\n");
            // Going to the user gratitude state
            deviceState = DS_GRATITUDE;
          }
          // Failed to publish the message
          else
          {
            Serial.printf("[ROSSERIAL] Failed to published donation\n");
            // HINT: The new loop iteration will handle the failure reason and will try to send it again
            return;
          }
        }

        static int coinValue;

        // Try to parse a frame
        coinValue = coinAcceptor.read();

        // The frame has been parsed
        while (coinValue >= 0)
        {
          Serial.printf("[DONATION] Added new coin: %u!\n", static_cast<unsigned int>(coinValue));

          // Apply a coin value divider
          float coinValueFraction = static_cast<float>(coinValue) / deviceEeprom.coinValueDivider;
          Serial.printf("[DONATION] Updated coin value: %s!\n", String(coinValueFraction, 2).c_str());

          // It's a new donation
          if (!donationMessage.sum)
            Serial.printf("[DONATION] Registered new donation with ID: %u\n", donationMessage.header.seq);

          // Increase the donation sum by the new coin sum
          donationMessage.sum += coinValueFraction;
          // HINT: ESP8266 printf doesn't support %f
          Serial.printf("[DONATION] Current donation sum: %s!\n", String(donationMessage.sum, 2).c_str());

          // Start the donation timeout timer
          // HINT: No need to stop the timer, it stops automatically on the new start
          donationTimeoutTicker.once(5, donationTimeoutCallback);
          Serial.printf("[DONATION] Donation completion timer has been started\n");

          // Reset the donation timeout flag
          donationTimeoutExpired = false;

          // It's a gratitude state
          if (deviceState == DS_GRATITUDE)
          {
            Serial.printf("[STATE] ROSSERIAL_READY\n");
            // Going to the rosserial ready state 
            deviceState = DS_ROSSERIAL_READY;
          }
            
          displayDonationFrame();

          // Try to get the frame again
          coinValue = coinAcceptor.read();
        }
      }
    }
    // Device has been disconnected from the target WiFi network
    else
    {
      Serial.printf("[WIFI] Disconnected from the target WiFi!\n");
      // server.stop(); There is another close() at the begin() method. Duplicate stop() call causes server doesn't start
      // Stop the coin acceptance
      enableCoinAcceptance(false);
      
      // Stop the clock update timer
      // TODO: It's not really important. Still, it's nice to stop regular timers...
      clockUpdateTicker.detach();
      Serial.printf("[GUI] Clock update timer has been stopped\n");

      Serial.printf("[STATE] WIFI_CONNECTING\n");
      // Going to the WiFi connecting state
      deviceState = DS_WIFI_CONNECTING;
    }
}
