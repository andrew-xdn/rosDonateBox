#ifndef H_TELNET_SERVER
#define H_TELNET_SERVER

#include <ESP8266WiFi.h>

#include "CommandScanner.hpp"

// Specify a debug printf function or comment out to disable debug output
#define TELNET_SERVER_DEBUG(...)  Serial.printf(__VA_ARGS__)

#ifndef TELNET_SERVER_DEBUG
// Remove all debug calls
#define TELNET_SERVER_DEBUG(...)
#endif // COMMAND_SCANNER_DEBUG

// TELNET server states
enum TelnetServerState
{
  TSS_RECIEVE = 0,  // TELNET server recieves the user input
  TSS_COMMAND,  // TELNET server waits for the NVT command
  TSS_OPTION  // TELNET server waits for the NVT option
};

// TELNET commands
enum TelnetCommand
{
  TELNET_INTERRUPT = 244,
  TELNET_WILL = 251,
  TELNET_WONT = 252,
  TELNET_DO = 253,
  TELNET_DONT = 254,
  TELNET_IAC = 255
};

class TelnetServer
{
private:
  // TELNET server state
  TelnetServerState state;
  // Server object
  WiFiServer server;
  // Client object
  // HINT: Only 1 client is availible
  WiFiClient client;
  // A message to display after the new client is connected
  String welcomeMessage;
  // A command line greeting
  String commandGreeting;
  // Command buffer
  String commandBuffer;
  // An external function that parses a scanned command
  //  Arguments:
  //    caller - a calling TELNET server,
  //    tokens - command tokens array after scanning,
  //    tokensNumber - a number of tokens the scanner has found.
  std::function<void(TelnetServer &, String (&)[CommandScanner::commandMaxTokens], unsigned int)> parseFunction;

public:
  TelnetServer(String welcomeMessage, String commandGreeting,
    std::function<void(TelnetServer &, String (&)[CommandScanner::commandMaxTokens], unsigned int)> parseFunction, const unsigned int port = 23)
  : server(port), welcomeMessage(welcomeMessage), commandGreeting(commandGreeting), parseFunction(parseFunction)
  {}

  // Start the TELNET server
  inline void begin()
  {
    server.begin();
    // setNoDelay(true) makes clients to close connection in a moment after connections
  }

  // Handle the incoming data to the TELNET server
  void handle();
  
  // Print a string to the client and flush the buffer
  // HINT: There is no printf-like version of the function (WiFiClient doesn't support a vprintf method)
  //  Arguments:
  //    s - a string to print.
  void print(const String &s)
  {
    this->client.print(s);
    this->client.flush();
  }

  // Disconnect a client connected to the server
  inline void disconnectClient()
  {
    this->client.stop();
  }

  // Stop the TELNET server
  inline void stop()
  {
    this->disconnectClient();
    server.stop();
  }
};

#endif
