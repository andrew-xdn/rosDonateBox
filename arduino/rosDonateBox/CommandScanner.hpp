#ifndef H_COMMAND_SCANNER
#define H_COMMAND_SCANNER

#include <WString.h>
#include <HardwareSerial.h>
#include "config.hpp"

// Specify a debug printf function or comment out to disable debug output
#define COMMAND_SCANNER_DEBUG(...)  Serial.printf(__VA_ARGS__)

#ifndef COMMAND_SCANNER_DEBUG
// Remove all debug calls
#define COMMAND_SCANNER_DEBUG(...)
#endif // COMMAND_SCANNER_DEBUG

// Command scanner error codes (non-negative number is a tokens count)
enum CommandScanError
{
  CSE_MANY_TOKENS = -1,  // Too many tokens in the command string
  CSE_UNTERMINATED_ESCAPE = -2  // Unterminated double quotes in the command string
};

namespace CommandScanner
{
  // Maximum command arguments number in a command
  static const unsigned int commandMaxArguments = 3;
  // Maximum command tokens number in a command string (command + maximum arguments number)
  static constexpr unsigned int commandMaxTokens = 1 + commandMaxArguments;

  // Split a command string to command tokens
  //  Arguments:
  //    command - command string,
  //    tokens - string object references array.
  //  Returns:
  //    tokens count (if positive or zero), error code (if negative).
  int scan(String& command, String (&tokens)[commandMaxTokens]);
}

#endif
