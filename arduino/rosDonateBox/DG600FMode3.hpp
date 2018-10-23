#ifndef H_DG600F_MODE3
#define H_DG600F_MODE3

/**
 DG600F coin acceptor driver for mode 3 (header + coin sum + checksum).
*/

#include "Stream.h"
#include <HardwareSerial.h>

// Specify a debug printf function or comment out to disable debug output
#define DG600F_MODE3_DEBUG(...)  Serial.printf(__VA_ARGS__)

#ifndef DG600F_MODE3_DEBUG
// Remove all debug calls
#define DG600F_MODE3_DPRINTF(...)
#endif // DG600F_MODE3_DEBUG

// DG600F Mode 3 driver FSM states
enum DG600FMode3State
{
  DM3S_HEADER = 0,  // Waiting for the header
  DM3S_VALUE,  // Waiting for the coin value
  DM3S_CHECKSUM  // Waiting for the checksum
};

class DG600FMode3
{
private:
  // DG600F Mode 3 header character
  static const char DG600FMode3Header = 0xAA;

  // Serial port stream
  Stream &serialStream;
  // DG600F Mode 3 FSM state
  // HINT: Self-synchronizing FSM, no need to reset this state outside of the function
  DG600FMode3State state;
  // A recieved coin value
  unsigned int value;
  
public:
  DG600FMode3(Stream &serialStream)
  : serialStream(serialStream),
    // Initialise FSM
    state(DM3S_HEADER)
  {
  }
  
  // Process DG600F Mode 3 data
  // Return:
  //    -1 (all data has been parsed), coin value (a frame has been found)
  // HINT: Call this function repeatedly until you get -1
  int read();
};

#endif
