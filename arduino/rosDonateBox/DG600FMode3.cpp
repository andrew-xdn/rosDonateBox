#include "DG600FMode3.hpp"

int DG600FMode3::read()
{
  // A character from the serial
  int c;

//  // The buffer overflow detected
//  // WARNING: Old SoftwareSerial library versions don't support this method
//  if (DG600FSerial.overflow())
//    Serial.printf("[DG600F_MODE3] Serial buffer overflow\n");

  // Get a byte from the serial
  c = this->serialStream.read();

  // It's a byte
  while (c >= 0)
  {
    switch(state)
    {
      // Waiting for the frame header
      case DM3S_HEADER:
        // It's a header
        if (c == DG600FMode3::DG600FMode3Header)
        {
          DG600F_MODE3_DEBUG("[DG600F_MODE3] Received header\n");

          // Waiting for a coin value
          state = DM3S_VALUE;
        }
        // It's not a header
        else
          DG600F_MODE3_DEBUG("[DG600F_MODE3] Invalid header: 0x%02X\n", static_cast<unsigned int>(c));
        break;
      // Waiting for the coin value
      case DM3S_VALUE:
        // Coin value can't be greater than 100
        if (c <= 100)
        {
          // Store the coin value
          this->value = static_cast<unsigned int>(c);
          DG600F_MODE3_DEBUG("[DG600F_MODE3] Received sum: %u\n", value);

          // Waiting for the frame checksum
          state = DM3S_CHECKSUM;
        }
        // HINT: It's really good that coin value can't be 0xAA. Here we can find out that
        // the previous frame had been canceled and the header from the new one recieved.
        else if (c == DG600FMode3::DG600FMode3Header)
        {
          DG600F_MODE3_DEBUG("[DG600F_MODE3] Received header\n");

          // No need to switch to the new state
        }
        // It's an invalid sum
        else
        {
          DG600F_MODE3_DEBUG("[DG600F_MODE3] Invalid sum: 0x%02X\n", static_cast<unsigned int>(c));

          // Waiting for a new frame, this one is corrupted
          state = DM3S_HEADER;
        }
        break;
      // Waiting for the checksum
      case DM3S_CHECKSUM:
        // Checksum is valid
        if (c == (DG600FMode3::DG600FMode3Header ^ this->value))
        {
          DG600F_MODE3_DEBUG("[DG600F_MODE3] Received checksum (valid frame): 0x%02X\n", static_cast<unsigned int>(c));

          // Waiting for a new frame
          state = DM3S_HEADER;

          // Jump out of the function to return the coin value
          // HINT: Some data may still be in the serial buffer. User should call this function repeatedly until he or she
          //  gets -1.
          return this->value;
        }
        // Invalid checksum
        else
        {
          DG600F_MODE3_DEBUG("[DG600F_MODE3] Received checksum (invalid frame): 0x%02X\n", static_cast<unsigned int>(c));

          // It may be a frame header
          if (c == DG600FMode3::DG600FMode3Header)
          {
            DG600F_MODE3_DEBUG("[DG600F_MODE3] Received header\n");

            // Waiting for a coin value
            // HINT: Even if it was a corrupted checksum or a valid checksum with a corrupted coin value it's not a problem
            state = DM3S_VALUE;

            break;
          }

          // Waiting for the next frame, this one is corrupted
          state = DM3S_HEADER;
        }
        break;
    }

    // Get a new byte from the serial
    c = this->serialStream.read();
  }

  // No more data from the serial
  return -1;
}
