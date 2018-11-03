#include "TelnetServer.hpp"

void TelnetServer::handle()
{
  // A new incoming client
  if (this->server.hasClient())
  {
    // We already have a client connected
    if (this->client.connected())
    {
      // Retrieve a client object
      // HINT: WiFiClient supports object copy
      WiFiClient newClient = this->server.available();
      
      TELNET_SERVER_DEBUG("[TELNET] Rejected the client %s:%u. There is another client connected.\n",
        newClient.remoteIP().toString().c_str(), newClient.remotePort());
      // Inform the user about the problem
      newClient.print("This server can serve only one client at the same time!\r\n");
      // Close the client's connection
      newClient.stop();
    }
    // No client is currently connected
    else
    {
      // Save the client object
      this->client = this->server.available();
      TELNET_SERVER_DEBUG("[TELNET] Accepted the client %s:%u\n", this->client.remoteIP().toString().c_str(), this->client.remotePort());

      // Clean the command buffer
      this->commandBuffer = "";

      // Reset the TELNET server FSM
      this->state = TSS_RECIEVE;

      // Print a welcome text and the first greeting
      this->print(this->welcomeMessage + "\r\n\r\n" + this->commandGreeting);
    }
  }

  // The client is online
  if (this->client.connected())
  {
    static int c;

    // Recieve a character from the client
    c = this->client.read();

    // If there is a character
    while (c != -1)
    { 
      // Depending on the TELENET server's state
      switch(this->state)
      {
        // Recieving a data from the client
        case TSS_RECIEVE:
          switch(c)
          {
            // Command termination
            case '\n':
              {
                TELNET_SERVER_DEBUG("[TELNET] Got command \"%s\"\n", commandBuffer.c_str());

                // Command tokens buffer
                String tokens[CommandScanner::commandMaxTokens];

                // Split the command string to the tokens
                int result = CommandScanner::scan(commandBuffer, tokens);

                // Split process has failed
                if (result < 0)
                {
                  switch (result)
                  {
                    case CSE_MANY_TOKENS:
                      this->print("Too many arguments in the command!\r\n");
                      break;
                    case CSE_UNTERMINATED_ESCAPE:
                      this->print("Unterminated '\"' in the command!\r\n");
                      break;
                    default:
                      break;
                  }
                }
                // Split process has been successful
                else if (result)
                  // Pass the tokenized command for the further processing
                  this->parseFunction(*this, tokens, static_cast<unsigned int>(result));

                // Clean the command buffer
                this->commandBuffer = "";

                // Print a command greeting
                this->print(this->commandGreeting);
              }
              break;
            // NVT IAC
            case TELNET_IAC:
              TELNET_SERVER_DEBUG("[TELNET] Incoming IAC\n");
              // Prepare TELNET server to recieve an NVT command
              this->state = TSS_COMMAND;
              break;
            // Some clients may send CR before the LF
            case '\r':
              // Ignore this character
              break;
            // ETX
            // HINT: Some clients (like Windows TELNET) send ETX on Ctrl-C
            case '\x03':
              TELNET_SERVER_DEBUG("[TELNET] Incoming ETX, closing the connection...\n", c);
              // Close the client's connection
              this->client.stop();
              break;
            // Any other character
            default:
              // It's printable
              if (isPrintable(c))
              {
                // The user command is too long
                if ((this->commandBuffer.length() + 1) > TelnetServer::commandMaxLength)
                {
                  // Print a command greeting
                  this->print("\n\rThe command is too long!\n\r");
                  
                  // Clean the command buffer
                  this->commandBuffer = "";

                  // Print a command greeting
                  this->print(this->commandGreeting);
                }
                else
                  // Store it inside the command buffer
                  this->commandBuffer += static_cast<char>(c);
              }
              // It's not a printable character
              else
                TELNET_SERVER_DEBUG("[TELNET] Ignoring unprintable character: 0x%02X\n", c);
              break;
          }
          break;
        // Waiting for an NVT command
        case TSS_COMMAND:
          switch(c)
          {
            // Escape command for the IAC character
            case TELNET_IAC:
              TELNET_SERVER_DEBUG("[TELNET] IAC escape detected\n");
              // HINT: Just ignore the character, it's important only for binary mode
              // Switch the server back to the recieving mode
              this->state = TSS_RECIEVE;
              break;
            // DO command
            case TELNET_DO:
              TELNET_SERVER_DEBUG("[TELNET] Incoming DO command: %i\n", c);
              // Informing the server about a coming option
              this->state = TSS_OPTION;
              break;
            // DONT command
            case TELNET_DONT:
              TELNET_SERVER_DEBUG("[TELNET] Incoming DON'T command: %i\n", c);
              // Informing the server about a coming option
              this->state = TSS_OPTION;
              break;
            // WILL command
            case TELNET_WILL:
              TELNET_SERVER_DEBUG("[TELNET] Incoming WILL command: %i\n", c);
              // Informing the server about a coming option
              this->state = TSS_OPTION;
              break;
            // WONT command
            case TELNET_WONT:
              TELNET_SERVER_DEBUG("[TELNET] Incoming WON'T command: %i\n", c);
              // Informing the server about a coming option
              this->state = TSS_OPTION;
              break;
            // Ctrl-C has been pushed
            case TELNET_INTERRUPT:
              TELNET_SERVER_DEBUG("[TELNET] Incoming process interrupt command: %i, closing the connection...\n", c);
              // Close the client's connection
              this->client.stop();
              // Switch the server back to the recieving mode
              this->state = TSS_RECIEVE;
              break;
            // HINT: Implement a new command here
            // Unknown command
            default:
              TELNET_SERVER_DEBUG("[TELNET] Unknown command: %i\n", c);
              // Switch the server back to the recieving mode
              this->state = TSS_RECIEVE;
              break;
          }
          break;
        // Waiting for an NVT option
        case TSS_OPTION:
          TELNET_SERVER_DEBUG("[TELNET] Incoming option: %i\n", c); 
          // HINT: In this version we just drop them (it's ok with the most clients)
          // TODO: Implement NVT answers for the most popular options
          // Switch the server back to the recieving mode
          this->state = TSS_RECIEVE;
          break;
      }

      // Read the new character from the client
      c = this->client.read();
    }
  }
}
