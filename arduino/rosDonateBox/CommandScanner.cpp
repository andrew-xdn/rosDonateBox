#include "CommandScanner.hpp"

int CommandScanner::scan(String& command, String (&tokens)[commandMaxTokens])
{
  Serial.printf("[COMMAND_SCANNER] Incoming command: \"%s\"\n", command.c_str());

  int charIndex;
  char c;
  
  int tokenIndex = 0;

  // Space escape flag (" has been found)
  bool escapeFlag = false;
  // Token is empty flag ("")
  // HINT: Empty tokens are useful to set an empty value to the parameter
  bool emptyToken = false;

  // Check all characters
  for (charIndex = 0; charIndex < command.length(); charIndex++)
  {
    // Retrieve a current character
    c = command[charIndex];
    
    switch (c)
    {
      // Token termination
      case ' ':
        // There has been no " character, it's really a token termination
        if (!escapeFlag)
        {
          // The token termination is only possible if it's an empty token ("") or a token with some characters
          if (emptyToken || tokens[tokenIndex].length())
          {
            // We have a free string object in the array to keep a new token
            if (tokenIndex < commandMaxTokens - 1)
            {
              COMMAND_SCANNER_DEBUG("[COMMAND_SCANNER] Token found: \"%s\"\n", tokens[tokenIndex].c_str());

              tokenIndex++;
              emptyToken = false;
              // Clean the storage for the new token
              tokens[tokenIndex] = "";
            }
            // We have run out of the string objects
            else
            {
              COMMAND_SCANNER_DEBUG("[COMMAND_SCANNER] Too many tokens!\n");

              // Return the error code
              return CSE_MANY_TOKENS;
            }
          }
          // It is a space sequence out of the "", ignore it
        }
        // A space character inside the "", it's a part of the token
        else
          tokens[tokenIndex] += c;
          
        break;
      // Space escape character
      case '"':
        // Escape flag is already set, but ther is nothing inside the token - it's an empty token ("")
        if (escapeFlag && !tokens[tokenIndex].length())
          emptyToken = true;

        // Inverse the empty token flag
        escapeFlag = !escapeFlag;
        break;
      // Any other character
      default:
       // Add to the token
       tokens[tokenIndex] += c;
       break;
    }
  }

  // The string has ended, we have to terminate the last token

  // There is no pair for the last escape character
  if (escapeFlag)
  {
    COMMAND_SCANNER_DEBUG("[COMMAND_SCANNER] Unterminated escape!\n");

    // Return the error
    return CSE_UNTERMINATED_ESCAPE;
  }

  // It's an empty token or a token with some characters
  if (emptyToken || tokens[tokenIndex].length())
  {
    COMMAND_SCANNER_DEBUG("[COMMAND_SCANNER] New token found: \"%s\"\n", tokens[tokenIndex].c_str());

    tokenIndex++;
  }

  COMMAND_SCANNER_DEBUG("[COMMAND_SCANNER] Total number of tokens: %u\n", tokenIndex);

  return tokenIndex;
}
