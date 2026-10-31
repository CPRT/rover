#include "MorseEncoder.h"
#include "Arduino.h"

MorseEncoder::MorseEncoder() {}

void MorseEncoder::begin(int pin, int wpm) {
  _pin = pin;
  _unitTime = 1200 / wpm;
  pinMode(_pin, OUTPUT);
}

unsigned int MorseEncoder::write(uint8_t character) {
  character = toupper(character);
  int index = character - 'A'; // Calculate the index for the letter

  if (index >= 0 && index < 26) {
    const char *morse = reinterpret_cast<const char *>(pgm_read_dword(
        &morseCodes[index])); // Get the Morse code for the letter

    for (int j = 0; morse[j] != '\0'; j++) {
      if (morse[j] == '.') {
        dot();
      } else if (morse[j] == '-') {
        dash();
      }
    }

    letterSpace();
    return 1;
  } else if (character >= '0' && character <= '9') {
    // Handle numbers if needed
    int index = character - '0' + 26; // Calculate the index for the digit

    const char *morse = reinterpret_cast<const char *>(
        pgm_read_dword(&morseCodes[index])); // Get the Morse code for the digit

    for (int j = 0; morse[j] != '\0'; j++) {
      if (morse[j] == '.') {
        dot();
      } else if (morse[j] == '-') {
        dash();
      }
    }

    letterSpace();
    return 1;
  } else if (character == ' ') {
    // Handle space if needed
    space();
    return 1;
  } else {
    // Handle other characters or invalid characters as needed
    int index = -1;

    // Check if the character is one of the special characters
    const char specialChars[] = ".,:;?=/!-_\"()$@&+";
    for (unsigned int j = 0; j < sizeof(specialChars) - 1; j++) {
      if (character == specialChars[j]) {
        index = j;
        break;
      }
    }

    if (index >= 0 && index < 17) {
      const char *morse = reinterpret_cast<const char *>(pgm_read_dword(
          &morseSpecialChars[index])); // Get the Morse code for the character

      for (int j = 0; morse[j] != '\0'; j++) {
        if (morse[j] == '.') {
          dot();
        } else if (morse[j] == '-') {
          dash();
        }
      }
      letterSpace();
      return 1;
    }
  }

  return 0;
}