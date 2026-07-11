#ifndef MorseEncoder_h
#define MorseEncoder_h

const char* const morseCodes[36] PROGMEM = {
  ".-",    // A
  "-...",  // B
  "-.-.",  // C
  "-..",   // D
  ".",     // E
  "..-.",  // F
  "--.",   // G
  "....",  // H
  "..",    // I
  ".---",  // J
  "-.-",   // K
  ".-..",  // L
  "--",    // M
  "-.",    // N
  "---",   // O
  ".--.",  // P
  "--.-",  // Q
  ".-.",   // R
  "...",   // S
  "-",     // T
  "..-",   // U
  "...-",  // V
  ".--",   // W
  "-..-",  // X
  "-.--",  // Y
  "--..",  // Z
  "-----", // 0
  ".----", // 1
  "..---", // 2
  "...--", // 3
  "....-", // 4
  ".....", // 5
  "-....", // 6
  "--...", // 7
  "---..", // 8
  "----."  // 9
};

const char* const morseSpecialChars[17] PROGMEM = {
  ".-.-.-",  // .
  "--..--",  // ,
  "---...",  // :
  "-.-.-.",  // ;
  "..--..",  // ?
  "-...-",   // =
  "-..-.",   // /
  "-.-.--",  // !
  "-....-",  // -
  "..--.-",  // _
  ".-..-.",  // "
  "-.--.",   // (
  "-.--.-",  // )
  "...-..-", // $
  ".--.-.",  // @
  ".-...",   // &
  ".-.-."    // +
};

class MorseEncoder : public Print {
  public:
    MorseEncoder();

    void begin(int pin, int wpm);

    unsigned int write(uint8_t character);
    using Print::write;

  private:
    int _pin;
    int _unitTime;
    int _freq;

    void dot() {
      digitalWrite(_pin, HIGH);
      delay(_unitTime);
      digitalWrite(_pin, LOW);
      delay(_unitTime);
    }

    void dash() {
      digitalWrite(_pin, HIGH);
      delay(3 * _unitTime);
      digitalWrite(_pin, LOW);
      delay(_unitTime);
    }

    void space() {
      delay(6 * _unitTime); // cuz every dot and dash end with one unit, (actual space 7 unit)
    }
    
    void letterSpace() {
      delay(_unitTime * 2);
    }
};

#endif
