#include <Arduino.h>

class fancyBTN {
  public:
    fancyBTN(uint8_t sensitivity, uint8_t longPressTime, bool InputWhenOnBTNdown); // @param sensitivity: 0-255, Time in 2 milliseconds you have to press the Button twice to trigger a double click. 0 = no double click detection, 255 = 510ms @param longPressTime: 0-255, Time in half seconds you have to press the Button to trigger a long press. 0 = no long press detection, 255 = 127.5 seconds
    bool isPressed() const { return _buttonState; };
    bool isReleased() const { return !_buttonState; };
    bool wasPressed() const { return _buttonPressed; };
    bool wasReleased() const { return _buttonReleased; };
    uint8_t getStatusCode() const { return _statusCode; }; // @return 0 = not pressed, 1 = new_pressed, 2 = pressed, 3= new_released, 4 = double click, 5 = long press 
    void update(bool buttonState); // @param buttonState: true = pressed, false = released

  private:
    uint8_t _sensitivity;
    uint8_t _longPressTime;
    bool _InputWhenOnBTNdown;
    unsigned long _lastTimePressed;
    unsigned long _scndlastTimePressed;
    unsigned long _lastTimeNotPressed;
    bool _lastButtonState;
    bool _buttonState;
    bool _buttonPressed;
    bool _buttonReleased;
    uint8_t _statusCode;
};
