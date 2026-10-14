#pragma once
#include <Arduino.h>

class fancyBTN
{
public:
    // @param sensitivity: 0-255, Time in 2 milliseconds you have to press the Button twice to trigger a double click. 0 = no double click detection, 255 = 510ms @param longPressTime: 0-255, Time in half seconds you have to press the Button to trigger a long press. 0 = no long press detection, 255 = 127.5 seconds
    fancyBTN(uint8_t sensitivity, uint8_t longPressTime, uint8_t debounceTime, bool InputWhenOnBTNdown);
    bool isPressed() const { return _buttonState; };
    uint8_t giveStatusCode() const { return _statusCode; }; // @return 0 = not pressed, 1 = new_pressed, 2 = pressed, 3= new_released,4 = clicked, 5 = double click, 6 = long press
    void update(bool buttonState);                          // @param buttonState: true = pressed, false = released

private:
    uint8_t _sensitivity;
    uint8_t _longPressTime;
    uint8_t _statusCodeLifetime;
    uint8_t _debounceTime;
    bool _InputWhenOnBTNdown;
    unsigned long _lastTimePressed;
    unsigned long _lastTimeReleased;
    bool _lastButtonState;
    bool _buttonState;
    bool _buttonPressed;
    bool _buttonReleased;
    bool _statusClickedSet;
    bool _disableDoubleClick;
    bool _disableLongPress;
    uint8_t _statusCode;
};

enum fancyBTNStatusCode
{
    NOT_PRESSED = 0,
    NEW_PRESSED = 1,
    PRESSED = 2,
    NEW_RELEASED = 3,
    fancy_CLICKED = 4,
    fancy_DOUBLE_CLICK = 5,
    fancy_LONG_PRESS = 6
};
