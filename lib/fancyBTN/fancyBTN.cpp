#include <Arduino.h>
#include "fancyBTN.h"

fancyBTN::fancyBTN(uint8_t sensitivity, uint8_t longPressTime, bool InputWhenOnBTNdown)
{
    _sensitivity = sensitivity;
    _longPressTime = longPressTime;
    _InputWhenOnBTNdown = InputWhenOnBTNdown;
}

void fancyBTN::update(bool buttonState)
{
    unsigned long currentTime = millis();
    buttonState = _InputWhenOnBTNdown ? buttonState : !buttonState;

    if (buttonState)
    {
        if(!_lastButtonState) // Button was just pressed
            _scndlastTimePressed = _lastTimePressed; // Update second last time pressed
        _lastTimePressed = currentTime;
    }

    else
        _lastTimeNotPressed = currentTime;

    if (buttonState == _lastButtonState)
    {
        _buttonPressed = buttonState;
        _buttonReleased = !buttonState;
        _statusCode = buttonState ? 2 : 0; // pressed or not pressed
    }
    else
    {
        if (buttonState)
        {
            _buttonPressed = true;
            _buttonReleased = false;
            _statusCode = 1; // new_pressed
        }
        else
        {
            _buttonPressed = false;
            _buttonReleased = true;
            _statusCode = 3; // new_released
        }
    }

    if (_statusCode == 1 && _sensitivity > 0 && (currentTime - _scndlastTimePressed <= _sensitivity * 2) && (currentTime - _scndlastTimePressed > 4))
    {
        _statusCode = 4; // double click
    }

    if (_buttonPressed && (currentTime - _lastTimeNotPressed >= _longPressTime * 500))
    {
        _statusCode = 5; // long press
    }

    _lastButtonState = buttonState;
}