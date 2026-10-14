#include <Arduino.h>
#include "fancyBTN.h"

fancyBTN::fancyBTN(uint8_t sensitivity, uint8_t longPressTime, uint8_t debounceTime = 5, bool InputWhenOnBTNdown = true)
{
    _sensitivity = sensitivity;
    _longPressTime = longPressTime;
    _debounceTime = debounceTime;
    _InputWhenOnBTNdown = InputWhenOnBTNdown;
}

void fancyBTN::update(bool buttonState)
{
    unsigned long currentTime = millis();
    buttonState = _InputWhenOnBTNdown ? buttonState : !buttonState;
    _buttonState = buttonState;

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
            _lastTimePressed = currentTime;
            _statusClickedSet = false;
            _disableLongPress = false;
        }
        else
        {
            _buttonPressed = false;
            _buttonReleased = true;
            _statusCode = 3; // new_released
            _lastTimeReleased = currentTime;
            _disableDoubleClick = false;
        }
    }

    if (_statusCode == 0 && (currentTime - _lastTimeReleased > _sensitivity * 2) && !_statusClickedSet && (_lastTimeReleased - _lastTimePressed < _longPressTime * 500))
    {
        _statusCode = 4; // clicked
        _statusClickedSet = true;
    }

    if (_statusCode == 1 && (currentTime - _lastTimeReleased < _sensitivity * 2) && (currentTime - _lastTimeReleased > _debounceTime) && !_disableDoubleClick)
    {
        _statusCode = 5; // double click
        _statusClickedSet = true;
        _disableLongPress = true;
    }

    if (_statusCode == 3 && (currentTime - _lastTimePressed > _longPressTime * 500) && !_disableLongPress)
    {
        _statusCode = 6; // long press
        _statusClickedSet = true;
        _disableDoubleClick = true;
    }

    _lastButtonState = buttonState;
}
