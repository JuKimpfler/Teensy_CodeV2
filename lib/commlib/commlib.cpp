#include "commlib.h"

commlib commlibC;

void commlib::setOwnFlush(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance){
    ownFlush_ = roboterDaten(x, y, ballAngle, ballDistance, goalAngle, goalDistance);

    //TODO: ownFlush an anderen Roboter senden
}

roboterDaten commlib::getOtherFlush() const{
    //TODO: otherFlush von anderem Roboter empfangen

    return otherFlush_;
}