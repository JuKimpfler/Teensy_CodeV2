#include "commlib.h"

void commlib::setOwnFlush(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance){
    ownFlush_ = RobotDataCom(x, y, ballAngle, ballDistance, goalAngle, goalDistance);
}

void commlib::sendOwnFlush(){
    //TODO
}

RobotDataCom commlib::getOtherFlush() const{
    //TODO: otherFlush von anderem Roboter empfangen

    return otherFlush_;
}