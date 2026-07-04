#include "commlib.h"

void commlib::setOwnFlush(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance){
    ownFlush_.positionX = x;
    ownFlush_.positionY = y;
    ownFlush_.BallAngle = ballAngle;
    ownFlush_.BallDistance = ballDistance;
    ownFlush_.GoalAngle = goalAngle;
    ownFlush_.GoalDistance = goalDistance;
}

void commlib::sendOwnFlush(){
    //TODO
}

RobotDataCom commlib::getOtherFlush() const{
    //TODO: otherFlush von anderem Roboter empfangen

    return otherFlush_;
}