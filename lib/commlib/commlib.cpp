#include "commlib.h"

void commlib::set(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance){
    ownFlush_.positionX = x;
    ownFlush_.positionY = y;
    ownFlush_.BallAngle = ballAngle;
    ownFlush_.BallDistance = ballDistance;
    ownFlush_.GoalAngle = goalAngle;
    ownFlush_.GoalDistance = goalDistance;
}

void commlib::set(RobotDataCom data){
    ownFlush_ = data;
}

void commlib::send(){
    //TODO
}

void commlib::read(){
    //TODO
}

bool commlib::init(){
    //TODO

    return false;
}