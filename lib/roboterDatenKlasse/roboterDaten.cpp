#include "roboterDaten.h" 

roboterDaten roboterDatenC;

roboterDaten::roboterDaten(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance) : x_(x), y_(y), ballAngle_(ballAngle), ballDistance_(ballDistance), goalAngle_(goalAngle), goalDistance_(goalDistance) {}

float roboterDaten::getX() const {
	return x_;
}

void roboterDaten::setX(float x){
	x_ = x;
}

float roboterDaten::getY() const {
	return y_;
}

void roboterDaten::setY(float y){
	y_ = y;
}

float roboterDaten::getBallAngle() const{
	return ballAngle_;
}

float roboterDaten::setBallAngle(float ballAngle){
	ballAngle_ = ballAngle;
}

float roboterDaten::getBallDistance() const {
	return ballDistance_;
}

float roboterDaten::setBallDistance(float ballDistance){
	ballDistance_ = ballDistance;
}

float roboterDaten::getGoalAngle() const{
	return goalAngle_;
}

float roboterDaten::setGoalAngle(float goalAngle){
	goalAngle_ = goalAngle;
}

float roboterDaten::getGoalDistance() const {
	return goalDistance_;
}

float roboterDaten::setGoalDistance(float goalDistance){
	goalDistance_ = goalDistance;
}

