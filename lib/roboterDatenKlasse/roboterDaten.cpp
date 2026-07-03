#include "roboterDaten.h" 

roboterDaten roboterDatenC;

roboterDaten::roboterDaten(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance) : x_(x), y_(y), ballAngle_(ballAngle), ballDistance_(ballDistance), goalAngle_(goalAngle), goalDistance_(goalDistance) {}

float roboterDaten::getX() const {
	return x_;
}

float roboterDaten::getY() const {
	return y_;
}

float roboterDaten::getBallAngle() const{
	return ballAngle_;
}

float roboterDaten::getBallDistance() const {
	return ballDistance_;
}

float roboterDaten::getGoalAngle() const{
	return goalAngle_;
}

float roboterDaten::getGoalDistance() const {
	return goalDistance_;
}