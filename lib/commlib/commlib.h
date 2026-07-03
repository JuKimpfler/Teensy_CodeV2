

#ifndef commlib_h
#define commlib_h

struct RobotDataCom
{
    float positionX;
    float positionY;

    float BallAngle;
    float BallDistance;

    float GoalAngle;
    float GoalDistance;

    RobotDataCom(float positionX_ , float positionY_ , float ballAngle_, float ballDistance_, float goalAngle_, float goalDistance_ ) : positionX(positionX_), positionY(positionY_), BallAngle(ballAngle_), BallDistance(ballDistance_), GoalAngle(goalAngle_), GoalDistance(goalDistance_) {}
};

class commlib
{
public:
    RobotDataCom getOtherFlush() const;
    void setOwnFlush(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance);
    void sendOwnFlush();

private:
    RobotDataCom otherFlush_;
    RobotDataCom ownFlush_;
};

#endif