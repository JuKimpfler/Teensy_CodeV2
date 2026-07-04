

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