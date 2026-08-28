

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
    RobotDataCom getOther() const { return otherFlush_; };
    RobotDataCom getOwn() const { return ownFlush_; };
    void set(RobotDataCom data);
    void set(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance);
    void send();
    void read();

private:
    RobotDataCom otherFlush_;
    RobotDataCom ownFlush_;
};

#endif