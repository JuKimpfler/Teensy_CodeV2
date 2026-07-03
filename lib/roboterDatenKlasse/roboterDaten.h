#ifndef roboterDateien_H
#define roboterDateien_H

class roboterDaten{
    public:
        roboterDaten(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance); 

        float getX() const;
        void setX(float x);
        float getY() const;
        void setY(float y);

        float getBallAngle() const;
        float setBallAngle(float ballAngle);
        float getBallDistance() const;
        float setBallDistance(float ballDistance);

        float getGoalAngle() const;
        float setGoalAngle(float goalAngle);
        float getGoalDistance() const;
        float setGoalDistance(float goalDistance);

    private:
        float x_;
        float y_;
        
        float ballAngle_;
        float ballDistance_;

        float goalAngle_;
        float goalDistance_;
};

extern roboterDaten roboterDatenC;

#endif