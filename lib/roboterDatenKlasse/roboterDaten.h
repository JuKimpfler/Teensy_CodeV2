#ifndef roboterDateien_H
#define roboterDateien_H

class roboterDaten{
    public:
        roboterDaten(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance); 

        float getX() const;
        float getY() const;

        float getBallAngle() const;
        float getBallDistance() const;

        float getGoalAngle() const;
        float getGoalDistance() const;

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