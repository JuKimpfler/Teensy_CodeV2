#include "roboterDaten.h"

#ifndef commlib_h
#define commlib_h

class commlib{
    public:
        roboterDaten getOtherFlush() const;
        void setOwnFlush(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance);

    private:
        roboterDaten otherFlush_;
        roboterDaten ownFlush_;
};

extern commlib commlibC;

#endif