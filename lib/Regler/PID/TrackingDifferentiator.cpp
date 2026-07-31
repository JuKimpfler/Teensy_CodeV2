#include "TrackingDifferentiator.h"

TrackingDifferentiator::TrackingDifferentiator(float Ts, float wTD)
    : Ts_(Ts), wTD_(wTD) {}

void TrackingDifferentiator::reset(float v0) {
    v1_ = v0;
    v2_ = 0.0f;
}

void TrackingDifferentiator::update(float input) {
    // Kritisch gedaempftes System 2. Ordnung:
    //   v1' = v2
    //   v2' = wTD^2 * (input - v1) - 2*wTD*v2
    float dv1 = v2_;
    float dv2 = wTD_ * wTD_ * (input - v1_) - 2.0f * wTD_ * v2_;
    v1_ += dv1 * Ts_;
    v2_ += dv2 * Ts_;
}
