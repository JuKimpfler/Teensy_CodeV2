/**
 * TrackingDifferentiator.h
 * ------------------------------------------------------------------------
 * Optionales Vorfilter fuer den Sollwert.
 *
 * Ein "harter" Sollwertsprung (z.B. neuer Soll-Winkel) wuerde den ADRC
 * dazu bringen, sofort mit maximalem Stellsignal zu reagieren. Der TD
 * erzeugt stattdessen einen glatten, physikalisch sinnvollen Verlauf
 * (v1 = geglaetteter Sollwert, v2 = dessen Ableitung), an dem sich der
 * ADRC orientieren kann. Nuetzlich z.B. wenn ein Roboter sich sanft statt
 * ruckartig auf einen neuen Zielwinkel ausrichten soll.
 *
 * Realisiert als kritisch gedaempftes Filter 2. Ordnung mit einstellbarer
 * Bandbreite wTD (je groesser, desto schneller/haerter folgt v1 dem Eingang).
 *
 * Verwendung ist optional: der ADRC funktioniert auch direkt mit einem
 * ungeglaetteten Sollwert (z.B. bei Geschwindigkeitsregelung meist nicht
 * noetig).
 * ------------------------------------------------------------------------
 */
#pragma once

#include <math.h>

class TrackingDifferentiator {
public:
    // Ts    = Abtastzeit [s]
    // wTD   = Bandbreite des Filters [rad/s] (je groesser, desto schneller)
    TrackingDifferentiator(float Ts, float wTD = 20.0f);

    void reset(float v0 = 0.0f);

    // Ein Schritt: liefert (per Referenz) geglaetteten Wert + Ableitung
    void update(float input);

    float v1() const { return v1_; } // geglaetteter Sollwert
    float v2() const { return v2_; } // Ableitung des Sollwerts

    void setBandwidth(float wTD) { wTD_ = wTD; }

private:
    float Ts_;
    float wTD_;
    float v1_ = 0.0f;
    float v2_ = 0.0f;
};
