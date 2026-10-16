#pragma once

/**
 * params.h  --  Sammel-Header ALLER Projektparameter
 *
 * Alle einstellbaren Werte des Projekts stehen in lib/include/params/, je
 * Themengebiet eine Datei. Bibliotheken binden nur diesen Header ein und legen
 * keine eigenen Kopien an. Pins: enum.h. Regeln: docs/Coding_Rules.md.
 */

#include "params/params_sources.h"  // Debug-Flag: Sensoren am Teensy oder am ESP
#include "params/params_system.h"   // Busse, Debug-Modus, Linien-Schwellen, Kurs-Regler
#include "params/params_radar.h"    // XM125-Radarsensoren
#include "params/params_field.h"    // Feld- und Torgeometrie
#include "params/params_position.h" // Odometrie, Radar-Validierung, Fusion
#include "params/params_motion.h"   // moveToPoint, Speed-Pattern, Punkt-Array
#include "params/params_path.h"     // Pfadplanung
#include "params/params_link.h"     // Link Teensy <-> ESP32-S3
