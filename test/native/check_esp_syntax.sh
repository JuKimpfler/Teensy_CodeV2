#!/bin/bash
# Syntaxlauf der ESP-Firmware gegen ein grobes Abbild des Arduino-ESP32-Cores (nur Typpruefung, kein echter Build).
set -e
cd "$(dirname "$0")/../.."
INC="-Itest/native/esp_stub -Ilib/Link -Ilib/BNO055 -Ilib/RadarArray -Ilib/XM125 -Ilib/IR -Ilib/IR/bus-interface -Ilib/include"
# Standard (alles am Teensy), alles am ESP und gemischt: jede Kombination muss bauen
for FLAGS in "-DSENSOR_SOURCE=0 -DESP_ARDUINO_VERSION_MAJOR=3" "-DSENSOR_SOURCE=1 -DESP_ARDUINO_VERSION_MAJOR=3" "-DSENSOR_SOURCE=1 -DESP_ARDUINO_VERSION_MAJOR=2" "-DHEADING_SOURCE=1 -DRADAR_SOURCE=0 -DIR_SOURCE=1 -DESP_ARDUINO_VERSION_MAJOR=3"; do
  echo "[$FLAGS]"
  for f in esp/src/main.cpp lib/Link/Link.cpp lib/BNO055/BNO055.cpp lib/RadarArray/RadarArray.cpp lib/XM125/XM125Distance.cpp lib/IR/IR_ring.cpp lib/IR/bus-interface/bus_interface.cpp lib/IR/bus-interface/i2c_interface.cpp; do
    g++ -std=gnu++17 -fsyntax-only -Wall -Wextra -DESP_TARGET=1 $FLAGS $INC "$f"
    echo "  ok  $f"
  done
done
