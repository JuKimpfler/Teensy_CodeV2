#!/bin/bash
# Baut und startet die Rechnertests (g++, mit AddressSanitizer und UBSan). Kein Teensy, kein ESP noetig.
set -e
cd "$(dirname "$0")/../.."
mkdir -p .pio_native
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=undefined \
    -Itest/native/stub -Ilib/Link -Ilib/SensorLink -Ilib/include \
    test/native/link_test.cpp lib/Link/Link.cpp lib/SensorLink/SensorLink.cpp -o .pio_native/link_test
.pio_native/link_test

# RadarArray mit simulierten Sensoren
g++ -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -fno-sanitize-recover=undefined \
    -Itest/native/radar_stub -Ilib/RadarArray -Ilib/include \
    test/native/radar_sim_test.cpp lib/RadarArray/RadarArray.cpp -o .pio_native/radar_sim_test
.pio_native/radar_sim_test
