# IR-Ring Library
***

Dieses Repository enthält eine Bibliothek für den [Teensy 4.0](https://www.pjrc.com/store/teensy40.html) zur Kommunikation mit der [IR-Ring](https://git.markdorf-robotics.de/PCBs/IR-Ring/-/releases/permalink/latest) Platine der Roboter-AG.

Die Firmware für den IR-Ring ist unter [Firmware/IR-Ring](https://git.markdorf-robotics.de/firmware/ir-ring-fw) zu finden. Es empfiehlt sich die Firmware immer auf aktuellem Stand zu halten. Es ist nicht garantiert, dass neue Features in der Bibliothek auch mit alter Firmware kompatibel sind.

> Achtung: Seit Version v0.1.0 wurde die Headerdatei in `ir_ring.h` umbenannt (vorher `IR_ring.h`). Bestehende Includes müssen aktualisiert werden.

***
## Beispielcode

Im folgenden ein kurzes Beispielprogramm. Ein ausführliches Codebeispiel, das alle implementierten Features zeigt, findet sich unter [Examples](/examples).

``` c++
#include <Arduino.h>
#include <Wire.h>

#include "ir_ring.h"

#define I2C_BUS Wire1     // IRL-Ring on Wire1
#define I2C_SPEED 1000000 // 1 MHz

// IR-Ring setup
I2C_bus i2c(I2C_BUS);
Device_handle ir_ring_handle = {i2c, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

void setup()
{
  Serial.begin(115200);

  // I2C setup
  I2C_BUS.begin();
  I2C_BUS.setClock(I2C_SPEED);
}

void loop()
{
  Serial.print("Ball angle: "); Serial.println(ir_ring.read_ball_angle());
  Serial.print("Ball distance: "); Serial.println(ir_ring.read_ball_distance());

  uint16_t calib_data[16];
  ir_ring.read_calibrated_values(calib_data);

  bool tssp_data[8];
  ir_ring.read_tssp(tssp_data);

  Serial.print("Calibrated data: ");
  for (int i=0; i<16; i++)
  {
    Serial.print(i+1); Serial.print(": ");
    Serial.print(calib_data[i]); Serial.print("  ");
  }
  Serial.println();

  Serial.print("TSSP4038 data: ");
  for (int i=0; i<8; i++)
  {
    Serial.print(i+1); Serial.print(": ");
    Serial.print(tssp_data[i]); Serial.print("  ");
  }
  Serial.println();


  delayMicroseconds(500);
}
```