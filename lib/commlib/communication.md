# Kommunikations Information

## Protokoll
* 8 Bytes:
    * OxAA = beginn Telemetrie, 0xBB = ESP Command
    * position x
    * position y
    * certainty
    * ball dx
    * ball dy
    * flags ()
    * 0xEE = valides Ende
    * /n

## Struct:
```
struct robotData_t
{
    uint8_t x;
    uint8_t y;
    uint8_t certainty;
    uint8_t dXBall;
    uint8_t dYBall;
    uint8_t flags; // Bit 0: Ball visible, Bit 1: Robot has Ball, Bit 2: Role (0=Goalie)
};
```