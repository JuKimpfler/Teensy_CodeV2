/**
 * XM125.cpp  —  Implementierung der XM125-Basisklasse.
 * Siehe XM125.h für Dokumentation.
 */

#include "XM125.h"

XM125Base::XM125Base(uint8_t i2cAddr, int8_t wakePin, int8_t intPin)
    : _wire(nullptr), _addr(i2cAddr), _wakePin(wakePin), _intPin(intPin)
{
}

bool XM125Base::begin(TwoWire &wire)
{
    _wire = &wire;

    if (_wakePin >= 0)
    {
        pinMode(_wakePin, OUTPUT);
        digitalWrite(_wakePin, HIGH);
    }
    if (_intPin >= 0)
    {
        pinMode(_intPin, INPUT);
    }

    if (!wakeUp())
        return false;

    return isConnected();
}

bool XM125Base::isConnected()
{
    if (_wire == nullptr)
        return false;
    _wire->beginTransmission(_addr);
    return (_wire->endTransmission() == 0);
}

bool XM125Base::wakeUp(uint32_t timeoutMs)
{
    if (_wakePin >= 0)
    {
        digitalWrite(_wakePin, HIGH);
    }

    if (_intPin >= 0)
    {
        uint32_t start = millis();
        while (digitalRead(_intPin) == LOW)
        {
            if (millis() - start > timeoutMs)
                return false;
        }
        return true;
    }

    // Kein MCU_INT verkabelt: Das Modul schläft nur nach explizitem
    // Host-Eingriff (Pin aktiv Low bzw. Sleep-Kommando) und bleibt sonst
    // dauerhaft wach — diese Bibliothek sendet nie ein Sleep-Kommando.
    // Der Retry hier ist daher nur eine generische Anlauf-Absicherung
    // (I²C-Bus kurz nach Power-On), kein Warten auf ein echtes Aufwachen.
    uint32_t start = millis();
    do
    {
        if (isConnected())
            return true;
        delay(20);
    } while (millis() - start < timeoutMs);
    return false;
}

void XM125Base::sleep()
{
    if (_wakePin >= 0)
    {
        digitalWrite(_wakePin, LOW);
    }
}

bool XM125Base::resetModule(uint32_t timeoutMs)
{
    if (_wire == nullptr)
        return false;

    // Rueckgabewert bewusst ignoriert: das Modul startet sofort neu und
    // quittiert die laufende Transaktion haeufig nicht mehr -- ein gemeldeter
    // Schreibfehler heisst hier also nicht, dass das Kommando nicht ankam.
    // Ob der Neustart geklappt hat, entscheidet allein die Bereitschafts-
    // pruefung weiter unten.
    _writeReg(XM125_REG_COMMAND, XM125_CMD_RESET_MODULE);

    // Waehrend des Bootvorgangs antwortet der Sensor gar nicht -- in dieser
    // Zeit auf dem Bus nachzufragen bringt nur NAKs.
    delay(XM125_RESET_BOOT_DELAY_MS);

    const uint32_t start = millis();

    // Pins wieder in den Wach-Zustand bringen und abwarten, bis ueberhaupt
    // eine Adresse quittiert wird (bei verkabeltem MCU_INT dessen Flanke).
    if (!wakeUp(timeoutMs))
        return false;

    // Antworten heisst noch nicht bereit: die MCU baut RSS, Sensor und
    // Detektor erst auf, solange steht BUSY. Lesefehler werden in dieser Phase
    // toleriert, sie gehoeren zum Anlauf.
    while (millis() - start < timeoutMs)
    {
        uint32_t status = 0;
        if (_readReg(XM125_REG_DETECTOR_STATUS, status) && !statusBusy(status))
        {
            return !statusError(status);
        }
        delay(XM125_RESET_POLL_INTERVAL_MS);
    }
    return false; // Timeout: Sensor meldet sich nicht bereit
}

uint32_t XM125Base::getVersionRaw()
{
    uint32_t v = 0;
    _readReg(XM125_REG_VERSION, v);
    return v;
}

uint32_t XM125Base::getProtocolStatus()
{
    uint32_t v = 0;
    _readReg(XM125_REG_PROTOCOL_STATUS, v);
    return v;
}

uint32_t XM125Base::getMeasureCounter()
{
    uint32_t v = 0;
    _readReg(XM125_REG_MEASURE_COUNTER, v);
    return v;
}

uint32_t XM125Base::getDetectorStatus()
{
    uint32_t v = 0;
    _readReg(XM125_REG_DETECTOR_STATUS, v);
    return v;
}

// ── Low-Level I²C ─────────────────────────────────────────────────────────────

bool XM125Base::_writeReg(uint16_t reg, uint32_t value)
{
    if (_wire == nullptr)
        return false;

    _wire->beginTransmission(_addr);
    _wire->write(static_cast<uint8_t>(reg >> 8));
    _wire->write(static_cast<uint8_t>(reg & 0xFF));
    _wire->write(static_cast<uint8_t>(value >> 24));
    _wire->write(static_cast<uint8_t>(value >> 16));
    _wire->write(static_cast<uint8_t>(value >> 8));
    _wire->write(static_cast<uint8_t>(value & 0xFF));
    return (_wire->endTransmission() == 0);
}

bool XM125Base::_readReg(uint16_t reg, uint32_t &value)
{
    if (_wire == nullptr)
        return false;

    _wire->beginTransmission(_addr);
    _wire->write(static_cast<uint8_t>(reg >> 8));
    _wire->write(static_cast<uint8_t>(reg & 0xFF));
    if (_wire->endTransmission(false) != 0)
        return false; // Repeated Start, kein STOP

    if (_wire->requestFrom(_addr, static_cast<uint8_t>(4)) != 4)
        return false;
    if (_wire->available() < 4)
        return false;

    uint32_t b3 = _wire->read();
    uint32_t b2 = _wire->read();
    uint32_t b1 = _wire->read();
    uint32_t b0 = _wire->read();
    value = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0;
    return true;
}

bool XM125Base::_executeCommand(uint32_t cmd, uint32_t timeoutMs)
{
    if (!_writeReg(XM125_REG_COMMAND, cmd))
    {
        // Der Schreibvorgang selbst kann als fehlgeschlagen gemeldet werden,
        // obwohl das Kommando beim Sensor angekommen ist -- z.B. wenn die
        // interne MCU des XM125 unmittelbar nach Erhalt des Kommandos den
        // I²C-Takt streckt (Clock-Stretching), um mit der Verarbeitung zu
        // beginnen, und der Host-Treiber das als Fehler/Timeout auf der
        // aktuellen Transaktion interpretiert. Daher: sofort den Status
        // gegenpruefen -- steht BUSY bereits, wurde das Kommando trotz des
        // gemeldeten Schreibfehlers offensichtlich angenommen.
        uint32_t probe = 0;
        if (!_readReg(XM125_REG_DETECTOR_STATUS, probe) || !statusBusy(probe))
        {
            return false; // Wirklich nicht angekommen
        }
    }
    delay(2); // Zeit geben, bis die interne MCU BUSY setzt, bevor wir es abfragen
    uint32_t status = 0;
    if (!_waitNotBusy(timeoutMs, status))
        return false;
    return !statusError(status);
}

bool XM125Base::_waitNotBusy(uint32_t timeoutMs, uint32_t &lastStatus)
{
    uint32_t start = millis();
    lastStatus = 0;
    do
    {
        if (!_readReg(XM125_REG_DETECTOR_STATUS, lastStatus))
            return false;
        if (!statusBusy(lastStatus))
            return true;
    } while (millis() - start < timeoutMs);
    return false; // Timeout, Sensor noch BUSY
}
