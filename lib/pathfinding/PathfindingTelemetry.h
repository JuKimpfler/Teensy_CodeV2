// PathfindingTelemetry.h
//
// Kleine Header-only Bibliothek, um Debug-Daten eines gewichteten-Karten
// Pathfinders (z.B. A*) binaer ueber eine Stream-Schnittstelle (Serial /
// USB-CDC) an ein PC-Analysetool zu senden.
//
// Kompatibel mit Teensy 4.0 und ESP32-S3 (beide little-endian, beide
// unterstuetzen Stream::write). Keine .cpp Datei noetig, einfach
// #include <PathfindingTelemetry.h> in den Sketch.
//
// ------------------------------------------------------------------
// PROTOKOLL
// ------------------------------------------------------------------
// Jeder Frame:
//   [0xA5][0x5A][TYPE:1][LEN_LO][LEN_HI][PAYLOAD: LEN bytes][CHECKSUM:1]
//
// CHECKSUM = (Summe aller Payload-Bytes) mod 256
//
// TYPE 'F' (FRAME_FIELD)  -> Payload: [W_LO][W_HI][H_LO][H_HI][W*H bytes]
//                            Feld ist row-major ueber x (0..W-1), dann y (0..H-1),
//                            d.h. dieselbe Reihenfolge wie field[x][y] im Sketch.
// TYPE 'P' (FRAME_PATH)   -> Payload: [COUNT_LO][COUNT_HI][x0][y0][x1][y1]...
// TYPE 'M' (FRAME_META)   -> Payload: 15 bytes, siehe struct Meta unten.
//
// Die Python-Seite synchronisiert sich anhand der beiden Sync-Bytes und
// ignoriert alles andere (z.B. normale Serial.println Debug-Ausgaben),
// solange diese nicht zufaellig die Byte-Folge 0xA5 0x5A enthalten.
// ------------------------------------------------------------------

#pragma once
#include <Arduino.h>
#include <string.h>

namespace PFTelemetry
{

    static const uint8_t SYNC1 = 0xA5;
    static const uint8_t SYNC2 = 0x5A;

    enum FrameType : uint8_t
    {
        FRAME_FIELD = 'F',
        FRAME_PATH = 'P',
        FRAME_META = 'M',
    };

    // Metadaten zu einer Pathfinding-Iteration.
    struct Meta
    {
        uint32_t iteration;   // laufender Zaehler
        uint32_t durationMs;  // Dauer der Suche in ms
        uint16_t pathLength;  // Anzahl Punkte im zuletzt gesendeten Pfad
        uint8_t startX, startY;
        uint8_t targetX, targetY;
        uint8_t pathFound; // 0/1
    };

    // Schreibt einen kompletten Frame inkl. Sync/Header/Checksumme.
    inline void writeFrame(Stream &port, uint8_t type, const uint8_t *payload, uint16_t len)
    {
        uint8_t checksum = 0;
        port.write(SYNC1);
        port.write(SYNC2);
        port.write(type);
        port.write((uint8_t)(len & 0xFF));
        port.write((uint8_t)((len >> 8) & 0xFF));
        if (len > 0)
        {
            port.write(payload, len);
            for (uint16_t i = 0; i < len; i++)
                checksum = (uint8_t)(checksum + payload[i]);
        }
        port.write(checksum);
    }

    // Sendet ein WxH grosses Gewichtsfeld (field[x][y]).
    // Nutzt einen statischen Puffer (kein malloc), Groesse wird zur
    // Compile-Zeit ueber die Array-Dimensionen von "field" bestimmt.
    template <uint16_t W, uint16_t H>
    inline void sendField(Stream &port, const uint8_t (&field)[W][H])
    {
        static uint8_t buf[4 + (uint32_t)W * H];
        buf[0] = (uint8_t)(W & 0xFF);
        buf[1] = (uint8_t)((W >> 8) & 0xFF);
        buf[2] = (uint8_t)(H & 0xFF);
        buf[3] = (uint8_t)((H >> 8) & 0xFF);

        uint32_t idx = 4;
        for (uint16_t x = 0; x < W; x++)
            for (uint16_t y = 0; y < H; y++)
                buf[idx++] = field[x][y];

        writeFrame(port, FRAME_FIELD, buf, (uint16_t)(4 + (uint32_t)W * H));
    }

    // Sendet den gefundenen Pfad. pathLength <= MaxPoints (Groesse des
    // zweiten Arrays, z.B. 600 wie im Original-Sketch).
    template <uint16_t MaxPoints>
    inline void sendPath(Stream &port, const uint8_t (&path)[MaxPoints][2], uint16_t pathLength)
    {
        static uint8_t buf[2 + (uint32_t)MaxPoints * 2];
        if (pathLength > MaxPoints)
            pathLength = MaxPoints;

        buf[0] = (uint8_t)(pathLength & 0xFF);
        buf[1] = (uint8_t)((pathLength >> 8) & 0xFF);
        for (uint16_t i = 0; i < pathLength; i++)
        {
            buf[2 + i * 2] = path[i][0];
            buf[2 + i * 2 + 1] = path[i][1];
        }
        writeFrame(port, FRAME_PATH, buf, (uint16_t)(2 + (uint32_t)pathLength * 2));
    }

    // Sendet die Metadaten einer Iteration.
    inline void sendMeta(Stream &port, const Meta &m)
    {
        uint8_t buf[15];
        memcpy(buf + 0, &m.iteration, 4);
        memcpy(buf + 4, &m.durationMs, 4);
        memcpy(buf + 8, &m.pathLength, 2);
        buf[10] = m.startX;
        buf[11] = m.startY;
        buf[12] = m.targetX;
        buf[13] = m.targetY;
        buf[14] = m.pathFound;
        writeFrame(port, FRAME_META, buf, sizeof(buf));
    }

    // Komfortfunktion: sendet Meta, Feld und Pfad in einem Rutsch.
    // Reihenfolge ist bewusst Meta -> Field -> Path, damit die GUI schon
    // die Iterationsnummer/Zeiten kennt, bevor die (groesste) Feld-Nutzlast
    // eintrudelt.
    template <uint16_t W, uint16_t H, uint16_t MaxPoints>
    inline void sendFrame(Stream &port,
                           const uint8_t (&field)[W][H],
                           const uint8_t (&path)[MaxPoints][2],
                           const Meta &meta)
    {
        sendMeta(port, meta);
        sendField(port, field);
        sendPath(port, path, meta.pathLength);
    }

} // namespace PFTelemetry
