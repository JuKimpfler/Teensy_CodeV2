#include "PDS.h"
#include "elapsedMillis.h"

elapsedMillis DBGTimer;

static float debugData[PowerDebugger::MAX_FLOATS];
#define DBG(channel, value)  debugData[(channel)] = static_cast<float>(value);

void PowerDebugger::Channel(u_int8_t chn , float val){
    DBG(chn,val);
}

void PowerDebugger::buildPacket() {
    // ── Header: Magic + Timestamp ─────────────────────────────────────────────
    const uint32_t magic = HEADER_MAGIC;
    const uint32_t ts    = micros();
    memcpy(_pkt_buf,     &magic, 4);
    memcpy(_pkt_buf + 4, &ts,    4);

    // ── Nutzdaten: debugData[] direkt kopieren ────────────────────────────────
    memcpy(_pkt_buf + 8, debugData, MAX_FLOATS * sizeof(float));
}

void PowerDebugger::init(){
    // Debug-Array initialisieren (alle Kanäle = inaktiv / Dummy)
    for (int i = 0; i < MAX_FLOATS; i++) debugData[i] = 0;  // 9898.0f Dummy wert

    // Serial1 TX-Buffer erweitern und UART starten
    UART_DBG.addMemoryForWrite(_serial1_tx_buf, sizeof(_serial1_tx_buf));
    UART_DBG.begin(UART_DBG_BAUD, SERIAL_8N1);

    pinMode(10,INPUT);
}

void PowerDebugger::update(){
    static uint32_t last_sample_us = 0;
    static uint32_t last_stat_ms   = 0;
    static uint32_t loop_start_us  = 0;

    DBGTimer = 0;

    // ── Alle 10 ms: Paket senden (100 Hz) ────────────────────────────────────
    if (DBGTimer >= SAMPLE_PERIOD_US) {
        buildPacket();

        // Serial1.write() kopiert 1608 Bytes in den TX-Buffer und kehrt
        // sofort zurück. Der UART-DMA überträgt asynchron (~4 ms bei 4 Mbps).
        // Bei 10 ms Paket-Intervall ist der Buffer stets leer wenn wir schreiben.
        //last_buffer = _pkt_buf;
        //last_bytes = PACKET_BYTES;
        UART_DBG.write(_pkt_buf, PACKET_BYTES);
        pkt_count++;
    }

    // ── Statistik alle 5 Sekunden auf USB-Serial ausgeben ────────────────────
    if (DBGTimer >= 5000UL) {
        const float hz    = pkt_count / 5.0f;
        const float kbps  = (float)pkt_count * PACKET_BYTES / 5.0f / 1024.0f;
        pkt_count = 0;
    }
}