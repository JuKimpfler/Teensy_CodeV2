/*
 * SerialSync.h - Variablen (bool, int32, float) zwischen zwei Mikrocontrollern
 *                (Teensy <-> ESP32) per UART synchronisieren.
 *
 * Richtung pro Variable (aus Sicht des jeweiligen Geraets):
 *   SYNC_SEND    -> diese Seite besitzt die Variable und sendet Aenderungen
 *   SYNC_RECEIVE -> die Gegenseite besitzt sie, hier wird nur empfangen
 *   SYNC_BOTH    -> beide duerfen schreiben (beim Start gilt der Wert des Masters)
 *
 * ID und Typ muessen auf beiden Seiten gleich sein, die Richtung ist gespiegelt.
 * Beide Seiten senden alle 500 ms einen Heartbeat -> connected() zeigt die Verbindung.
 * Wiederanlauf: Sobald die Gegenseite (neu) auftaucht, werden alle eigenen Werte erneut
 * gesendet. Zusaetzlich werden SEND-Variablen alle REFRESH_MS wiederholt, damit verlorene
 * Pakete (Start, Stoerungen) sich selbst heilen, auch wenn sich der Wert nicht aendert.
 *
 * Paketformat (8 Byte): [0xAA][ID][TYP][4 Byte Wert LE][CRC8 ueber ID..Wert]
 */
#pragma once
#include <Arduino.h>
#include <string.h>
#include <math.h>

enum SyncDir : uint8_t { SYNC_SEND = 0, SYNC_RECEIVE = 1, SYNC_BOTH = 2 };

class SerialSync {
public:
  static const uint8_t  MAX_VARS   = 32;
  static const uint32_t HEARTBEAT_MS = 500;
  static const uint32_t TIMEOUT_MS   = 1500;
  static const uint32_t REFRESH_MS   = 1000;   // 0 = Refresh aus (setRefreshInterval)

  SerialSync(Stream &port, bool isMaster) : _s(port), _master(isMaster) {}

  bool addBool(uint8_t id, bool &v, SyncDir dir)                         { return reg(id, T_BOOL,  &v, dir, 0.0f); }
  bool addInt(uint8_t id, int32_t &v, SyncDir dir)                       { return reg(id, T_INT,   &v, dir, 0.0f); }
  bool addFloat(uint8_t id, float &v, SyncDir dir, float threshold = 0)  { return reg(id, T_FLOAT, &v, dir, threshold); }

  // Serial-Port muss vorher mit begin() initialisiert sein
  void begin() {
    for (uint8_t i = 0; i < _count; i++) _vars[i].shadow = readRaw(_vars[i]);
    _started = false;
    tryStart();
  }

  // im loop() moeglichst oft aufrufen
  void update() {
    receive();
    if (!_started) {
      tryStart();
      if (!_started) return;          // Startverzoegerung laeuft: nichts senden
    }
    sendChanges();
    uint32_t now = millis();
    if (now - _lastHb >= HEARTBEAT_MS) {
      _lastHb = now;
      sendPacket(0, T_HEARTBEAT, 0);
    }
    const uint32_t interval = (_burstMs && (now - _startedAt) < _burstMs) ? 100 : _refreshMs;
    if (interval && now - _lastRefresh >= interval) {
      _lastRefresh = now;
      refreshSend();
    }
  }

  void setRefreshInterval(uint32_t ms) { _refreshMs = ms; }

  // Sende-Start verzoegern: Es wird NICHTS gesendet, bis millis() >= ms (Zeit seit Prozessorstart,
  // die Dauer von setup() zaehlt also mit). Empfangen laeuft normal. Danach: alle eigenen Werte + Request.
  // Vor begin() aufrufen. 0 = sofort (Standard).
  void setStartupDelay(uint32_t ms) { _startDelayMs = ms; }

  // Nach dem Start fuer 'ms' Millisekunden alle 100 ms alle SEND-Werte wiederholen (Start entzerren).
  void setStartupBurst(uint32_t ms) { _burstMs = ms; }

  bool started() const { return _started; }

  // Diagnose: kommen Bytes an? Sind die Pakete gueltig?
  uint32_t rxBytes() const { return _rxBytes; }   // alle empfangenen Bytes
  uint32_t rxOk()    const { return _rxOk; }      // Pakete mit richtiger Pruefsumme
  uint32_t rxBad()   const { return _rxBad; }     // Pakete mit falscher Pruefsumme

  bool connected() const { return _gotRx && (millis() - _lastRx) < TIMEOUT_MS; }

  void sendAll() { sendOwned(); }

private:
  enum : uint8_t { T_BOOL = 1, T_INT = 2, T_FLOAT = 3, T_REQUEST = 0xF0, T_HEARTBEAT = 0xF1 };
  static const uint8_t SYNC_BYTE = 0xAA;
  static const uint8_t PKT       = 8;

  struct Var {
    uint8_t  id, type;
    void    *ptr;
    SyncDir  dir;
    uint32_t shadow;
    float    threshold;
  };

  Stream  &_s;
  bool     _master;
  Var      _vars[MAX_VARS];
  uint8_t  _count = 0;
  uint8_t  _buf[PKT];
  uint8_t  _n = 0;
  uint32_t _lastHb = 0, _lastRx = 0, _lastRefresh = 0;
  uint32_t _refreshMs = REFRESH_MS;
  uint32_t _rxBytes = 0, _rxOk = 0, _rxBad = 0;
  uint32_t _startDelayMs = 0, _burstMs = 0, _startedAt = 0;
  bool     _started = false;
  bool     _gotRx = false;

  bool reg(uint8_t id, uint8_t type, void *ptr, SyncDir dir, float thr) {
    if (_count >= MAX_VARS) return false;
    _vars[_count++] = { id, type, ptr, dir, 0, thr };
    return true;
  }

  static uint8_t crc8(const uint8_t *d, uint8_t len) {
    uint8_t c = 0;
    while (len--) {
      c ^= *d++;
      for (uint8_t i = 0; i < 8; i++) c = (c & 0x80) ? (c << 1) ^ 0x07 : (c << 1);
    }
    return c;
  }

  static uint32_t readRaw(const Var &v) {
    uint32_t raw = 0;
    if (v.type == T_BOOL) raw = *(bool *)v.ptr ? 1 : 0;
    else                  memcpy(&raw, v.ptr, 4);
    return raw;
  }

  static void writeRaw(Var &v, uint32_t raw) {
    if (v.type == T_BOOL) *(bool *)v.ptr = (raw != 0);
    else                  memcpy(v.ptr, &raw, 4);
    v.shadow = raw;
  }

  static bool changed(const Var &v, uint32_t now) {
    if (v.type == T_FLOAT && v.threshold > 0) {
      float a, b;
      memcpy(&a, &now, 4);
      memcpy(&b, &v.shadow, 4);
      return fabsf(a - b) >= v.threshold;
    }
    return now != v.shadow;
  }

  bool mayWrite(const Var &v) const { return v.dir != SYNC_RECEIVE; }
  bool isOwned(const Var &v) const  { return v.dir == SYNC_SEND || (v.dir == SYNC_BOTH && _master); }

  void tryStart() {
    if (_started || millis() < _startDelayMs) return;
    _started   = true;
    _startedAt = millis();
    sendOwned();
    sendPacket(0, T_REQUEST, 0);
    _lastHb = _lastRefresh = _startedAt;
  }

  void sendPacket(uint8_t id, uint8_t type, uint32_t raw) {
    if (!_started) return;            // waehrend der Startverzoegerung bleibt die Leitung still
    uint8_t p[PKT];
    p[0] = SYNC_BYTE; p[1] = id; p[2] = type;
    memcpy(&p[3], &raw, 4);
    p[7] = crc8(&p[1], 6);
    _s.write(p, PKT);
  }

  void sendOwned() {
    for (uint8_t i = 0; i < _count; i++) {
      if (!isOwned(_vars[i])) continue;
      _vars[i].shadow = readRaw(_vars[i]);
      sendPacket(_vars[i].id, _vars[i].type, _vars[i].shadow);
    }
  }

  // Nur SEND-Variablen wiederholen (BOTH nicht, sonst koennte ein Refresh eine
  // gerade geaenderte Variable der Gegenseite ueberschreiben).
  void refreshSend() {
    for (uint8_t i = 0; i < _count; i++) {
      if (_vars[i].dir != SYNC_SEND) continue;
      _vars[i].shadow = readRaw(_vars[i]);
      sendPacket(_vars[i].id, _vars[i].type, _vars[i].shadow);
    }
  }

  void sendChanges() {
    for (uint8_t i = 0; i < _count; i++) {
      if (!mayWrite(_vars[i])) continue;
      uint32_t now = readRaw(_vars[i]);
      if (changed(_vars[i], now)) {
        _vars[i].shadow = now;
        sendPacket(_vars[i].id, _vars[i].type, now);
      }
    }
  }

  void handlePacket(const uint8_t *p) {
    uint8_t id = p[1], type = p[2];
    const bool wasConnected = connected();
    _lastRx = millis();
    _gotRx  = true;
    _rxOk++;
    // Anfrage oder Gegenseite taucht (wieder) auf -> alle eigenen Werte (nochmal) senden
    if (type == T_REQUEST || !wasConnected) sendOwned();
    if (type == T_HEARTBEAT || type == T_REQUEST) return;
    uint32_t raw;
    memcpy(&raw, &p[3], 4);
    for (uint8_t i = 0; i < _count; i++) {
      if (_vars[i].id == id && _vars[i].type == type) {
        if (_vars[i].dir != SYNC_SEND) writeRaw(_vars[i], raw);
        return;
      }
    }
  }

  void receive() {
    while (_s.available()) {
      uint8_t b = _s.read();
      _rxBytes++;
      if (_n == 0 && b != SYNC_BYTE) continue;
      _buf[_n++] = b;
      if (_n == PKT) {
        if (crc8(&_buf[1], 6) == _buf[7]) {
          handlePacket(_buf);
          _n = 0;
        } else {
          _rxBad++;
          uint8_t k = 1;
          while (k < PKT && _buf[k] != SYNC_BYTE) k++;
          memmove(_buf, _buf + k, PKT - k);
          _n = PKT - k;
        }
      }
    }
  }
};