# Pathfinder Telemetrie-Viewer (PyQt6)

Analyse-GUI für die `PathfinderTelemetry`-Arduino-Bibliothek. Empfängt
Karten-Snapshots, gefundene Pfade und Iterations-Metadaten über eine
serielle Verbindung und zeigt sie live an.

## Setup

```bash
python3 -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate
pip install -r requirements.txt
python pathfinder_viewer.py
```

## Bedienung

1. Teensy mit dem Sketch `lib/Pathfinder/examples/Pathfinder_Gui.cpp` anschließen (als `src/main.cpp` kopieren).
2. Im Dropdown den seriellen Port auswählen (ggf. "Aktualisieren" klicken,
   falls das Board erst nachträglich angesteckt wurde).
3. Baudrate einstellen (muss zu `Serial.begin(...)` im Sketch passen; bei
   nativem USB von Teensy 4.0 / ESP32-S3 ist der genaue Wert meist egal).
4. "Verbinden" klicken.

Links erscheint die Karte als Heatmap (grün = günstig, grau = neutral,
orange/rot = teuer, fast schwarz = Wand/nicht begehbar), mit dem
gefundenen Pfad als blaue Linie sowie Start- (grün, "S") und
Zielmarkierung (rot, "Z"). Rechts stehen die aktuellen Iterationsdaten,
ein Verlaufsdiagramm (Dauer/Pfadlänge über Zeit) sowie ein Log für
Verbindungs-/Parsefehler.

## Hinweise

- Das erste vollständige Kartenbild kann je nach Baudrate/USB-Modus einen
  Moment dauern (ein 90x120-Feld sind 10800 Rohbytes plus Header) - für
  einen ersten Funktionstest ist das erwartet und unkritisch.
- Reguläre `Serial.println(...)`-Debugausgaben aus dem Sketch stören die
  Anzeige nicht; der Parser sucht aktiv nach den Sync-Bytes und
  überspringt alles andere.
- Bei Prüfsummenfehlern (z.B. durch USB-Aussetzer) wird der betroffene
  Frame verworfen und im Log vermerkt; die Anzeige läuft mit dem nächsten
  gültigen Frame weiter.
