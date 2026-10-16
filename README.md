# Teensy Code Bodensee Koalas Saison 2027 [![pipeline status](https://git.markdorf-robotics.de/koalas/2vs2_soccer/teensy_code/badges/master/pipeline.svg)](https://git.markdorf-robotics.de/koalas/2vs2_soccer/teensy_code/-/pipelines)

## Soccer 2v2 Lightweight - die Liga
Wir spielen beim RoboCup in der Liga Soccer 2v2 Lightweight.

## Unser Ansatz
Innovativ, genial und nicht funktional

## Aufbau

| Ordner | Inhalt |
|---|---|
| `lib/systemlib` | Hauptlib: alle Sensoren und Aktuatoren, Position, Fahrfunktionen ([Doku](lib/systemlib/systemlib.md)) |
| `lib/include` | **Pins und alle Parameter** ([Übersicht](lib/include/include.md)) |
| `lib/Pathfinder` | A*-Wegsuche ([Doku](lib/Pathfinder/Pathfinder.md)) |
| `lib/Link`, `lib/SensorLink`, `esp/` | Link Teensy ↔ ESP32-S3 und die ESP-Firmware ([Link](lib/Link/Link.md), [ESP](esp/README.md), [Inbetriebnahme](docs/Inbetriebnahme_ESP.md)) |
| `lib/RadarArray`, `lib/XM125` | Radar-Abstandssensoren |
| `lib/Regler`, `lib/PDS`, `lib/…` | Regler, Power Debugger und weitere Sensor-/Hilfslibs, je mit `.md` |
| `tools/`, `test/native/` | `sync_pds.py`, PC-Oberfläche für den Pathfinder; Rechnertests (`test/native/run.sh`) |
| `docs/` | [Coding Rules](docs/Coding_Rules.md), [Plan externe Navigation-Lib](docs/Plan_Navigation-Lib.md), [Cleanup-Bericht](docs/Cleanup_Bericht.md) |

Formatieren vor dem Commit: `clang-format -i <Datei>` (Regeln in `.clang-format`).
