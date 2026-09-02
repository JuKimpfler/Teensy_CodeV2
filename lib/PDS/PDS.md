# PDS – Einbau in dieses Repository

Die eigentliche Anleitung zur Bibliothek steht in
[README.md](README.md) – die Datei wird aus dem Repository `Robotic_PDS`
mitkopiert und beschreibt die Bibliothek allgemein. Hier steht nur, was in
**diesem** Projekt anders ist.

---

## Herkunft und Aktualisierung

`lib/PDS/` ist eine Kopie von `Robotic_PDS/teensy_firmware`. Aktualisiert
wird sie nicht von Hand, sondern mit:

```bash
python tools/sync_pds.py
```

```bash
python tools/sync_pds.py --check
```

`--check` schreibt nichts und meldet nur, ob es Unterschiede gibt
(Exitcode 1 = ja). Liegt das PDS-Repository woanders, hilft `--upstream PFAD`.

---

## Die zwei Abweichungen vom Original

| Original | Hier | Warum |
|---|---|---|
| `src/params.h` | `params_PDS.h` | `lib/include/params.h` ist die `params.h` des Roboters. Zwei Dateien gleichen Namens im selben Include-Pfad würden sich gegenseitig verdecken. |
| I2C-Defines in `params.h` | entfallen | `I2C_BNO`, `BNO_ADDRESS` usw. kommen hier aus `lib/include/params.h`. |

Beides macht das Sync-Skript automatisch. `PDS.cpp` bleibt unverändert – die
Verweise auf „params.h" in seinen Kommentaren meinen also `params_PDS.h`.

`channel_config.h` wird vom Skript **nie** überschrieben: darin steht die
Belegung dieses Roboters. Wenn eine neue PDS-Version dort neue Abschnitte
mitbringt, meldet das Skript das und man zieht sie von Hand nach.

`example.cpp` ist der Beispiel-Sketch aus dem PDS-Repo. `library.json` hält
ihn per `srcFilter` aus dem Build heraus – er hat ein eigenes
`setup()`/`loop()`.

---

## Was der Roboter davon schon benutzt

* `systemlib::initialize()` ruft `PDS.init()` **und** `setupPDS()` auf.
  `setupPDS()` bindet die Systemwerte auf die festen Kanäle 0–25
  (Akku, Radar, Feldposition, Motoren, Zykluszeit) – siehe
  [docs/PDS_Belegung.md](../../docs/PDS_Belegung.md).
* `systemlib::readSensors()` ruft `PDS.update()` – **bedingungslos**, in
  jedem Zyklus. Das muss so sein: erst `update()` liest die RX-Leitung aus.
  Hinge der Aufruf an `PDS.linkOk()`, käme die Verbindung nach dem ersten
  Aussetzer nie wieder zustande.
* Im Sketch ist damit nichts mehr zu tun; eigene Werte kommen mit
  `PDS.plot()` / `PDS.track()` dazu (Kanäle ab 32).

---

## Was bewusst *nicht* eingeschaltet ist

| Funktion | Warum nicht |
|---|---|
| `PDS.enableWatchdog(ms)` | Der Teensy startet neu, wenn `update()` länger ausbleibt. `calibRadar()` blockiert pro Sensor bis ~2 s – das würde einen Reset auslösen. Wer den Watchdog will, muss in den blockierenden Stellen `PDS.feedWatchdog()` aufrufen. |
| `if (!PDS.linkOk()) motorenStopp();` | Sinnvoll beim Testen am Kabel, aber im Spiel läuft der Roboter ohne GUI. Ein Not-Aus an `linkOk()` würde ihn dann sofort stilllegen. |
| `PDS.enableSelfDiagnostics()` | Belegt die Kanäle 194–199. Bei Bedarf einfach in `setup()` aufrufen. |
