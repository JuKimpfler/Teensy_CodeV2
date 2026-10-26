#!/usr/bin/env python3
"""
sync_pds.py -- holt die PDS-Teensy-Bibliothek aus dem Robotic_PDS-Repository
nach lib/PDS.

lib/PDS ist eine Kopie ("vendored") von Robotic_PDS/teensy_firmware. Bis auf
zwei Punkte ist die Kopie identisch mit dem Original:

  1. params.h heisst hier params_PDS.h.
     Grund: lib/include/params.h ist die params.h des Roboters. Zwei Dateien
     gleichen Namens im selben Include-Pfad wuerden sich gegenseitig
     verdecken, und PDS.cpp bekaeme die falsche.
  2. Der Block mit den projektspezifischen I2C-Defines (I2C_BNO, BNO_ADDRESS,
     ...) faellt weg -- die Makros kommen hier aus lib/include/params.h.

channel_config.h wird NICHT ueberschrieben: darin steht die Kanal-, Parameter-
und Overlay-Belegung dieses Roboters. Weicht sie von der Vorlage ab, meldet
das Skript das nur.

Aufruf:
    python tools/sync_pds.py                 # kopieren
    python tools/sync_pds.py --check         # nur melden (Exit 1 = Drift)
    python tools/sync_pds.py --upstream PFAD # andere Quelle
"""

from __future__ import annotations

import argparse
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB_DIR = os.path.join(REPO_ROOT, "lib", "PDS")
DEFAULT_UPSTREAM = os.path.normpath(
    os.path.join(REPO_ROOT, "..", "Robotic_PDS", "teensy_firmware")
)

# Zeilen, die beim Uebernehmen von params.h verschwinden.
_I2C_COMMENT = re.compile(r"Projektspezifische I2C-Zuordnung")
_I2C_DEFINE = re.compile(r"^\s*#define\s+(I2C_[A-Za-z0-9_]+|BNO_ADDRESS)\b")


def _read(path: str) -> tuple[str, str]:
    """Datei als LF-Text lesen; gibt (text, zeilenende_der_quelle) zurueck."""
    raw = open(path, "rb").read().decode("utf-8")
    newline = "\r\n" if "\r\n" in raw else "\n"
    return raw.replace("\r\n", "\n"), newline


def _write(path: str, text: str, newline: str) -> None:
    open(path, "wb").write(text.replace("\n", newline).encode("utf-8"))


def rename_params(text: str) -> str:
    """params.h -> params_PDS.h (Include und die Verweise in den Kommentaren)."""
    text = text.replace('#include "params.h"', '#include "params_PDS.h"')
    return re.sub(r"\bparams\.h\b", "params_PDS.h", text)


def strip_project_defines(text: str) -> str:
    """Den I2C-Block aus params.h entfernen, samt seiner Leerzeilen."""
    out: list[str] = []
    dropped = False
    for line in text.split("\n"):
        if _I2C_COMMENT.search(line) or _I2C_DEFINE.match(line):
            dropped = True
            continue
        blank = not line.strip()
        last_blank = bool(out) and not out[-1].strip()
        if blank and dropped and last_blank:
            continue           # Leerzeile, die nur durch das Streichen entstand
        out.append(line)
        if not blank:
            dropped = False
    return "\n".join(out)


def params_header(text: str) -> str:
    return rename_params(strip_project_defines(text))


# Quelle (relativ zum Upstream-Ordner) -> Ziel (in lib/PDS), Umformer
FILES = [
    ("src/PDS.cpp", "PDS.cpp", None),
    ("src/PDS.h", "PDS.h", rename_params),
    ("src/params.h", "params_PDS.h", params_header),
    ("src/main.cpp", "example.cpp", rename_params),
    ("README.md", "README.md", None),
]

# Diese Datei gehoert dem Roboter und wird nie ueberschrieben.
PROTECTED = ("src/channel_config.h", "channel_config.h")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--upstream", default=DEFAULT_UPSTREAM,
                    help="Pfad auf Robotic_PDS/teensy_firmware (Default: %(default)s)")
    ap.add_argument("--check", action="store_true",
                    help="nichts schreiben, nur melden; Exitcode 1 bei Unterschieden")
    args = ap.parse_args()

    upstream = os.path.abspath(args.upstream)
    if not os.path.isdir(upstream):
        print(f"FEHLER: Quellverzeichnis nicht gefunden: {upstream}", file=sys.stderr)
        print("        Pfad mit --upstream angeben.", file=sys.stderr)
        return 2

    print(f"Quelle: {upstream}")
    print(f"Ziel  : {LIB_DIR}\n")

    changed: list[str] = []

    for src_rel, dst_name, transform in FILES:
        src = os.path.join(upstream, src_rel)
        dst = os.path.join(LIB_DIR, dst_name)
        if not os.path.isfile(src):
            print(f"  {dst_name:<18} WARNUNG: fehlt in der Quelle, uebersprungen")
            continue

        text, newline = _read(src)
        if transform:
            text = transform(text)

        old = _read(dst)[0] if os.path.isfile(dst) else None
        if old == text:
            print(f"  {dst_name:<18} unveraendert")
            continue

        changed.append(dst_name)
        if args.check:
            print(f"  {dst_name:<18} WUERDE sich aendern")
        else:
            _write(dst, text, newline)
            print(f"  {dst_name:<18} aktualisiert")

    # channel_config.h: nur vergleichen, nie kopieren.
    cc_src = os.path.join(upstream, PROTECTED[0])
    cc_dst = os.path.join(LIB_DIR, PROTECTED[1])
    if os.path.isfile(cc_src) and os.path.isfile(cc_dst):
        if _read(cc_src)[0] != _read(cc_dst)[0]:
            print("\n  channel_config.h weicht von der Vorlage ab. Das ist der")
            print("  Normalfall -- hier steht die Belegung dieses Roboters, die")
            print("  Datei wird nicht ueberschrieben. Neue Abschnitte der Vorlage")
            print("  bei Bedarf von Hand nachziehen:")
            print(f'      git diff --no-index "{cc_dst}" "{cc_src}"')

    print()
    if not changed:
        print("lib/PDS ist auf dem Stand von Robotic_PDS.")
        return 0
    if args.check:
        print("Unterschiede in: " + ", ".join(changed))
        return 1
    print("Aktualisiert: " + ", ".join(changed))
    print("Jetzt 'pio run' laufen lassen und die Aenderungen committen.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
