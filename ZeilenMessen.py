import os
import time

def count_lines_in_files(directory, extensions=('.cpp', '.c', '.h'), exclude_dirs=None):
    if exclude_dirs is None:
        exclude_dirs = []

    total_lines = 0
    file_count = 0

    # Normalisiere auszuschließende Ordnerpfade
    exclude_dirs = [os.path.abspath(os.path.join(directory, d)) for d in exclude_dirs]

    for root, dirs, files in os.walk(directory):
        abs_root = os.path.abspath(root)

        # Verzeichnisse filtern, die ausgeschlossen werden sollen
        if any(abs_root.startswith(excl) for excl in exclude_dirs):
            continue

        for file in files:
            if file.endswith(extensions):
                filepath = os.path.join(root, file)
                try:
                    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                        lines = f.readlines()
                        line_count = len(lines)
                        total_lines += line_count
                        file_count += 1
                        print(f"{filepath}: {line_count} Zeilen")
                except Exception as e:
                    print(f"Fehler beim Lesen von {filepath}: {e}")

    print(f"\nGesamtanzahl Dateien: {file_count}")
    print(f"Gesamtanzahl Zeilen: {total_lines}")

# Beispielaufruf
count_lines_in_files(
    directory="C:/Users/hp/Desktop/AG/teensy_code",
    exclude_dirs=[".pio", "include", ".vscode", "pcbs", ".git" ]
)

time.sleep(20000)
