#!/usr/bin/env python3
"""
pathfinder_viewer.py
----------------------
PyQt6-GUI zum Anzeigen von Karten/Overlay/Pfaden, die ein Arduino
(Teensy 4.0 / ESP32-S3) via Serial im PathfinderTelemetry-Protokoll
sendet (siehe PathfinderTelemetry.h).

Start:
    pip install -r requirements.txt
    python pathfinder_viewer.py

Dann rechts oben den seriellen Port waehlen, Baudrate einstellen
(muss zur Serial.begin()-Baudrate im Sketch passen - bei nativem USB von
Teensy/ESP32-S3 spielt der Wert meist keine Rolle) und "Verbinden" klicken.
"""

import sys
import struct
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QRectF, QPointF
from PyQt6.QtGui import QImage, QPainter, QPen, QColor, QPolygonF
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QComboBox, QPushButton,
    QSpinBox, QHBoxLayout, QVBoxLayout, QSplitter, QGroupBox, QFormLayout,
    QTextEdit, QSizePolicy,
)


# ----------------------------------------------------------------------
# Protokoll / Framing
# ----------------------------------------------------------------------

SYNC1 = 0xA5
SYNC2 = 0x5A
HEADER_LEN = 5  # sync1, sync2, type, len_lo, len_hi

FRAME_FIELD = ord('F')
FRAME_PATH = ord('P')
FRAME_META = ord('M')


class FrameParser:
    """Inkrementeller Parser fuer das PathfinderTelemetry-Framing.

    feed(bytes) wird mit beliebig grossen/kleinen Byte-Chunks aufgerufen
    und liefert eine Liste aus (type, payload) Tupeln fuer alle in diesem
    Aufruf vollstaendig gewordenen, gueltigen Frames zurueck. Kaputte
    Frames (Checksummenfehler) werden verworfen, der Parser resynchronisiert
    sich automatisch anhand der Sync-Bytes.
    """

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []

        while True:
            idx = self.buf.find(bytes([SYNC1, SYNC2]))
            if idx == -1:
                # Kein Sync gefunden: nur letztes Byte behalten (koennte
                # der Anfang von SYNC1 SYNC2 sein), Rest verwerfen.
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < HEADER_LEN:
                break  # Header noch nicht vollstaendig da

            ftype = self.buf[2]
            length = self.buf[3] | (self.buf[4] << 8)
            total_len = HEADER_LEN + length + 1  # +1 Checksumme

            if len(self.buf) < total_len:
                break  # Payload/Checksumme noch nicht vollstaendig da

            payload = bytes(self.buf[HEADER_LEN:HEADER_LEN + length])
            checksum = self.buf[HEADER_LEN + length]
            del self.buf[:total_len]

            calc = sum(payload) & 0xFF
            if calc == checksum:
                frames.append((ftype, payload))
            # bei Checksummenfehler: Frame verwerfen, weitersuchen

        return frames


def parse_field(payload: bytes):
    w = payload[0] | (payload[1] << 8)
    h = payload[2] | (payload[3] << 8)
    expected = 4 + w * h
    if len(payload) < expected:
        raise ValueError("field payload too short")
    arr = np.frombuffer(payload, dtype=np.uint8, count=w * h, offset=4)
    return arr.reshape((w, h)).copy(), w, h


def parse_path(payload: bytes):
    count = payload[0] | (payload[1] << 8)
    pts = []
    for i in range(count):
        x = payload[2 + i * 2]
        y = payload[2 + i * 2 + 1]
        pts.append((x, y))
    return pts


def parse_meta(payload: bytes):
    iteration, duration_ms = struct.unpack_from('<II', payload, 0)
    (path_length,) = struct.unpack_from('<H', payload, 8)
    start_x, start_y, target_x, target_y, path_found = payload[10:15]
    return {
        'iteration': iteration,
        'duration_ms': duration_ms,
        'path_length': path_length,
        'start': (start_x, start_y),
        'target': (target_x, target_y),
        'path_found': bool(path_found),
    }


# ----------------------------------------------------------------------
# Serieller Empfangs-Thread
# ----------------------------------------------------------------------

class SerialWorker(QThread):
    fieldReceived = pyqtSignal(object, int, int)  # arr, w, h
    pathReceived = pyqtSignal(list)
    metaReceived = pyqtSignal(dict)
    errorOccurred = pyqtSignal(str)
    connectionChanged = pyqtSignal(bool)

    def __init__(self, port_name: str, baud: int):
        super().__init__()
        self.port_name = port_name
        self.baud = baud
        self._running = False
        self.parser = FrameParser()

    def run(self):
        try:
            ser = serial.Serial(self.port_name, self.baud, timeout=0.1)
        except Exception as exc:
            self.errorOccurred.emit(f"Konnte {self.port_name} nicht oeffnen: {exc}")
            return

        self._running = True
        self.connectionChanged.emit(True)
        try:
            while self._running:
                try:
                    chunk = ser.read(4096)
                except Exception as exc:
                    self.errorOccurred.emit(f"Lesefehler: {exc}")
                    break
                if chunk:
                    for ftype, payload in self.parser.feed(chunk):
                        self._dispatch(ftype, payload)
                else:
                    self.msleep(1)
        finally:
            ser.close()
            self.connectionChanged.emit(False)

    def stop(self):
        self._running = False
        self.wait(2000)

    def _dispatch(self, ftype: int, payload: bytes):
        try:
            if ftype == FRAME_FIELD:
                arr, w, h = parse_field(payload)
                self.fieldReceived.emit(arr, w, h)
            elif ftype == FRAME_PATH:
                self.pathReceived.emit(parse_path(payload))
            elif ftype == FRAME_META:
                self.metaReceived.emit(parse_meta(payload))
        except Exception as exc:
            self.errorOccurred.emit(f"Parse-Fehler ({chr(ftype)}): {exc}")


# ----------------------------------------------------------------------
# Farb-Mapping fuer Gewichte (0..255, 255 = nicht begehbar)
# ----------------------------------------------------------------------

_STOPS = np.array([0, 100, 200, 254, 255])
_R = np.array([46, 149, 230, 231, 20])
_G = np.array([204, 165, 126, 76, 20])
_B = np.array([113, 166, 34, 60, 20])


def field_to_qimage(field: np.ndarray) -> QImage:
    flat = field.astype(np.float64).ravel()
    r = np.interp(flat, _STOPS, _R).astype(np.uint8)
    g = np.interp(flat, _STOPS, _G).astype(np.uint8)
    b = np.interp(flat, _STOPS, _B).astype(np.uint8)
    rgb = np.dstack((r, g, b)).reshape(field.shape[0], field.shape[1], 3)
    rgb = np.ascontiguousarray(rgb)
    h, w, _ = rgb.shape
    img = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
    # Kopie erzwingen, da "rgb" nach Funktionsende freigegeben werden koennte
    return img.copy()


# ----------------------------------------------------------------------
# Karten-/Pfad-Widget
# ----------------------------------------------------------------------

class FieldView(QWidget):
    """Zeigt das Gewichtsfeld als Heatmap plus Pfad-Overlay und
    Start/Ziel-Marker. Koordinatenkonvention: field[x][y], x = Zeile
    (0..W-1), y = Spalte (0..H-1) - identisch zur Speicherung im Sketch."""

    def __init__(self):
        super().__init__()
        self.field = None
        self.path = []
        self.start = None
        self.target = None
        self.setMinimumSize(480, 360)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_field(self, arr, w, h):
        self.field = arr
        self.update()

    def set_path(self, pts):
        self.path = pts
        self.update()

    def set_meta(self, meta):
        self.start = meta.get('start')
        self.target = meta.get('target')
        self.update()

    def _fit_rect(self, img_w, img_h):
        avail_w = self.width() - 20
        avail_h = self.height() - 20
        scale = min(avail_w / img_w, avail_h / img_h)
        scale = max(scale, 0.01)
        w = img_w * scale
        h = img_h * scale
        x = (self.width() - w) / 2
        y = (self.height() - h) / 2
        return QRectF(x, y, w, h)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(24, 24, 24))

        if self.field is None:
            painter.setPen(QColor(170, 170, 170))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                              "Warte auf Kartendaten ...")
            return

        img = field_to_qimage(self.field)  # rows = W (x), cols = H (y)
        rect = self._fit_rect(img.width(), img.height())
        painter.drawImage(rect, img)

        sx = rect.width() / img.width()   # Skalierung fuer y-Achse (Spalten/Breite)
        sy = rect.height() / img.height()  # Skalierung fuer x-Achse (Zeilen/Hoehe)
        ox, oy = rect.x(), rect.y()

        def to_screen(px, py):
            # (px, py) = (x, y) im Feld-Koordinatensystem
            return QPointF(ox + py * sx + sx / 2.0, oy + px * sy + sy / 2.0)

        if len(self.path) >= 2:
            poly = QPolygonF([to_screen(p[0], p[1]) for p in self.path])
            painter.setPen(QPen(QColor(52, 152, 219), 2))
            painter.drawPolyline(poly)

        if self.start is not None:
            self._draw_marker(painter, to_screen(*self.start), QColor(46, 204, 113), "S")
        if self.target is not None:
            self._draw_marker(painter, to_screen(*self.target), QColor(231, 76, 60), "Z")

    @staticmethod
    def _draw_marker(painter: QPainter, center: QPointF, color: QColor, label: str):
        r = 6
        painter.setPen(QPen(QColor(20, 20, 20), 1.5))
        painter.setBrush(color)
        painter.drawEllipse(center, r, r)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(QRectF(center.x() - 10, center.y() - 24, 20, 16),
                          Qt.AlignmentFlag.AlignCenter, label)


# ----------------------------------------------------------------------
# Verlaufs-Diagramm (Dauer / Pfadlaenge ueber Iterationen)
# ----------------------------------------------------------------------

class HistoryChart(QWidget):
    def __init__(self, max_points=300):
        super().__init__()
        self.max_points = max_points
        self.durations = deque(maxlen=max_points)
        self.path_lengths = deque(maxlen=max_points)
        self.setMinimumHeight(160)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def add_point(self, duration_ms, path_length):
        self.durations.append(duration_ms)
        self.path_lengths.append(path_length)
        self.update()

    def clear(self):
        self.durations.clear()
        self.path_lengths.clear()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(24, 24, 24))
        painter.setPen(QColor(70, 70, 70))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))

        if not self.durations:
            painter.setPen(QColor(150, 150, 150))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Keine Daten")
            return

        margin = 22
        w, h = self.width(), self.height()

        def plot(values, color):
            maxv = max(values) if max(values) > 0 else 1
            step = (w - 2 * margin) / max(1, self.max_points - 1)
            pts = []
            n = len(values)
            offset = self.max_points - n
            for i, v in enumerate(values):
                x = margin + (offset + i) * step
                y = h - margin - (v / maxv) * (h - 2 * margin)
                pts.append(QPointF(x, y))
            painter.setPen(QPen(color, 2))
            painter.drawPolyline(QPolygonF(pts))

        plot(list(self.durations), QColor(52, 152, 219))
        plot(list(self.path_lengths), QColor(230, 126, 34))

        painter.setPen(QColor(52, 152, 219))
        painter.drawText(6, 14, f"Dauer (max {max(self.durations)} ms)")
        painter.setPen(QColor(230, 126, 34))
        painter.drawText(6, 30, f"Pfadlaenge (max {max(self.path_lengths)})")


# ----------------------------------------------------------------------
# Hauptfenster
# ----------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pathfinder Telemetrie-Viewer")
        self.resize(1200, 760)

        self.worker = None
        self.iteration_count = 0

        self._build_ui()
        self._refresh_ports()

    # -- UI Aufbau -------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Verbindungsleiste
        conn_bar = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("Aktualisieren")
        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(300, 4_000_000)
        self.baud_spin.setValue(2_000_000)
        self.baud_spin.setSingleStep(100)
        self.connect_btn = QPushButton("Verbinden")
        self.status_label = QLabel("Nicht verbunden")
        self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold;")

        conn_bar.addWidget(QLabel("Port:"))
        conn_bar.addWidget(self.port_combo, 1)
        conn_bar.addWidget(self.refresh_btn)
        conn_bar.addWidget(QLabel("Baud:"))
        conn_bar.addWidget(self.baud_spin)
        conn_bar.addWidget(self.connect_btn)
        conn_bar.addStretch(1)
        conn_bar.addWidget(self.status_label)
        root.addLayout(conn_bar)

        # Hauptbereich: Karte links, Stats/Diagramm/Log rechts
        splitter = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(splitter, 1)

        self.field_view = FieldView()
        splitter.addWidget(self.field_view)

        right = QWidget()
        right_layout = QVBoxLayout(right)

        stats_box = QGroupBox("Aktuelle Iteration")
        form = QFormLayout(stats_box)
        self.lbl_iteration = QLabel("-")
        self.lbl_duration = QLabel("-")
        self.lbl_pathlen = QLabel("-")
        self.lbl_start = QLabel("-")
        self.lbl_target = QLabel("-")
        self.lbl_found = QLabel("-")
        form.addRow("Iteration:", self.lbl_iteration)
        form.addRow("Dauer:", self.lbl_duration)
        form.addRow("Pfadlaenge:", self.lbl_pathlen)
        form.addRow("Start:", self.lbl_start)
        form.addRow("Ziel:", self.lbl_target)
        form.addRow("Pfad gefunden:", self.lbl_found)
        right_layout.addWidget(stats_box)

        chart_box = QGroupBox("Verlauf")
        chart_layout = QVBoxLayout(chart_box)
        self.history_chart = HistoryChart()
        chart_layout.addWidget(self.history_chart)
        right_layout.addWidget(chart_box)

        log_box = QGroupBox("Log")
        log_layout = QVBoxLayout(log_box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        right_layout.addWidget(log_box, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        # Signale
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._toggle_connection)

    # -- Ports -------------------------------------------------------
    def _refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    # -- Verbindung ----------------------------------------------------
    def _toggle_connection(self):
        if self.worker is not None:
            self._log("Trenne Verbindung ...")
            self.worker.stop()
            self.worker = None
            return

        port = self.port_combo.currentText()
        if not port:
            self._log("Kein serieller Port ausgewaehlt.")
            return

        baud = self.baud_spin.value()
        self.worker = SerialWorker(port, baud)
        self.worker.fieldReceived.connect(self._on_field)
        self.worker.pathReceived.connect(self._on_path)
        self.worker.metaReceived.connect(self._on_meta)
        self.worker.errorOccurred.connect(self._on_error)
        self.worker.connectionChanged.connect(self._on_connection_changed)
        self.worker.start()

    def _on_connection_changed(self, connected: bool):
        if connected:
            self.status_label.setText(f"Verbunden ({self.port_combo.currentText()})")
            self.status_label.setStyleSheet("color: #2ecc71; font-weight: bold;")
            self.connect_btn.setText("Trennen")
            self.history_chart.clear()
        else:
            self.status_label.setText("Nicht verbunden")
            self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
            self.connect_btn.setText("Verbinden")
            self.worker = None

    # -- Datenempfang --------------------------------------------------
    def _on_field(self, arr, w, h):
        self.field_view.set_field(arr, w, h)

    def _on_path(self, pts):
        self.field_view.set_path(pts)

    def _on_meta(self, meta):
        self.field_view.set_meta(meta)
        self.lbl_iteration.setText(str(meta['iteration']))
        self.lbl_duration.setText(f"{meta['duration_ms']} ms")
        self.lbl_pathlen.setText(str(meta['path_length']))
        self.lbl_start.setText(str(meta['start']))
        self.lbl_target.setText(str(meta['target']))
        self.lbl_found.setText("ja" if meta['path_found'] else "nein")
        self.history_chart.add_point(meta['duration_ms'], meta['path_length'])

    def _on_error(self, message: str):
        self._log(f"Fehler: {message}")

    def _log(self, text: str):
        self.log_view.append(text)

    def closeEvent(self, event):
        if self.worker is not None:
            self.worker.stop()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
