import sensor, image, time, math
from pyb import UART
import pyb

# =========================
# Einstellungen
# =========================
TARGET_GOAL = 'B'      # 'B' (blau) oder 'Y' (gelb)

DEBUG_DRAW  = True     # Rechteck/Kreuz ins Bild zeichnen (kostet FPS)
DEBUG_PRINT = False    # print() pro Frame (kostet viel FPS, nur zum Testen an)

# -------------------------
# Auflösung & Streifen-ROI
# -------------------------
USE_QVGA = True        # True = 320x240 (präziser), False = 160x120 (schneller)

if USE_QVGA:
    FRAME = sensor.QVGA
    BASE_W, BASE_H = 320, 240
else:
    FRAME = sensor.QQVGA
    BASE_W, BASE_H = 160, 120

STRIP_H = 40           # Streifenhöhe: kleiner = schneller
Y_START = 120          # Position des Streifens (anpassen!)
X_START = 0
STRIP_W = BASE_W

CX0 = STRIP_W / 2.0    # Bildmitte im gecroppten Bild

# -------------------------
# Sensor Setup
# -------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(FRAME)
sensor.set_windowing((X_START, Y_START, STRIP_W, STRIP_H))  # NACH framesize
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

red_led = pyb.LED(2)

# UART1: P0=TX, P1=RX
# UART3: P4=TX, P5=RX
uart = UART(1, 115200, timeout_char=20)

# =========================
# Weitwinkelkorrektur
# =========================
USE_LENS_CORR = False
LENS_STRENGTH = 2.2
LENS_ZOOM = 1.0

# =========================
# Thresholds
# =========================
TH_BLUE   = (21, 86, 20, 127, -5, 86)
TH_YELLOW = (48, 71, 29, 127, -128, 127)

PIXELS_THR  = 60
AREA_THR    = 60
MERGE_BLOBS = True
X_STRIDE    = 2        # >1 beschleunigt find_blobs (Blobs müssen breiter sein)
Y_STRIDE    = 1

# Bearing-Parameter
FX = 120.0             # Brennweite in Pixel - kalibrieren!

# Randkorrektur ab +-40 Grad
CORR_START = 40.0
CORR_RANGE = 15.0
CORR_GAIN  = 0.3

# =========================
# Ziel-Auswahl (einmalig, nicht pro Frame)
# =========================
if TARGET_GOAL == 'B':
    TH_TARGET  = TH_BLUE
    DRAW_COLOR = (0, 0, 255)
    GOAL_CHAR  = 'B'
else:
    TH_TARGET  = TH_YELLOW
    DRAW_COLOR = (255, 255, 0)
    GOAL_CHAR  = 'Y'

RAD2DEG = 180.0 / math.pi

# =========================
# Hilfsfunktionen
# =========================
def val(x):
    """Blob-Eigenschaft lesen - funktioniert mit alter (Methode) UND
    neuer Firmware (Attribut)."""
    return x() if callable(x) else x

def largest_blob(blobs):
    best = None
    best_px = 0
    for b in blobs:
        px = val(b.pixels)
        if px > best_px:
            best_px = px
            best = b
    return best

def edge_correction(angle_deg):
    """Vergrößert Winkel ab +-40 Grad (symmetrisch). Gibt (winkel, faktor) zurück."""
    a = abs(angle_deg)
    if a <= CORR_START:
        return angle_deg, 0.0
    factor = 1.0 + CORR_GAIN * ((a - CORR_START) / CORR_RANGE)
    a *= factor
    return (a if angle_deg > 0 else -a), factor

# =========================
# Hauptschleife
# =========================
clock = time.clock()
red_led.on()

while True:
    clock.tick()
    img = sensor.snapshot()

    if USE_LENS_CORR:
        img.lens_corr(strength=LENS_STRENGTH, zoom=LENS_ZOOM)

    # Nur das Zielobjekt suchen (halbiert die Rechenzeit gegenüber 2 Suchen)
    blobs = img.find_blobs([TH_TARGET],
                           pixels_threshold=PIXELS_THR,
                           area_threshold=AREA_THR,
                           merge=MERGE_BLOBS,
                           x_stride=X_STRIDE,
                           y_stride=Y_STRIDE)

    b = largest_blob(blobs) if blobs else None

    if b is None:
        uart.write("N\n")
        red_led.off()
        continue

    red_led.on()

    cx = val(b.cx)
    cy = val(b.cy)
    blob_h = val(b.h)

    if DEBUG_DRAW:
        rect = (val(b.x), val(b.y), val(b.w), blob_h)
        img.draw_rectangle(rect, color=DRAW_COLOR, thickness=2)
        try:
            img.draw_cross(cx, cy, color=DRAW_COLOR, thickness=2)
        except TypeError:
            img.draw_cross((cx, cy), color=DRAW_COLOR, thickness=2)

    # Bearing relativ zur Kamera-Vorwärtsachse (rechts +, links -)
    angle_deg = math.atan((cx - CX0) / FX) * RAD2DEG
    angle_deg, corr = edge_correction(angle_deg)

    if DEBUG_PRINT:
        print("Angle: %.2f, Hoehe: %d, Corr: %.3f, FPS: %.1f" %
              (angle_deg, blob_h, corr, clock.fps()))

    uart.write("A,%s,%.2f,%d\n" % (GOAL_CHAR, angle_deg, blob_h))
