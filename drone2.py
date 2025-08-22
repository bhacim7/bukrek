# -*- coding: utf-8 -*-
# Gerekli kütüphaneleri içe aktar
import cv2
import numpy as np
import serial
import time
from pymavlink import mavutil
import os

# --- Seri Port ve Telemetri Ayarları ---
# RC alıcınız için seri portu ayarlayın. Genellikle Raspberry Pi'de bu /dev/ttyS0'dır.
RC_SERIAL_PORT = "/dev/ttyS0"
# Kumandanızın i-BUS baud hızı, genellikle 115200'dür.
BAUD_RATE = 115200             

# MAVLink bağlantısı için seri portu ayarlayın.
TELEMETRY_PORT = "/dev/ttyUSB0"
TELEMETRY_BAUD = 57600 

# Renk tespiti için kullanılacak RC kanalını ayarlayın.
TRIGGER_CHANNEL = 5

# MAVLink bağlantısını oluştur. Hata durumunda kodun çalışmasını durdurmaz.
master = None
try:
    master = mavutil.mavlink_connection(TELEMETRY_PORT, baud=TELEMETRY_BAUD)
    print("MAVLink bağlantısı bekleniyor...")
    master.wait_heartbeat(timeout=5)
    print("MAVLink bağlantısı başarıyla kuruldu.")
except Exception as e:
    print(f"MAVLink bağlantısı başarısız oldu: {e}")
    master = None

# --- Kamera ve Görüntü Ayarları ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_WB, 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Ortama göre bu değişkeni True/False olarak ayarlayın
USE_OUTDOOR = False 

# Sizin daha iyi sonuç aldığınız renk aralıklarını kullanıyoruz
if USE_OUTDOOR:
    lower_red1, upper_red1 = (0, 140, 110), (10, 255, 255)
    lower_red2, upper_red2 = (170, 140, 110), (179, 255, 255)
    lower_green, upper_green = (40, 120, 110), (85, 255, 255)
    lower_black, upper_black = (0, 0, 0), (179, 70, 45)
else:
    lower_red1, upper_red1 = (0, 120, 80), (10, 255, 255)
    lower_red2, upper_red2 = (170, 120, 80), (179, 255, 255)
    lower_green, upper_green = (36, 80, 80), (85, 255, 255)
    lower_black, upper_black = (0, 0, 0), (179, 80, 55)

kernel = np.ones((5,5), np.uint8)

def clean(mask):
    """Morfolojik işlemlerle (açma ve kapatma) gürültüyü temizler."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask

def detect_color(roi):
    """ROI'deki en baskın rengi (Siyah, Kırmızı, Yeşil veya Belirsiz) tespit eder."""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    mask_red = clean(mask_red)
    mask_green = clean(mask_green)
    mask_black = clean(mask_black)
    
    min_area = int(roi.shape[0]*roi.shape[1]*0.005)

    def max_contour_area(mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return 0
        return max((cv2.contourArea(c) for c in cnts))

    red_area = max_contour_area(mask_red)
    green_area = max_contour_area(mask_green)
    black_area = max_contour_area(mask_black)
    
    areas = {"KIRMIZI": red_area, "YESIL": green_area, "SIYAH": black_area}
    max_area_label = max(areas, key=areas.get)
    max_area_value = areas[max_area_label]
    
    if max_area_value > min_area:
        return max_area_label, max_area_value / (roi.shape[0] * roi.shape[1])
    else:
        return "BELIRSIZ", 0.0

def send_mavlink_message(label, conf):
    """Tespit edilen renk ve güvenilirlik ile MAVLink STATUSTEXT mesajı gönderir."""
    if master is None:
        return
    message = f"Tespit Edilen: {label} (Guven: {conf:.2f})"
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, message.encode())

# --- Ana Döngü ---
ser = None
try:
    ser = serial.Serial(RC_SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
except serial.SerialException as e:
    ser = None
except FileNotFoundError:
    ser = None

last_detected_color = "BELIRSIZ"
last_detected_conf = 0.0

# Terminali temizle ve başlık yazdır
def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

clear_screen()
print("-------------------------------------")
print("  Canlı Renk Tespiti ve Durum Ekranı  ")
print("-------------------------------------")

while True:
    ok, frame = cap.read()
    if not ok:
        print("\nKamera okunamadı. Program sonlandırılıyor.")
        break

    # Kamera görüntüsünü sürekli işle
    small_frame = cv2.resize(frame, (320, 240))
    h, w = small_frame.shape[:2]
    roi_ratio = 0.7
    x0 = int((1-roi_ratio)/2 * w); x1 = int((1+(roi_ratio))/2 * w)
    y0 = int((1-roi_ratio)/2 * h); y1 = int((1+(roi_ratio))/2 * h)
    roi = small_frame[y0:y1, x0:x1]

    # Renk tespiti sürekli olarak yapılır.
    current_color, current_conf = detect_color(roi)
    
    # Sadece tespit değiştiğinde güncellemeyi göster
    if current_color != last_detected_color:
        last_detected_color = current_color
        last_detected_conf = current_conf

    is_triggered = False
    if ser:
        try:
            ibus_frame = ser.read(32)
            if len(ibus_frame) == 32:
                channel_index = TRIGGER_CHANNEL - 1
                byte_index = 2 + (channel_index * 2)
                channel_value = ibus_frame[byte_index] | (ibus_frame[byte_index + 1] << 8)
                
                if channel_value > 1500:
                    is_triggered = True
                    send_mavlink_message(last_detected_color, last_detected_conf)
        except serial.SerialException as e:
            ser = None
        except Exception as e:
            pass
            
    # Kamera görüntüsü üzerinde ROI'yi ve bilgiyi göster
    out = frame.copy()
    h_orig, w_orig = frame.shape[:2]
    x0_orig = int((1-roi_ratio)/2 * w_orig); x1_orig = int((1+(roi_ratio))/2 * w_orig)
    y0_orig = int((1-roi_ratio)/2 * h_orig); y1_orig = int((1+(roi_ratio))/2 * h_orig)

    cv2.rectangle(out, (x0_orig,y0_orig), (x1_orig,y1_orig), (255,255,255), 2)
    cv2.putText(out, f"Son Renk: {last_detected_color}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.putText(out, f"Guven: {last_detected_conf:.3f}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    if is_triggered:
        cv2.putText(out, "RC Tetikleme: AKTIF!", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    else:
        cv2.putText(out, "RC Tetikleme: Boşta", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    
    cv2.imshow("Canlı Kamera Görüntüsü", out)

    # Terminali temizle ve yeni durumu yazdır
    print("\033[H\033[J")
    print("-------------------------------------")
    print("  Canlı Renk Tespiti ve Durum Ekranı  ")
    print("-------------------------------------")
    print(f"Son Tespit Edilen Renk: {last_detected_color}")
    print(f"Güvenilirlik Oranı: {last_detected_conf:.3f}")
    
    if not ser:
        print("\nSeri Port Bağlantısı: HATA")
    else:
        print(f"\nSeri Port Bağlantısı: OK ({RC_SERIAL_PORT})")

    if master:
        print(f"MAVLink Bağlantısı: OK ({TELEMETRY_PORT})")
    else:
        print(f"MAVLink Bağlantısı: HATA")

    if is_triggered:
        print("\nRC Tetikleme Durumu: AKTİF")
    else:
        print("\nRC Tetikleme Durumu: Boşta")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()
