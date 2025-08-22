# -*- coding: utf-8 -*-
# Gerekli kütüphaneleri içe aktar
import cv2
import numpy as np
import time
from pymavlink import mavutil
import os
from gpiozero import DigitalInputDevice

# --- GPIO ve Telemetri Ayarları ---
# Düğmenin bağlı olduğu Raspberry Pi GPIO pinini BCM numarasıyla ayarlayın.
# Örneğin, GPIO 17 için 17 yazın.
TRIGGER_PIN = 17

# MAVLink bağlantısı için seri portu ayarlayın.
TELEMETRY_PORT = "/dev/ttyUSB0"
TELEMETRY_BAUD = 57600 

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
# Not: Sadece renk tespiti için kullanılıyor, görüntü gösterilmiyor.
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
# GPIO pinini en temel haliyle giriş olarak ayarla.
pin = DigitalInputDevice(TRIGGER_PIN)

last_detected_color = "BELIRSIZ"
last_detected_conf = 0.0
last_status = "Boşta"

# Terminali temizle ve başlık yazdır
def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

clear_screen()
print("-------------------------------------")
print("  Canlı Renk Tespiti ve Durum Ekranı  ")
print("-------------------------------------")
print(f"RC Tetikleme Pini: GPIO {TRIGGER_PIN}")

# Sinyal genişliğini ölçmek için daha kararlı bir fonksiyon
def get_reliable_pulse_width(pin, timeout=0.03):
    start_time = time.time()
    # Pinin HIGH olmasını bekle
    if pin.wait_for_active(timeout=timeout):
        # Pinin LOW olmasını bekle
        if pin.wait_for_inactive(timeout=timeout):
            return (time.time() - start_time) * 1000000
    return 0

while True:
    # Sinyal genişliğini (pulse width) ölç
    pulse_width_us = get_reliable_pulse_width(pin)

    # Sinyal genişliğine göre durumu belirle
    is_triggered = False
    if pulse_width_us > 1500:
        is_triggered = True
        current_status = "AKTİF"
    else:
        current_status = "Boşta"
    
    if current_status != last_status:
        last_status = current_status
    
    # Kamera görüntüsünü sürekli işle
    ok, frame = cap.read()
    if not ok:
        print("\nKamera okunamadı. Program sonlandırılıyor.")
        break
        
    small_frame = cv2.resize(frame, (320, 240))
    h, w = small_frame.shape[:2]
    roi_ratio = 0.7
    x0 = int((1-roi_ratio)/2 * w)
    x1 = int((1+roi_ratio)/2 * w)
    y0 = int((1-roi_ratio)/2 * h)
    y1 = int((1+roi_ratio)/2 * h)
    roi = small_frame[y0:y1, x0:x1]

    # Renk tespiti sürekli olarak yapılır.
    current_color, current_conf = detect_color(roi)
    
    # Sadece tespit değiştiğinde güncellemeyi göster
    if current_color != last_detected_color:
        last_detected_color = current_color
        last_detected_conf = current_conf

    if is_triggered:
        send_mavlink_message(last_detected_color, last_detected_conf)

    # Terminali temizle ve yeni durumu yazdır
    print("\033[H\033[J")
    print("-------------------------------------")
    print("  Canlı Renk Tespiti ve Durum Ekranı  ")
    print("-------------------------------------")
    print(f"Son Tespit Edilen Renk: {last_detected_color}")
    print(f"Güvenilirlik Oranı: {last_detected_conf:.3f}")
    print(f"RC Tetikleme Pini: GPIO {TRIGGER_PIN}")
    print(f"Sinyal Genişliği (µs): {pulse_width_us:.2f}")
    
    if master:
        print(f"MAVLink Bağlantısı: OK ({TELEMETRY_PORT})")
    else:
        print(f"MAVLink Bağlantısı: HATA")

    print(f"\nRC Tetikleme Durumu: {current_status}")

    time.sleep(0.01)

cap.release()
