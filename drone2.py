# -*- coding: utf-8 -*-
# Gerekli kütüphaneleri içe aktar
# (Import necessary libraries)
import cv2
import numpy as np
import serial
import time
from pymavlink import mavutil

# --- Seri Port ve Telemetri Ayarları ---
# (Serial Port and Telemetry Settings)
# RC alıcınız için seri portu ayarlayın. Genellikle Raspberry Pi'de bu /dev/ttyS0'dır.
# (Set the serial port for your RC receiver. On Raspberry Pi, it is usually /dev/ttyS0.)
RC_SERIAL_PORT = "/dev/ttyS0"
# Kumandanızın i-BUS baud hızı, genellikle 115200'dür.
# (The i-BUS baud rate of your remote, usually 115200.)
BAUD_RATE = 115200             

# MAVLink bağlantısı için seri portu ayarlayın.
# (Set the serial port for MAVLink connection.)
TELEMETRY_PORT = "/dev/ttyUSB0"
TELEMETRY_BAUD = 57600 

# Renk tespiti için kullanılacak RC kanalını ayarlayın.
# (Set the RC channel to be used for color detection.)
TRIGGER_CHANNEL = 5

# MAVLink bağlantısını oluştur. Hata durumunda kodun çalışmasını durdurmaz.
# (Establish MAVLink connection. Does not stop the code in case of an error.)
master = None
try:
    master = mavutil.mavlink_connection(TELEMETRY_PORT, baud=TELEMETRY_BAUD)
    print("MAVLink bağlantısı bekleniyor...")
    print("Waiting for MAVLink connection...")
    master.wait_heartbeat(timeout=5)
    print("MAVLink bağlantısı başarıyla kuruldu.")
    print("MAVLink connection established successfully.")
except Exception as e:
    print(f"MAVLink bağlantısı başarısız oldu: {e}")
    print(f"MAVLink connection failed: {e}")
    master = None

# --- Kamera ve Görüntü Ayarları ---
# (Camera and Image Settings)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_WB, 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Ortama göre bu değişkeni True/False olarak ayarlayın
# (Set this variable to True/False according to the environment)
USE_OUTDOOR = False 

# Sizin daha iyi sonuç aldığınız renk aralıklarını kullanıyoruz
# (We are using the color ranges that gave you better results)
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
    """
    Morfolojik işlemlerle (açma ve kapatma) gürültüyü temizler.
    (Cleans up noise with morphological operations (opening and closing).)
    """
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask

def detect_color(roi):
    """
    ROI'deki en baskın rengi (Siyah, Kırmızı, Yeşil veya Belirsiz) tespit eder.
    (Detects the most dominant color (Black, Red, Green, or Undetermined) in the ROI.)
    """
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Renk maskelerini oluştur
    # (Create color masks)
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    # Morfolojik temizleme işlemini her maskeye uygula
    # (Apply morphological cleaning to each mask)
    mask_red = clean(mask_red)
    mask_green = clean(mask_green)
    mask_black = clean(mask_black)
    
    # Minimum alan eşiği (ROI alanının %0.5'i)
    # (Minimum area threshold (0.5% of the ROI area))
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
    
    # Belirli bir alanın üzerinde bir kontur varsa rengi döndür
    # (Return the color if a contour is above a certain area)
    if max_area_value > min_area:
        return max_area_label, max_area_value / (roi.shape[0] * roi.shape[1])
    else:
        return "BELIRSIZ", 0.0

def send_mavlink_message(label, conf):
    """
    Tespit edilen renk ve güvenilirlik ile MAVLink STATUSTEXT mesajı gönderir.
    (Sends a MAVLink STATUSTEXT message with the detected color and confidence.)
    """
    if master is None:
        print("MAVLink bağlantısı yok, mesaj gönderilmiyor.")
        print("No MAVLink connection, message is not being sent.")
        return
    
    message = f"Tespit Edilen: {label} (Guven: {conf:.2f})"
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, message.encode())
    print(f"MAVLink mesajı gönderildi: {message}")
    print(f"MAVLink message sent: {message}")

# --- Ana Döngü ---
# (Main Loop)
ser = None
try:
    ser = serial.Serial(RC_SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
    print("RC Seri portu açıldı.")
    print("RC Serial port opened.")
except serial.SerialException as e:
    print(f"RC Seri portu hatası: {e}")
    print(f"RC Serial port error: {e}")
    ser = None
except FileNotFoundError:
    print(f"RC Seri portu bulunamadı: {RC_SERIAL_PORT}")
    print(f"RC Serial port not found: {RC_SERIAL_PORT}")
    ser = None

last_detected_color = "BELIRSIZ"
last_detected_conf = 0.0

while True:
    ok, frame = cap.read()
    if not ok:
        print("Kamera okunamadı.")
        print("Camera could not be read.")
        break

    # Kamera görüntüsünü sürekli işle
    # (Continuously process camera feed)
    small_frame = cv2.resize(frame, (320, 240))
    h, w = small_frame.shape[:2]
    roi_ratio = 0.7
    x0 = int((1-roi_ratio)/2 * w); x1 = int((1+(roi_ratio))/2 * w)
    y0 = int((1-roi_ratio)/2 * h); y1 = int((1+(roi_ratio))/2 * h)
    roi = small_frame[y0:y1, x0:x1]

    # Renk tespiti sürekli olarak yapılır.
    # (Color detection is performed continuously.)
    last_detected_color, last_detected_conf = detect_color(roi)
    print(f"Sürekli Tespit: Renk={last_detected_color}, Güven={last_detected_conf:.3f}")
    print(f"Continuous Detection: Color={last_detected_color}, Confidence={last_detected_conf:.3f}")

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

        except serial.SerialException as e:
            print(f"Seri okuma hatası: {e}")
            print(f"Serial read error: {e}")
            ser.close()
            ser = None
        except Exception as e:
            print(f"RC sinyal işleme hatası: {e}")
            print(f"RC signal processing error: {e}")

    # Sadece RC kanalı tetiklendiğinde telemetri mesajı gönderilir.
    # (Telemetry message is sent only when the RC channel is triggered.)
    if is_triggered:
        send_mavlink_message(last_detected_color, last_detected_conf)

