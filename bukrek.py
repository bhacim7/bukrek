# -*- coding: utf-8 -*-
# Gerekli kütüphaneleri içe aktar
# cv2 ve numpy'nin kurulu olduğu varsayılıyor.
# Sadece pymavlink ve pyserial kütüphanelerini import ediyoruz.
import cv2
import numpy as np
import serial
import time
from pymavlink import mavutil

# --- Seri Port ve Telemetri Ayarları ---
# RC alıcınız için seri portu ayarlayın. Genellikle Raspberry Pi'de bu /dev/ttyS0'dır.
# Lütfen donanım kurulumunuzu kontrol edin.
RC_SERIAL_PORT = "/dev/ttyS0"
# SBUS için standart baud hızı 100000'dir.
BAUD_RATE = 100000             

# MAVLink bağlantısı için seri portu ayarlayın.
# Bu, AGX Orin'e bağlı telemetriye bağlanmak içindir.
# Radyo modelinize göre /dev/ttyUSB0 gibi bir değer olabilir.
TELEMETRY_PORT = "/dev/ttyUSB0"
# Standart telemetri baud hızı 57600'dür.
TELEMETRY_BAUD = 57600 

# Renk tespiti için kullanılacak RC kanalını ayarlayın.
# Kumandanızdaki bir anahtar (switch) kanalına karşılık gelmelidir.
# Bu değeri kumandanızın kanal monitöründen kontrol edebilirsiniz.
TRIGGER_CHANNEL = 5 # Örnek: Kanal 5'teki anahtar

# MAVLink bağlantısını oluştur. Başarısız olursa, kod telemetri göndermeden devam eder.
master = None
try:
    master = mavutil.mavlink_connection(TELEMETRY_PORT, baud=TELEMETRY_BAUD)
    print("MAVLink bağlantısı bekleniyor...")
    master.wait_heartbeat(timeout=5)
    print("MAVLink bağlantısı başarıyla kuruldu.")
except Exception as e:
    print(f"MAVLink bağlantısı başarısız oldu: {e}")
    master = None

# --- Kamera ve Renk Tespiti Ayarları ---
# Kamera nesnesini başlat
cap = cv2.VideoCapture(0)
# Otomatik beyaz ayarı ve pozlamayı kapat, manuel ayar için daha stabil bir görüntü sağlar.
cap.set(cv2.CAP_PROP_AUTO_WB, 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Ortama göre bu değişkeni True/False olarak ayarlayın
# Açık hava için renk aralıkları genellikle daha geniştir.
USE_OUTDOOR = True 

# Renk aralıklarını tanımlayın (HSV formatında)
if USE_OUTDOOR:
    # Kırmızı renk için iki aralık kullanılır (HSV döngüsü nedeniyle)
    lower_red1, upper_red1 = (0, 150, 100), (10, 255, 255)
    lower_red2, upper_red2 = (170, 150, 100), (179, 255, 255)
    # Yeşil ve Siyah renk aralıkları
    lower_green, upper_green = (40, 150, 100), (85, 255, 255)
    lower_black, upper_black = (0, 0, 0), (179, 100, 50)
else:
    # İç mekan/kapalı alan renk aralıkları
    lower_red1, upper_red1 = (0, 140, 90), (10, 255, 255)
    lower_red2, upper_red2 = (170, 140, 90), (179, 255, 255)
    lower_green, upper_green = (36, 120, 90), (85, 255, 255)
    lower_black, upper_black = (0, 0, 0), (179, 100, 60)

# Görüntü işleme için morfolojik çekirdek
kernel = np.ones((5,5), np.uint8)

def clean(mask):
    """Morfolojik işlemlerle (açma ve kapatma) gürültüyü temizler."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask

def detect_color(roi):
    """ROI'deki en baskın rengi (Siyah, Kırmızı, Yeşil veya Belirsiz) tespit eder."""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Renk maskelerini oluştur
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    # Siyah renk için piksel sayımına dayalı bir yaklaşım daha güvenilirdir.
    black_pixel_count = cv2.countNonZero(mask_black)
    black_threshold = int(roi.shape[0] * roi.shape[1] * 0.5)

    if black_pixel_count > black_threshold:
        return "SIYAH", black_pixel_count / (roi.shape[0] * roi.shape[1])
    
    # Diğer renkler için morfolojik temizlik ve en büyük kontur tespiti
    mask_red = clean(mask_red)
    mask_green = clean(mask_green)
    
    min_area = int(roi.shape[0] * roi.shape[1] * 0.005)

    def max_contour_area(mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return 0
        return max((cv2.contourArea(c) for c in cnts))

    red_area = max_contour_area(mask_red)
    green_area = max_contour_area(mask_green)
    
    areas = {"KIRMIZI": red_area, "YESIL": green_area}
    max_area_label = max(areas, key=areas.get)
    max_area_value = areas[max_area_label]
    
    # Belirli bir alanın üzerinde bir kontur varsa rengi döndür
    if max_area_value > min_area:
        return max_area_label, max_area_value / (roi.shape[0] * roi.shape[1])
    else:
        return "BELIRSIZ", 0.0

def send_mavlink_message(label, conf):
    """Tespit edilen renk ve güvenilirlik ile MAVLink STATUSTEXT mesajı gönderir."""
    if master is None:
        print("MAVLink bağlantısı yok, mesaj gönderilmiyor.")
        return
    
    message = f"Tespit Edilen: {label} (Guven: {conf:.2f})"
    # MAV_SEVERITY_INFO, mesajın yer istasyonunda bilgilendirici olarak görünmesini sağlar.
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, message.encode())
    print(f"MAVLink mesajı gönderildi: {message}")

# --- Ana Döngü ---
# RC seri okuyucusunu başlat
ser = None
try:
    ser = serial.Serial(RC_SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
    print("RC Seri portu açıldı.")
except serial.SerialException as e:
    print(f"RC Seri portu hatası: {e}")
    ser = None
except FileNotFoundError:
    print(f"RC Seri portu bulunamadı: {RC_SERIAL_PORT}")
    ser = None

while True:
    ok, frame = cap.read()
    if not ok:
        print("Kamera okunamadı.")
        break

    # RC girişini seri porttan oku
    is_triggered = False
    if ser:
        try:
            # SBUS protokolünü basitleştirilmiş bir şekilde okuma
            sbus_frame = ser.read(25) # SBUS çerçevesi 25 bayttır
            if len(sbus_frame) == 25:
                # Sadece tetikleyici kanalın değerini kontrol et.
                # Bu kısım daha karmaşık bir SBUS çözücü ile değiştirilebilir.
                channel_value = (sbus_frame[TRIGGER_CHANNEL * 2 + 1] | ((sbus_frame[TRIGGER_CHANNEL * 2] & 0x07) << 8))
                
                # Kumandanızdaki switch'in 1500 üzerindeki bir değeri tetikleme olarak kabul ediyoruz.
                if channel_value > 1500:
                    is_triggered = True
        except serial.SerialException as e:
            print(f"Seri okuma hatası: {e}")
            ser.close()
            ser = None
        except Exception as e:
            print(f"RC sinyal işleme hatası: {e}")


    # Sadece RC kanalı tetiklendiğinde renk tespiti yap
    if is_triggered:
        # Performans için kareyi yeniden boyutlandır
        small_frame = cv2.resize(frame, (320, 240))
        h, w = small_frame.shape[:2]

        # İlgi alanını (ROI) belirle - karenin ortası
        roi_ratio = 0.7
        x0 = int((1-roi_ratio)/2 * w); x1 = int((1+(roi_ratio))/2 * w)
        y0 = int((1-roi_ratio)/2 * h); y1 = int((1+(roi_ratio))/2 * h)
        roi = small_frame[y0:y1, x0:x1]

        label, conf = detect_color(roi)
        send_mavlink_message(label, conf)

        # Görselleştirme için orijinal kareye bilgi yazdır ve kareyi göster.
        out = frame.copy()
        h_orig, w_orig = frame.shape[:2]
        x0_orig = int((1-roi_ratio)/2 * w_orig); x1_orig = int((1+(roi_ratio))/2 * w_orig)
        y0_orig = int((1-roi_ratio)/2 * h_orig); y1_orig = int((1+(roi_ratio))/2 * h_orig)

        cv2.rectangle(out, (x0_orig,y0_orig), (x1_orig,y1_orig), (255,255,255), 2)
        cv2.putText(out, f"Renk: {label} (Guven: {conf:.3f})", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("frame", out)
    else:
        # Tetiklenmediyse sadece canlı kamera görüntüsünü göster
        cv2.putText(frame, "RC Tetikleyici Boşta", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()
