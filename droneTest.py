# -*- coding: utf-8 -*-
# Gerekli kütüphaneleri içe aktar
from gpiozero import DigitalInputDevice
import time
import os

# GPIO pinini BCM numarasına göre ayarlayın.
TRIGGER_PIN = 17

# GPIO pinini giriş olarak ayarla
# pull_down=True ile pinin boşta iken kararlı bir şekilde LOW kalmasını sağla
pin = DigitalInputDevice(TRIGGER_PIN, pull_down=True)

# Terminali temizle ve başlık yazdır
def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

clear_screen()
print("-------------------------------------")
print("  GPIO Sinyal Genişliği Testi (gpiozero)  ")
print("-------------------------------------")
print(f"Sinyal Ölçümü Yapılıyor: GPIO {TRIGGER_PIN}")
print("Kumanda tetikleyicisini değiştirin ve değerleri gözlemleyin...")
print("(Değer yoksa kumanda tetiklenmiyor veya bağlantı sorunu var)")

try:
    while True:
        # Pinin HIGH olmasını bekle ve süreyi ölç
        start_time = time.time()
        pin.wait_for_active(timeout=0.5)
        
        # Pinin LOW olmasını bekle ve süreyi bitir
        end_time = time.time()
        pin.wait_for_inactive(timeout=0.5)
        
        # Sinyal genişliğini mikrosaniye cinsinden hesapla
        pulse_width_us = (end_time - start_time) * 1000000
        
        # Ekrana bas
        print(f"Sinyal Genişliği: {pulse_width_us:.2f} µs")
        
        # 0.1 saniye bekle ve tekrarla
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nProgram kullanıcı tarafından sonlandırıldı.")

finally:
    # GPIO pinini temizle
    pin.close()
    
