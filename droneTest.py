# -*- coding: utf-8 -*-
# Gerekli kütüphaneleri içe aktar
from gpiozero import DigitalInputDevice
import time
import os

# GPIO pinini BCM numarasına göre ayarlayın.
TRIGGER_PIN = 17

# GPIO pinini en temel haliyle giriş olarak ayarla.
# Herhangi bir pull-up/down direnci ayarı yapmıyoruz.
pin = DigitalInputDevice(TRIGGER_PIN)

# Terminali temizle ve başlık yazdır
def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

clear_screen()
print("-------------------------------------")
print("  GPIO Sinyal Genişliği Testi (manuel)  ")
print("-------------------------------------")
print(f"Sinyal Ölçümü Yapılıyor: GPIO {TRIGGER_PIN}")
print("Kumanda tetikleyicisini değiştirin ve değerleri gözlemleyin...")
print("(Değer yoksa kumanda tetiklenmiyor veya bağlantı sorunu var)")

try:
    while True:
        # Pinin LOW seviyesine düşmesini bekle (ya da halihazırda LOW ise devam et)
        pin.wait_for_inactive(timeout=0.5)
        
        # Pinin HIGH seviyesine çıkmasını bekle ve başlangıç zamanını kaydet
        if pin.wait_for_active(timeout=0.5):
            start_time = time.time()
            # Pinin LOW seviyesine geri dönmesini bekle ve bitiş zamanını kaydet
            if pin.wait_for_inactive(timeout=0.5):
                end_time = time.time()
                
                # Sinyal genişliğini mikrosaniye cinsinden hesapla
                pulse_width_us = (end_time - start_time) * 1000000
                
                # Ekrana bas
                print(f"Sinyal Genişliği: {pulse_width_us:.2f} µs")
        
        # Döngüyü çok hızlı döndürmemek için kısa bir bekleme
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nProgram kullanıcı tarafından sonlandırıldı.")

finally:
    # GPIO pinini temizle
    pin.close()
    
