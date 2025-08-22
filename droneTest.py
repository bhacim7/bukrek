# -*- coding: utf-8 -*-
# Gerekli kütüphaneleri içe aktar
import RPi.GPIO as GPIO
import time

# GPIO pinini BCM numarasına göre ayarlayın.
TRIGGER_PIN = 17

# GPIO modunu BCM olarak ayarla
GPIO.setmode(GPIO.BCM)

# Pini giriş olarak ayarla ve pull-down direncini aktifleştir.
# Bu, pinin boşta kalmasını engeller.
GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Terminali temizle ve başlık yazdır
def clear_screen():
    import os
    os.system('cls' if os.name == 'nt' else 'clear')

clear_screen()
print("-------------------------------------")
print("  GPIO Sinyal Genişliği Testi  ")
print("-------------------------------------")
print(f"Sinyal Ölçümü Yapılıyor: GPIO {TRIGGER_PIN}")
print("Kumanda tetikleyicisini değiştirin ve değerleri gözlemleyin...")

try:
    while True:
        # Pinin LOW seviyesine düşmesini bekle
        GPIO.wait_for_edge(TRIGGER_PIN, GPIO.FALLING, timeout=0.5)
        
        # Sinyal HIGH olduğunda başlangıç zamanını kaydet
        start_time = time.time()
        
        # Pinin HIGH seviyesine çıkmasını bekle
        GPIO.wait_for_edge(TRIGGER_PIN, GPIO.RISING, timeout=0.5)
        
        # Sinyal LOW olduğunda bitiş zamanını kaydet
        end_time = time.time()
        
        # Sinyal genişliğini mikrosaniye cinsinden hesapla
        pulse_width_ms = (end_time - start_time) * 1000000
        
        # Ekrana bas
        print(f"Sinyal Genişliği: {pulse_width_ms:.2f} µs")
        
        # 0.1 saniye bekle ve tekrarla
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nProgram kullanıcı tarafından sonlandırıldı.")

finally:
    # GPIO pinlerini temizle
    GPIO.cleanup()

