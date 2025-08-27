# renk_kalibrasyon.py
import cv2
import numpy as np

# Trackbar'lar için boş bir fonksiyon
def nothing(x):
    pass

# --- Kamera Ayarları ---
# Kamerayı başlat
cap = cv2.VideoCapture(0)
# Otomatik ayarları kapatmak, renklerin daha stabil kalmasını sağlar
cap.set(cv2.CAP_PROP_AUTO_WB, 0) # Otomatik Beyaz Dengesini Kapat
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # Otomatik Pozlamayı Manuel Moda Al
cap.set(cv2.CAP_PROP_EXPOSURE, -6) # Pozlama Değerini Manuel Sabitle (Ortamınıza göre -7, -5 gibi değiştirilebilir)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Ayar çubukları (trackbar) için bir pencere oluştur
cv2.namedWindow("Trackbars")

# Başlangıç değerlerini, ana kodunuzdaki son çalışan değerlere ayarlayabilirsiniz.
# Bu, kalibrasyona daha yakın bir noktadan başlamanıza yardımcı olur.
# H = Hue (Ton), S = Saturation (Doygunluk), V = Value (Parlaklık)
cv2.createTrackbar("L_H", "Trackbars", 36, 179, nothing) # Alt Ton
cv2.createTrackbar("L_S", "Trackbars", 60, 255, nothing) # Alt Doygunluk
cv2.createTrackbar("L_V", "Trackbars", 40, 255, nothing) # Alt Parlaklık
cv2.createTrackbar("U_H", "Trackbars", 85, 179, nothing) # Üst Ton
cv2.createTrackbar("U_S", "Trackbars", 255, 255, nothing) # Üst Doygunluk
cv2.createTrackbar("U_V", "Trackbars", 255, 255, nothing) # Üst Parlaklık

print("\n--- Renk Kalibrasyon Aracina Hos Geldiniz! ---")
print("1. Rengini bulmak istediginiz nesneyi kameraya gosterin.")
print("2. 'Trackbars' penceresindeki cubuklari oynatarak 'Mask' ekraninda nesnenizin tamamen beyaz, geri kalan her yerin siyah olmasini saglayin.")
print("3. İdeal degerleri buldugunuzda, bu degerleri ana kodunuzda kullanin.")
print("4. Cikmak icin klavyeden 'q' tusuna basin.")
print("--------------------------------------------------")

while True:
    # Kameradan bir kare oku
    ret, frame = cap.read()
    if not ret:
        print("Kamera okunamadı, program sonlandırılıyor.")
        break

    # Görüntüyü BGR'dan HSV renk uzayına çevir
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Trackbar'lardan anlık HSV değerlerini al
    l_h = cv2.getTrackbarPos("L_H", "Trackbars")
    l_s = cv2.getTrackbarPos("L_S", "Trackbars")
    l_v = cv2.getTrackbarPos("L_V", "Trackbars")
    u_h = cv2.getTrackbarPos("U_H", "Trackbars")
    u_s = cv2.getTrackbarPos("U_S", "Trackbars")
    u_v = cv2.getTrackbarPos("U_V", "Trackbars")

    # Yeni HSV aralıklarını oluştur
    lower_range = np.array([l_h, l_s, l_v])
    upper_range = np.array([u_h, u_s, u_v])

    # Belirlenen aralığa göre bir maske oluştur
    mask = cv2.inRange(hsv, lower_range, upper_range)
    
    # Gürültüyü temizlemek için morfolojik işlemler (isteğe bağlı ama önerilir)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # Orijinal görüntü ve maskeyi göster
    cv2.imshow("Original Goruntu", frame)
    cv2.imshow("Maske (Sonuc)", mask)

    # Çıkış için 'q' tuşunu bekle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("\nKalibrasyon tamamlandı. Bulunan değerler:")
        print(f"lower_range = ({l_h}, {l_s}, {l_v})")
        print(f"upper_range = ({u_h}, {u_s}, {u_v})")
        break

# Pencereleri kapat ve kamerayı serbest bırak
cap.release()
cv2.destroyAllWindows()
