import cv2
import traceback

tracker = None

def init_tracker(frame, bbox):
    """
    KCF takipçisini başlatır.
    :param frame: Takipçinin başlatılacağı video karesi.
    :param bbox: Takip edilecek nesnenin sınırlayıcı kutusu (x, y, w, h).
    """
    global tracker
    try:
        print("DEBUG (kcf_tracker): init_tracker çalışıyor...")
        if frame is None:
            raise ValueError("init_tracker: frame None geldi!")
        if not isinstance(bbox, (tuple, list)) or len(bbox) != 4:
            raise ValueError(f"init_tracker: bbox formatı hatalı: {bbox}")

        # OpenCV sürümüne göre KCF takipçi oluşturma
        # cv2.legacy modülü, bazı eski veya deneysel algoritmaları içerir.
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
            tracker = cv2.legacy.TrackerKCF_create()
            print("DEBUG (kcf_tracker): cv2.legacy.TrackerKCF_create() kullanıldı.")
        elif hasattr(cv2, 'TrackerKCF_create'): # Daha yeni OpenCV sürümlerinde doğrudan cv2 altında olabilir
            tracker = cv2.TrackerKCF_create()
            print("DEBUG (kcf_tracker): cv2.TrackerKCF_create() kullanıldı.")
        else:
            raise AttributeError("OpenCV KCF Tracker desteklemiyor. Lütfen 'opencv-contrib-python' kurulu olduğundan emin olun.")

        tracker.init(frame, bbox)
        print("DEBUG (kcf_tracker): Tracker başarıyla başlatıldı.")
    except Exception as e:
        print(f"HATA (kcf_tracker): init_tracker HATASI: {e}")
        traceback.print_exc() # Detaylı hata çıktısı
        tracker = None

def update_tracker(frame):
    """
    KCF takipçisini bir sonraki karede günceller.
    :param frame: Güncel video karesi.
    :return: (success, box) - Takip başarılı mı ve güncel sınırlayıcı kutu.
    """
    global tracker
    if tracker is None:
        # print("DEBUG (kcf_tracker): Tracker başlatılmamış, güncelleme atlandı.")
        return False, None
    try:
        if frame is None:
            raise ValueError("update_tracker: frame None geldi.")
        success, box = tracker.update(frame)
        # if success:
        #     print(f"DEBUG (kcf_tracker): Tracker güncellendi, başarı: {success}, kutu: {box}")
        # else:
        #     print("DEBUG (kcf_tracker): Tracker güncelleme başarısız.")
        return success, box
    except Exception as e:
        print(f"HATA (kcf_tracker): update_tracker HATASI: {e}")
        traceback.print_exc() # Detaylı hata çıktısı
        return False, None
