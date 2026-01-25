import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np # Matris işlemleri için gerekli

class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 5)
        
        # --- AYARLAR ---
        TARGET_WIDTH = 640  # DataMatrix için devasa çözünürlüğe gerek yok, 640x480 daha hızlıdır
        TARGET_HEIGHT = 480
        TARGET_FPS = 30.0
        
        # Kamerayı başlat
        self.cap = cv2.VideoCapture(0)
        
        # Ayarları uygula
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Gecikmeyi önle
        
        # CLAHE (Akıllı Kontrast) Nesnesi Oluştur
        # clipLimit: Kontrast sınırı (Yüksek olursa gürültü artar, 2.0 ideal)
        # tileGridSize: Görüntüyü kaç parçaya bölüp analiz edeceği (8x8 standarttır)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

        # Keskinleştirme Kernel Matrisi (Matrix Convolution)
        # Merkezdeki pikseli güçlendirip (5), etrafındakileri (-1) baskılar.
        self.sharpen_kernel = np.array([[0, -1, 0],
                                        [-1, 5,-1],
                                        [0, -1, 0]])
        
        self.br = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error('Kamera açılamadı!')
        else:
            self.get_logger().info('Kamera: Gri Tonlama + Yüksek Kontrast Modunda Başlatıldı 👁️')

        self.timer = self.create_timer(1.0/TARGET_FPS, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # ADIM 1: Griye Çevir
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # ADIM 2: Akıllı Kontrast (CLAHE) Uygula
            # Bu işlem, ilaç kutusu üzerindeki parlama ve gölgeleri dengeler.
            high_contrast = self.clahe.apply(gray)
            
            # ADIM 3: Keskinleştir (Sharpen)
            # Bulanık karekodları netleştirir.
            sharp = cv2.filter2D(high_contrast, -1, self.sharpen_kernel)
            
            # ADIM 4: Tekrar BGR Formatına Çevir (Önemli!)
            # Scanner node'un "bgr8" bekliyor olabilir. Hata vermemesi için
            # gri görüntüyü 3 kanallı formata geri çeviriyoruz (görüntü hala gri görünür).
            final_image = cv2.cvtColor(sharp, cv2.COLOR_GRAY2BGR)

            # Yayınla
            self.publisher_.publish(self.br.cv2_to_imgmsg(final_image, "bgr8"))
        else:
            self.get_logger().warn('Görüntü alınamadı!')

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
