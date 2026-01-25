import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from . import MFRC522
import time

class RFIDReaderNode(Node):
    def __init__(self):
        super().__init__('rfid_reader_node')
        self.publisher_ = self.create_publisher(String, 'rfid_data', 10)
        
        # MFRC522 başlat
        try:
            self.reader = MFRC522.MFRC522(bus=0, dev=0, spd=500000)
            self.get_logger().info('✓ MFRC522 başarıyla başlatıldı (bus=10)')
        except Exception as e:
            self.get_logger().error(f'✗ MFRC522 başlatma hatası: {str(e)}')
            raise
            
        self.timer = self.create_timer(0.1, self.read_rfid)  # 100ms
        self.get_logger().info('RFID Okuyucu Node Başlatıldı (Pi 5 - Ubuntu 24.04)')
        self.last_uid = None
        self.last_read_time = 0

    def read_rfid(self):
        try:
            # Kart tarama
            (status, TagType) = self.reader.MFRC522_Request(self.reader.PICC_REQIDL)

            if status == self.reader.MI_OK:
                self.get_logger().info("✓ Kart Algılandı!")
                current_time = time.time()
                
                # DOĞRU METODU KULLAN: SelectTagSN (eskisi: Anticoll)
                (status, uid) = self.reader.MFRC522_SelectTagSN()
                
                if status == self.reader.MI_OK and len(uid) > 0:
                    uid_str = ",".join(map(str, uid))
                    uid_hex = ":".join(f'{x:02X}' for x in uid)
                    
                    # Aynı kartı 2 saniye içinde tekrar yayınlama
                    if uid_str != self.last_uid or (current_time - self.last_read_time) > 2.0:
                        msg = String()
                        msg.data = uid_str
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'✓ Okunan Kart UID: {uid_str} ({uid_hex})')
                        self.last_uid = uid_str
                        self.last_read_time = current_time
                else:
                    self.get_logger().warn("✗ UID okunamadı")
                        
        except Exception as e:
            self.get_logger().error(f'✗ Okuma hatası: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = RFIDReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
