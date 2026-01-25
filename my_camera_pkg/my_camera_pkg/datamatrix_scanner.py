import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from pylibdmtx.pylibdmtx import decode
import re
from gpiozero import Button 
import time
import csv
import os
import sys
from datetime import datetime

# ==========================================
#               AYARLAR
# ==========================================

HOME_DIR = os.path.expanduser("~") 
DATABASE_FILE = os.path.join(HOME_DIR, "ilac_veritabani.csv")

# RFID Yetkilendirme - Yetkili Kart UID'leri
AUTHORIZED_CARDS = [
    [203, 34, 35, 11],  # Ana yönetici kartı
    # Buraya başka yetkili kartlar ekleyebilirsiniz
    #[203, 34, 35, 11],
]

# Kritik Tarih Eşiği
CRITICAL_DATE_STR = "01/01/2026"

# Buton Pinleri (BCM)
BTN_SAVE_PIN = 4    
BTN_DEL_PIN = 27    
BTN_VIEW_PIN = 22   

MEDICINE_DB = {
    "08699527090025": "DEVIT-3 DAMLA",
    "08699546020556": "VERMAZOL TABLET",
    "08699844751493": "DOLVEN SURUP"
}

class PharmaSystemNode(Node):
    def __init__(self):
        super().__init__('pharma_system_node')
        
        # RFID Yetkilendirme Durumu
        self.is_authenticated = False
        self.auth_timeout = None
        self.AUTH_DURATION = 300  # 5 dakika (saniye)
        
        # --- GPIO KURULUMU ---
        try:
            self.btn_save = Button(BTN_SAVE_PIN, pull_up=False, bounce_time=0.2) 
            self.btn_del = Button(BTN_DEL_PIN, pull_up=False, bounce_time=0.2)
            self.btn_view = Button(BTN_VIEW_PIN, pull_up=False, bounce_time=0.2)
            self.get_logger().info("✅ Donanım Kontrolleri Tamam.")
        except Exception as e:
            self.get_logger().error(f"GPIO Hatası: {e}")
            self.btn_save = None

        self.current_mode = "VIEW" 
        self.last_code = "" 
        
        self.init_database()
        self.check_expired_products()

        # --- ROS2 SUBSCRIPTIONS ---
        self.br = CvBridge()
        
        # Kamera subscription
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # RFID subscription
        self.rfid_subscription = self.create_subscription(
            String, '/rfid_data', self.rfid_callback, 10)
        
        # Buton kontrolü ve timeout kontrolü için timer
        self.timer = self.create_timer(0.1, self.check_buttons_and_timeout)

        self.get_logger().info('✅ SİSTEM BAŞLATILDI!')
        print("\n" + "🔒"*30)
        print("🔐 SİSTEM KİLİTLİ - RFID KART OKUTUNUZ")
        print("🔒"*30 + "\n")

    def init_database(self):
        if not os.path.exists(DATABASE_FILE):
            try:
                with open(DATABASE_FILE, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["GTIN", "SeriNo", "SKT", "IlacAdi", "PartiNo"])
            except Exception as e:
                self.get_logger().error(f"Dosya hatası: {e}")

    def rfid_callback(self, msg):
        """RFID kartı okunduğunda çalışır"""
        try:
            # UID string'ini listeye çevir: "203,34,35,11" -> [203, 34, 35, 11]
            uid_list = [int(x) for x in msg.data.split(',')]
            
            # Yetkili kartlarla karşılaştır
            if uid_list in AUTHORIZED_CARDS:
                self.is_authenticated = True
                self.auth_timeout = time.time() + self.AUTH_DURATION
                
                print("\n" + "✅"*30)
                print(f"🔓 ERİŞİM ONAYLANDI!")
                print(f"   Kart UID: {':'.join(f'{x:02X}' for x in uid_list)}")
                print(f"   Oturum Süresi: {self.AUTH_DURATION//60} dakika")
                print("✅"*30 + "\n")
                print(f">>> MOD: {self.current_mode} (Görüntüle) <<<\n")
                
            else:
                print("\n" + "❌"*30)
                print(f"⛔ YETKİSİZ KART!")
                print(f"   Okunan UID: {':'.join(f'{x:02X}' for x in uid_list)}")
                print("❌"*30 + "\n")
                
        except Exception as e:
            self.get_logger().error(f"RFID işleme hatası: {e}")

    def check_buttons_and_timeout(self):
        """Buton kontrolü ve oturum timeout kontrolü"""
        
        # Oturum timeout kontrolü
        if self.is_authenticated and self.auth_timeout:
            remaining = int(self.auth_timeout - time.time())
            if remaining <= 0:
                self.is_authenticated = False
                self.auth_timeout = None
                print("\n" + "⏰"*30)
                print("🔒 OTURUM SÜRESİ DOLDU - SİSTEM KİLİTLENDİ")
                print("   Lütfen tekrar RFID kart okutunuz")
                print("⏰"*30 + "\n")
                return
            
            # Her 60 saniyede bir kalan süreyi göster
            if remaining % 60 == 0 and remaining > 0:
                mins = remaining // 60
                self.get_logger().info(f"⏱️  Kalan süre: {mins} dakika")
        
        # Buton kontrolü - sadece yetkili ise
        if self.btn_save is None or not self.is_authenticated:
            return
            
        if self.btn_save.is_pressed:
            self.change_mode("SAVE")
        elif self.btn_del.is_pressed:
            self.change_mode("DELETE")
        elif self.btn_view.is_pressed:
            self.change_mode("VIEW")

    def change_mode(self, new_mode):
        if not self.is_authenticated:
            print("⛔ Lütfen önce RFID kart okutunuz!")
            return
            
        if self.current_mode != new_mode:
            self.current_mode = new_mode
            self.last_code = "" 
            print("\n" + "#"*40)
            print(f"🔄 MOD DEĞİŞTİ -> {new_mode}")
            print("#"*40 + "\n")
            time.sleep(0.5)

    def check_expired_products(self):
        """Veritabanını tarar ve tarihi geçmiş ilaçları listeler"""
        print("\n" + "⏳"*15 + " SKT KONTROLÜ BAŞLIYOR " + "⏳"*15)
        
        expired_list = []
        try:
            target_date = datetime.strptime(CRITICAL_DATE_STR, "%d/%m/%Y")
            
            with open(DATABASE_FILE, mode='r') as file:
                reader = csv.reader(file)
                next(reader, None)
                
                for row in reader:
                    if len(row) >= 3:
                        try:
                            med_date_str = row[2].strip()
                            med_date = datetime.strptime(med_date_str, "%d/%m/%Y")
                            
                            if med_date < target_date:
                                expired_list.append(row)
                        except ValueError:
                            pass

            if len(expired_list) > 0:
                print("\n" + "🚨"*20)
                print(f"⚠️  DİKKAT! ({CRITICAL_DATE_STR}) TARİHİNDEN ÖNCE MİADI DOLAN İLAÇLAR TESPİT EDİLDİ!")
                print("⚠️  BU İLAÇLARIN SON KULLANIM TARİHLERİ GEÇMİŞTİR LÜTFEN RAFTAN İNDİRİNİZ.")
                print("🚨"*20 + "\n")
                
                print(f"{'İLAÇ ADI':<30} | {'SKT':<12} | {'SERİ NO'}")
                print("-" * 60)
                for item in expired_list:
                    print(f"\033[91m{item[3]:<30} | {item[2]:<12} | {item[1]}\033[0m")
                print("-" * 60 + "\n")
            else:
                print("✅ Veritabanında kritik tarihli ilaç bulunmadı. Her şey yolunda.\n")

        except FileNotFoundError:
            print("ℹ️ Henüz veritabanı yok, kontrol geçildi.")

    def parse_gs1_robust(self, raw_data):
        parsed = {
            "gtin": "Bilinmiyor", "serial": "Bilinmiyor", 
            "expiry": "Bilinmiyor", "name": "Bilinmeyen İlaç"
        }
        temp_data = raw_data.replace('\x1d', '').strip()

        # 1. GTIN
        if "01" in temp_data:
            start = temp_data.find("01")
            if start == 0 or start == 1: 
                gtin_candidate = temp_data[start+2 : start+16]
                if len(gtin_candidate) == 14 and gtin_candidate.isdigit():
                    parsed["gtin"] = gtin_candidate
                    if parsed["gtin"] in MEDICINE_DB:
                        parsed["name"] = MEDICINE_DB[parsed["gtin"]]
                    temp_data = temp_data.replace("01" + gtin_candidate, "", 1)

        # 2. Tarih
        matches = list(re.finditer("17(\d{6})", temp_data))
        for match in matches:
            d_raw = match.group(1)
            mm = int(d_raw[2:4])
            if 1 <= mm <= 12:
                parsed["expiry"] = f"{d_raw[4:6]}/{d_raw[2:4]}/20{d_raw[0:2]}"
                temp_data = temp_data.replace("17" + d_raw, "", 1)
                break

        # 3. Seri No
        if "21" in temp_data:
            start_serial = temp_data.find("21")
            rest = temp_data[start_serial+2:]
            if "10" in rest and len(rest) > 10: 
                parts = rest.split("10")
                parsed["serial"] = parts[0]
            else:
                parsed["serial"] = rest
        parsed["serial"] = parsed["serial"].strip()
        return parsed

    def execute_db_action(self, data):
        if not self.is_authenticated:
            print("⛔ YETKİSİZ İŞLEM! Lütfen RFID kart okutunuz.")
            return
            
        gtin = data['gtin'].strip()
        serial = data['serial'].strip()
        
        if gtin == "Bilinmiyor" or serial == "Bilinmiyor":
            print("⚠️ Eksik Veri.")
            return

        all_rows = []
        found_index = -1
        
        try:
            with open(DATABASE_FILE, mode='r') as file:
                reader = csv.reader(file)
                all_rows = list(reader)
        except FileNotFoundError:
            return

        for i in range(1, len(all_rows)):
            row = all_rows[i]
            if len(row) >= 2:
                if row[0].strip() == gtin and row[1].strip() == serial:
                    found_index = i
                    break

        if self.current_mode == "SAVE":
            if found_index != -1:
                print(f"⚠️ ZATEN KAYITLI! ({all_rows[found_index][3]})")
            else:
                with open(DATABASE_FILE, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    name_to_save = MEDICINE_DB.get(gtin, data['name'])
                    writer.writerow([gtin, serial, data['expiry'], name_to_save, ""])
                print(f"✅ KAYDEDİLDİ: {name_to_save}")

        elif self.current_mode == "DELETE":
            if found_index != -1:
                deleted_item = all_rows.pop(found_index)
                with open(DATABASE_FILE, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerows(all_rows)
                print(f"🗑️ SİLİNDİ: {deleted_item[3]}")
            else:
                print("❌ SİLİNECEK KAYIT BULUNAMADI.")

        elif self.current_mode == "VIEW":
            if found_index != -1:
                row = all_rows[found_index]
                print(f"\n🔎 MEVCUT | {row[3]} | SKT: {row[2]}")
            else:
                print(f"ℹ️ YOK")

    def image_callback(self, msg):
        # Görüntüleme modunda yetki kontrolü yok, diğer modlarda var
        if self.current_mode != "VIEW" and not self.is_authenticated:
            return
            
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            decoded_objects = decode(gray, timeout=50)

            for obj in decoded_objects:
                raw_data = obj.data.decode("utf-8")
                if raw_data != self.last_code:
                    self.last_code = raw_data
                    parsed = self.parse_gs1_robust(raw_data)
                    print("\n" + "="*40)
                    print(f"📷 OKUNAN: {parsed['name']}")
                    self.execute_db_action(parsed)
                    print("="*40 + "\n")
        except Exception as e:
            self.get_logger().error(f'Hata: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PharmaSystemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n👋 Sistem kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
