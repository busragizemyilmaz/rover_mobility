import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray, Float32MultiArray
import math
import sys

class EncoderDashboard(Node):
    def __init__(self):
        super().__init__('encoder_dashboard')
        
        # C++ sürücüsünden gelen ham tikler
        self.sub = self.create_subscription(
            Int64MultiArray, 
            '/encoder_raw', 
            self.callback, 
            10)
            
        # Otonom paketi için işlenmiş metre verisini yayınladım
        self.pub = self.create_publisher(
            Float32MultiArray, 
            '/wheel_distances', 
            10)

        # --- FİZİKSEL AYARLAR ---
        # Elektronikçinin verdiği bilgi: 1 tur = 409600 tik
        self.TICKS_PER_REV = 409600.0  
        # Tekerlek Yarıçapı (Şimdilik 10 cm varsaydım, değişebilir)
        self.WHEEL_RADIUS_M = 0.10     
        # Çevre = 2 * pi * r
        self.WHEEL_CIRCUM = 2 * math.pi * self.WHEEL_RADIUS_M

        # --- HAFIZA DEĞİŞKENLERİ ---
        self.prev_ticks = [0, 0, 0, 0]
        self.total_ticks = [0, 0, 0, 0]
        self.first_run = True

        self.get_logger().info("Rover Odometri Baslatildi. Veri bekleniyor...")

    def callback(self, msg):
        if len(msg.data) < 4:
            return

        current_ticks = msg.data

        # İlk veriyi referans olarak al (Araç açıldığında sıfır noktası)
        if self.first_run:
            self.prev_ticks = list(current_ticks)
            self.first_run = False
            return

        distances = []
        
        for i in range(4):
            # 1. Adım: İki okuma arasındaki farkı (delta) bul
            delta = current_ticks[i] - self.prev_ticks[i]

            # 2. Adım: 32-Bit Overflow (Taşma) Koruması
            # Max 32-bit uint: 4294967295. Yarısı: ~2.1 milyar.
            # Eğer fark 2.1 milyardan büyükse, araç geri giderken sınır aşılmış (Underflow) demektir.
            if delta > 2147483647:
                delta -= 4294967296
            # Eğer fark -2.1 milyardan küçükse, araç ileri giderken sınır aşılmış (Overflow) demektir.
            elif delta < -2147483647:
                delta += 4294967296

            # Gerçek atılan adımı toplam hafızaya ekle
            self.total_ticks[i] += delta
            self.prev_ticks[i] = current_ticks[i]

            # 3. Adım: Matematiği yap (Tik -> Tur -> Metre)
            revolutions = self.total_ticks[i] / self.TICKS_PER_REV
            distance_meters = revolutions * self.WHEEL_CIRCUM
            distances.append(distance_meters)

        # 4. Adım: Otonom sistemi için veriyi ROS ağına gönder
        dist_msg = Float32MultiArray()
        dist_msg.data = distances
        self.pub.publish(dist_msg)

        # 5. Adım: Terminal (Dashboard)
        self.print_dashboard(current_ticks, distances)

    def print_dashboard(self, raw, dist):
        # Ekranı temizle ve verileri üst üste yaz (Araba kadranı gibi)
        sys.stdout.write("\033c") 
        output = (
            f"=================================================\n"
            f"          ROVER ODOMETRI DASHBOARD (CANLI)       \n"
            f"=================================================\n"
            f" Tekerlek        | Ham Tik (STM32) | Gidilen Mesafe\n"
            f"-----------------|-----------------|---------------\n"
            f" Sag On   (1)    | {raw[0]:<15} | {dist[0]:.4f} m\n"
            f" Sag Arka (2)    | {raw[1]:<15} | {dist[1]:.4f} m\n"
            f" Sol On   (3)    | {raw[2]:<15} | {dist[2]:.4f} m\n"
            f" Sol Arka (4)    | {raw[3]:<15} | {dist[3]:.4f} m\n"
            f"=================================================\n"
        )
        sys.stdout.write(output)
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
