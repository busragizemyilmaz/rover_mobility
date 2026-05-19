import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

class FakeSTM32(Node):
    def __init__(self):
        super().__init__('fake_stm32')
        self.pub = self.create_publisher(Int64MultiArray, '/encoder_raw', 10)
        
        # Saniyede 50 kere veri gönder (Gerçek STM32 hızımız)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Tekerleklerin başlangıç tikleri
        self.ticks = [0, 0, 0, 0]

    def timer_callback(self):
        # Sağ tekerlekler hızlı, sol tekerlekler yavaş dönsün (Araç sola dönüyor gibi)
        self.ticks[0] += 500  # Sağ Ön
        self.ticks[1] += 500  # Sağ Arka
        self.ticks[2] += 200  # Sol Ön
        self.ticks[3] += 200  # Sol Arka

        msg = Int64MultiArray()
        msg.data = self.ticks
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeSTM32()
    print("🚀 Sahte STM32 Veri Yayınlamaya Başladı! (CTRL+C ile durdurun)")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
