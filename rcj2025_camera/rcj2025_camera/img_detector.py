import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

class ImgDetector(Node):
    def __init__(self):
        super().__init__('img_detector')
        self.publisher_ = self.create_publisher(UInt8, 'rec_result', 10)
        # コールバックタイマー
        timer_period = 0.5
        self.tiemr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = UInt8()

        # 画像認識ゴニョゴニョ

        # 結果を以下の式に代入
        msg.data =  2
        self.get_logger().info("Publishing:" + str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    # nodeを作成
    img_detector = ImgDetector()
    # nodeが「スピン」してコールバックが呼び出される
    rclpy.spin(img_detector)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    