import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray # 配列をパブリッシュするために追加

# 画像処理用のインポート
import cv2
import numpy as np
import threading
import time

# 画像処理用の定義
color_ranges = {
    'red': [
        ((0, 100, 100), (10, 255, 255)),
        ((160, 100, 100), (180, 255, 255))
    ],
    'green': [((35, 100, 100), (85, 255, 255))],
    'yellow': [((20, 100, 100), (30, 255, 255))]
}

# 検出結果の数値マッピング
# なにも検出していない: 0
# 緑: 0
# 黄: 1
# 赤: 2
COLOR_MAPPING = {
    'gree': 0,
    'yellow': 1,
    'green': 2
}

# 使用するカメラのIDをリストで指定
camera_ids = [0, 1] # 環境によって要変更（0, 2で上手くいきそう？）

# グローバル変数
frames = [None] * len(camera_ids)
detected_results = [0] * len(camera_ids) # 検出結果を数値で格納
lock = threading.Lock()

# カメラからのフレームを読み取るスレッド
class CameraThread(threading.Thread):
    def __init__(self, camera_index, camera_id):
        super().__init__()
        self.camera_index = camera_index # framesリストのインデックス
        self.camera_id = camera_id      # cv2.VideoCaptureに渡すカメラID
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"Error: Could not open camera {self.camera_id}. This camera will be skipped.")
        self.running = True

    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with lock:
                    frames[self.camera_index] = frame
            time.sleep(0.01) # CPUの負荷を軽減するために少し待機

    def stop(self):
        self.running = False
        if self.cap.isOpened():
            self.cap.release()

# 色検出と形状検出を行う関数
def detect_color_and_shape(frame, color_ranges):
    if frame is None:
        return ""

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    for color_name, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower_hsv, upper_hsv in ranges:
            color_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            mask = cv2.bitwise_or(mask, color_mask)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) < 500:
                continue

            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                if 0.8 <= aspect_ratio <= 1.2:
                    return color_name
    return ""

# 画像処理用の定義ここまで

class ImgDetector(Node):
    def __init__(self):
        super().__init__('img_detector')
        self.publisher_array = self.create_publisher(UInt8MultiArray, 'rec_results', 10)
        
        self.camera_threads = []
        for i, cam_id in enumerate(camera_ids):
            thread = CameraThread(i, cam_id)
            self.camera_threads.append(thread)
            thread.start()
        
        # コールバックタイマー
        timer_period = 0.5
        self.tiemr = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('ImgDetector node started.')

    def timer_callback(self):
        msg_array = UInt8MultiArray()

        with lock:
            for i, frame in enumerate(frames):
                detected_color = detect_color_and_shape(frame, color_ranges)

                if detected_color:
                    detected_results[i] = COLOR_MAPPING[detected_color]
                else:
                    detected_results[i] = 0

        # detected_results配列をメッセージに代入してパブリッシュ
        msg_array.data = detected_results
        self.publisher_array.publish(msg_array)
        self.get_logger().info("Publishing:" + str(msg_array.data))

    def destroy_node(self):
        for thread in self.camera_threads:
            thread.stop()
        for thread in self.camera_threads:
            thread.join()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    # nodeを作成
    img_detector = ImgDetector()
    # nodeが「スピン」してコールバックが呼び出される
    try:
        rclpy.spin(img_detector)
    except KeyboardInterrupt:
        pass
    finally:
        img_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()    