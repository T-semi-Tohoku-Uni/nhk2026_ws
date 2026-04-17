#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os

BASE_DIR = os.path.abspath(os.path.dirname(__file__))
KFS_MODEL_PATH = os.path.join(BASE_DIR, "../../../../src/nhk2026_yolo/Models/kfs.pt")
model = YOLO(KFS_MODEL_PATH)
# cap = cv2.VideoCapture(0)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # Subscriberの作成
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # RealSenseのカラー画像トピック '/camera/color/image_raw'
            self.listener_callback,
            10)
        # CvBridgeの初期化
        self.bridge = CvBridge()

    def listener_callback(self, data):
        # ROS 2 メッセージを OpenCV 形式に変換
        # "bgr8" を指定することで、OpenCV標準のBGR形式になります
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding = 'bgr8')

        predict_kfs(current_frame)

        # ここにcv2の処理を書く（例：表示）
        # cv2.imshow("RealSense Color Frame", current_frame)
        # cv2.waitKey(1)

def predict_kfs(frame):
    results = model.predict(frame, verbose = False)
    class_colors = [(255, 0, 0), (235, 206, 135), (0, 0, 255), (203, 192, 255)] # bgr の順番　kfs の場合
    annotated = frame.copy()
    for box in results[0].boxes:
        cls_id = int(box.cls[0])          # クラスID
        label = model.names[cls_id]       # クラス名
        conf = round(float(box.conf[0]), 2)         # 信頼度（0〜1）
        color = class_colors[cls_id]      # クラスごとの色

        x1, y1, x2, y2 = box.xyxy[0].int().tolist()

        # バウンディングボックス
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

        # 表示するテキスト（例： "person 87%"）
        text = f"{label} {conf}"

        # ラベル背景（テキスト幅に合わせて可変）
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
        cv2.putText(
            annotated,
            text,
            (x1 + 3, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

    cv2.imshow('Web Camera View', annotated)

def main(args = None):

    rclpy.init(args = args)
    image_subscriber = ImageSubscriber()
    rclpy.shutdown()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()

        # カメラを開放し、バッファを破棄する
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

