#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import math
import os
import threading

class Recorder(Node):
    def __init__(self):
        super().__init__('book_coord_recorder')

        # 현재 로봇 위치 저장 변수
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # /amcl_pose 구독
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info("AMCL pose subscriber started.")

    def pose_callback(self, msg):
        # 위치
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # orientation -> yaw 변환
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_yaw = math.degrees(yaw)


def load_from_json():
    if os.path.exists("book_locations.json"):
        with open("book_locations.json", "r") as f:
            data = json.load(f)
        print(f"기존 저장된 항목 {len(data)}개 불러옴: {list(data.keys())}\n")
        return data
    return {}


def save_to_json(data):
    with open("book_locations.json", "w") as f:
        json.dump(data, f, indent=4)


def input_thread(recorder):
    book_data = load_from_json()

    print("=== 좌표 저장 모드 ===")
    print("Nav2 Goal로 위치 이동 후 Enter 누르고 책 ID 입력")
    print("종료: Ctrl + C\n")

    while True:
        input("현재 위치 저장하려면 Enter >>> ")

        if recorder.current_x is None:
            print("아직 위치 정보가 없습니다.")
            continue

        print(f"\n현재 좌표")
        print(f"x: {recorder.current_x:.3f}")
        print(f"y: {recorder.current_y:.3f}")
        print(f"yaw: {recorder.current_yaw:.2f}")

        book_id = input("책 ID 입력 (예: B001): ")

        book_data[book_id] = [
            recorder.current_x,
            recorder.current_y,
            recorder.current_yaw
        ]

        save_to_json(book_data)

        print(f"{book_id} 저장 완료!\n")


def main():
    rclpy.init()

    recorder = Recorder()

    # ROS spin은 백그라운드
    thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    thread.start()

    # 입력은 메인 스레드
    input_thread(recorder)

    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
