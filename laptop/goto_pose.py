#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import json
import math
import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def load_locations():
    with open(os.path.join(BASE_DIR, "book_locations.json"), "r") as f:
        return json.load(f)


def yaw_to_quat(yaw_deg):
    yaw = math.radians(yaw_deg)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    qz, qw = yaw_to_quat(yaw)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


def main():
    rclpy.init()

    navigator = BasicNavigator()

    print("Nav2 활성화 대기 중...")
    navigator.waitUntilNav2Active()

    locations = load_locations()

    book_id = sys.argv[1] if len(sys.argv) > 1 else input("책 ID 입력: ")

    if book_id not in locations:
        print("존재하지 않는 ID")
        rclpy.shutdown()
        return

    x, y, yaw = locations[book_id]

    print(f"\n이동 시작 : {book_id}")
    print(f"x={x}, y={y}, yaw={yaw}")

    goal = make_pose(navigator, x, y, yaw)

    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        time.sleep(0.1)

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print("도착 완료!")

    elif result == TaskResult.CANCELED:
        print("이동 취소됨")

    elif result == TaskResult.FAILED:
        print("이동 실패")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
