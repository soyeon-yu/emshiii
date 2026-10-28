#!/usr/bin/env python3

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


LOCATIONS_FILE = Path.home() / "turtlebot_test" / "book_locations.json"


class AppNavBridge(Node):
    def __init__(self):
        super().__init__("app_nav_bridge")

        self.create_subscription(
            String,
            "/app/book_request",
            self.book_request_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/app/nav_status",
            10
        )

        self.navigator = BasicNavigator()
        self.is_navigating = False
        self.current_book_id = None

        self.create_timer(0.2, self.check_navigation_result)

        self.publish_status("idle")
        self.get_logger().info("앱-Nav2 브리지 준비 완료")
        self.get_logger().info("/app/book_request 대기 중")

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"상태 전송: {text}")

    def load_location(self, book_id):
        try:
            with open(LOCATIONS_FILE, "r", encoding="utf-8") as file:
                locations = json.load(file)
        except FileNotFoundError:
            self.get_logger().error(f"좌표 파일이 없습니다: {LOCATIONS_FILE}")
            return None
        except json.JSONDecodeError:
            self.get_logger().error("book_locations.json 형식이 잘못되었습니다.")
            return None

        return locations.get(book_id)

    def book_request_callback(self, msg):
        book_id = msg.data.strip()

        if not book_id:
            self.publish_status("invalid_request")
            return

        if self.is_navigating:
            self.publish_status("busy")
            self.get_logger().warn("이미 이동 중입니다.")
            return

        location = self.load_location(book_id)

        if location is None:
            self.publish_status(f"not_found:{book_id}")
            self.get_logger().warn(f"좌표가 없는 ID입니다: {book_id}")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        x = float(location[0])
        y = float(location[1])
        yaw_degrees = float(location[2])
        yaw_radians = math.radians(yaw_degrees)

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw_radians / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw_radians / 2.0)
        self.current_book_id = book_id

        self.get_logger().info(
            f"{book_id} 이동 시작: "
            f"x={goal_pose.pose.position.x:.2f}, "
            f"y={goal_pose.pose.position.y:.2f}"
        )

        self.navigator.goToPose(goal_pose)
        self.is_navigating = True
        self.publish_status(f"navigating:{book_id}")

    def check_navigation_result(self):
        if not self.is_navigating:
            return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        book_id = self.current_book_id

        if result == TaskResult.SUCCEEDED:
            self.publish_status(f"succeeded:{book_id}")
        elif result == TaskResult.CANCELED:
            self.publish_status(f"canceled:{book_id}")
        else:
            self.publish_status(f"failed:{book_id}")

        self.is_navigating = False
        self.current_book_id = None


def main():
    rclpy.init()
    node = AppNavBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
