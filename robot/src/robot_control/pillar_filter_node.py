#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class PillarFilterNode(Node):

    def __init__(self):
        super().__init__("pillar_filter_node")

        # 입력: 방향 보정이 완료된 라이다 데이터
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan_fixed",
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 출력: 기둥 제거가 완료된 라이다 데이터
        self.publisher = self.create_publisher(
            LaserScan,
            "/scan_filtered",
            qos_profile_sensor_data
        )

        # 기둥 중심 좌표 (laser_frame 기준)
        # X: 앞쪽(+), 뒤쪽(-)
        # Y: 왼쪽(+), 오른쪽(-)
        self.pillars = [
            (-0.138,  0.132),  # 왼쪽 기둥
            (-0.138, -0.132),  # 오른쪽 기둥
        ]

        # 기둥 제거 반경 (m). 기둥이 남으면 0.05로 올려서 테스트
        self.pillar_radius = 0.04

        # 진단용: 기둥 중심에서 이 거리 이내의 점 개수를 로그로 확인
        self.diag_radius = 0.10

        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            "Pillar filter started: /scan_fixed -> /scan_filtered "
            f"(radius={self.pillar_radius} m)"
        )

    def nearest_pillar_distance(self, x, y):
        return min(
            math.hypot(x - px, y - py)
            for px, py in self.pillars
        )

    def is_pillar(self, x, y):

        for pillar_x, pillar_y in self.pillars:

            distance = math.hypot(
                x - pillar_x,
                y - pillar_y
            )

            if distance <= self.pillar_radius:
                return True

        return False

    def scan_callback(self, msg):

        output = LaserScan()

        # 기존 스캔 정보 유지
        output.header = msg.header
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max

        filtered_ranges = list(msg.ranges)
        filtered_intensities = list(msg.intensities)
        has_intensities = len(filtered_intensities) == len(filtered_ranges)

        removed_count = 0
        near_count = 0

        for i, distance in enumerate(msg.ranges):

            # 유효하지 않은 거리 데이터는 그대로 유지
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            # 해당 측정값의 각도
            angle = msg.angle_min + i * msg.angle_increment

            # 극좌표 -> 직교좌표
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            # 진단: 기둥 중심 근처(10cm 이내) 점 개수
            if self.nearest_pillar_distance(x, y) <= self.diag_radius:
                near_count += 1

            # 기둥 영역에 해당하면 측정값 제외
            if self.is_pillar(x, y):
                filtered_ranges[i] = float("inf")
                if has_intensities:
                    filtered_intensities[i] = 0.0
                removed_count += 1

        output.ranges = filtered_ranges
        output.intensities = filtered_intensities

        self.publisher.publish(output)

        # 로그 출력은 5초마다
        now = self.get_clock().now()

        if (now - self.last_log_time).nanoseconds >= 5_000_000_000:

            self.get_logger().info(
                f"removed={removed_count}, "
                f"within{int(self.diag_radius * 100)}cm={near_count}"
            )

            self.last_log_time = now


def main(args=None):

    rclpy.init(args=args)

    node = PillarFilterNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
