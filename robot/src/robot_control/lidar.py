import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy


# =========================================================
# 영역 설정
# =========================================================

# 정면 영역: -50° ~ +50°
FRONT_ANGLE = 50.0

# 측면 영역
SIDE_MIN = 50.0
SIDE_MAX = 135.0

# 현재 RViz 기준으로 추가 180도 회전 제거
FRONT_RAW_ANGLE = 180.0


class LidarReader(Node):

    def __init__(self):
        super().__init__("lidar_reader")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.front = 999.0
        self.left = 999.0
        self.right = 999.0

        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos
        )

        self.get_logger().info("LiDAR Reader Started")

    # =====================================================
    # 각도를 -180° ~ +180°로 정규화
    # =====================================================
    def normalize_angle(self, angle_deg):

        angle_rad = math.radians(angle_deg)

        normalized_rad = math.atan2(
            math.sin(angle_rad),
            math.cos(angle_rad)
        )

        return math.degrees(normalized_rad)

    # =====================================================
    # LaserScan 콜백
    # =====================================================
    def scan_callback(self, msg):

        front_list = []
        left_list = []
        right_list = []

        for i, distance in enumerate(msg.ranges):

            if math.isnan(distance) or math.isinf(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            raw_angle = math.degrees(
                msg.angle_min
                + i * msg.angle_increment
            )

            # 현재는 추가 회전 없음
            robot_angle = self.normalize_angle(
                FRONT_RAW_ANGLE - raw_angle
            )

            # 정면: -50° ~ +50°
            if -FRONT_ANGLE <= robot_angle <= FRONT_ANGLE:
                front_list.append(distance)

            # 왼쪽: -135° ~ -50°
            elif -SIDE_MAX <= robot_angle < -SIDE_MIN:
                left_list.append(distance)

            # 오른쪽: +50° ~ +135°
            elif SIDE_MIN < robot_angle <= SIDE_MAX:
                right_list.append(distance)

        self.front = min(front_list) if front_list else 999.0
        self.left = min(right_list) if right_list else 999.0
        self.right = min(left_list) if left_list else 999.0

    # =====================================================
    # 외부에서 거리값 가져오기
    # =====================================================
    def get_regions(self):
        return self.front, self.left, self.right

