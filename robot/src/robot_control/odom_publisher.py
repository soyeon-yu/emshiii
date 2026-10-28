# odom_publisher.py

import math
import time
import threading

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from mpu6050 import mpu6050

import motor


# =========================================================
# 로봇 및 엔코더 스펙
# =========================================================

# 바퀴 지름
WHEEL_DIAMETER = 0.130

# 바퀴 둘레
WHEEL_CIRCUMFERENCE = (
    math.pi * WHEEL_DIAMETER
)

# 좌우 바퀴 중심 사이 거리
TRACK_WIDTH = 0.378

# 실제 바퀴 1회전 실측값
PULSES_PER_REV = 3793.0

# 하드웨어팀 검증 거리 보정값
LEFT_DISTANCE_SCALE = 0.972
RIGHT_DISTANCE_SCALE = 0.972


# =========================================================
# 오도메트리 설정
# =========================================================

# 오도메트리 발행 주기: 20Hz
ODOM_UPDATE_PERIOD = 0.05

# 이 시간보다 UART 데이터가 오래되면 속도 0 처리
ENCODER_DATA_TIMEOUT = 0.5

# 디버그 출력
ODOM_DEBUG = True

# 디버그 출력 주기
ODOM_DEBUG_INTERVAL = 0.5


# =========================================================
# IMU 설정
# =========================================================

MPU6050_ADDRESS = 0x68

# IMU 읽기 주기: 100 Hz
IMU_UPDATE_PERIOD = 0.01

# 시작 시 자이로 Z bias 보정
IMU_CALIBRATION_SAMPLES = 500
IMU_CALIBRATION_INTERVAL = 0.005

# 좌회전 시 yaw 증가 기준
IMU_YAW_SIGN = 1.0

# 정지 상태의 작은 자이로 노이즈 제거
GYRO_DEAD_ZONE = 0.15


# =========================================================
# OdomPublisher
# =========================================================

class OdomPublisher(Node):

    def __init__(self):
        super().__init__("odom_publisher")

        # -------------------------------------------------
        # /odom publisher
        # -------------------------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        # -------------------------------------------------
        # TF broadcaster
        # -------------------------------------------------

        self.tf_broadcaster = (
            TransformBroadcaster(self)
        )

        # -------------------------------------------------
        # 로봇 위치 상태
        # -------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # -------------------------------------------------
        # 시간 상태
        # -------------------------------------------------

        self.last_time = (
            self.get_clock().now()
        )

        self.last_debug_time = (
            self.get_clock().now()
        )

        # -------------------------------------------------
        # MPU6050 상태
        # -------------------------------------------------

        self.data_lock = threading.Lock()
        self.running = True

        self.imu = None
        self.gyro_z_bias = 0.0

        self.imu_yaw_deg = 0.0
        self.imu_yaw_rad = 0.0

        self.last_imu_time = time.monotonic()
        self.previous_imu_theta = 0.0

        self.initialize_imu()

        self.imu_thread = threading.Thread(
            target=self.imu_update_loop,
            daemon=True
        )

        self.imu_thread.start()

        # -------------------------------------------------
        # 20Hz timer
        # -------------------------------------------------

        self.timer = self.create_timer(
            ODOM_UPDATE_PERIOD,
            self.update_odom
        )

        self.get_logger().info(
            "Odom Publisher Started"
        )

        self.get_logger().info(
            f"Wheel diameter="
            f"{WHEEL_DIAMETER:.3f} m, "
            f"wheel circumference="
            f"{WHEEL_CIRCUMFERENCE:.4f} m, "
            f"track width="
            f"{TRACK_WIDTH:.3f} m, "
            f"pulses/rev="
            f"{PULSES_PER_REV:.1f}"
        )


    # =====================================================
    # IMU 초기화 및 bias 보정
    # =====================================================

    def initialize_imu(self):
        self.imu = mpu6050(MPU6050_ADDRESS)

        self.get_logger().info(
            "MPU6050 connected. Keep robot completely still."
        )

        bias_sum = 0.0
        valid_samples = 0

        for _ in range(IMU_CALIBRATION_SAMPLES):
            try:
                gyro_data = self.imu.get_gyro_data()
                bias_sum += gyro_data["z"]
                valid_samples += 1

            except Exception as error:
                self.get_logger().warning(
                    f"IMU calibration read failed: {error}"
                )

            time.sleep(IMU_CALIBRATION_INTERVAL)

        if valid_samples == 0:
            raise RuntimeError(
                "No valid MPU6050 samples received"
            )

        self.gyro_z_bias = bias_sum / valid_samples

        with self.data_lock:
            self.imu_yaw_deg = 0.0
            self.imu_yaw_rad = 0.0

        self.previous_imu_theta = 0.0
        self.last_imu_time = time.monotonic()

        self.get_logger().info(
            f"MPU6050 calibration complete: "
            f"gyro Z bias={self.gyro_z_bias:+.4f} deg/s"
        )


    # =====================================================
    # IMU yaw 적분 (별도 스레드, 100Hz)
    # =====================================================

    def imu_update_loop(self):
        while self.running and rclpy.ok():
            loop_start = time.monotonic()

            try:
                gyro_data = self.imu.get_gyro_data()

                current_time = time.monotonic()
                dt = current_time - self.last_imu_time
                self.last_imu_time = current_time

                if dt <= 0.0 or dt > 0.1:
                    dt = IMU_UPDATE_PERIOD

                gyro_z = (
                    gyro_data["z"]
                    - self.gyro_z_bias
                ) * IMU_YAW_SIGN

                if abs(gyro_z) < GYRO_DEAD_ZONE:
                    gyro_z = 0.0

                with self.data_lock:
                    self.imu_yaw_deg += gyro_z * dt
                    self.imu_yaw_rad = math.radians(
                        self.imu_yaw_deg
                    )

            except Exception as error:
                self.get_logger().warning(
                    f"IMU read failed: {error}"
                )

            elapsed = time.monotonic() - loop_start
            sleep_time = IMU_UPDATE_PERIOD - elapsed

            if sleep_time > 0.0:
                time.sleep(sleep_time)


    # =====================================================
    # 오도메트리 업데이트
    # =====================================================

    def update_odom(self):
        now = self.get_clock().now()

        dt = (
            now - self.last_time
        ).nanoseconds / 1e9

        self.last_time = now

        if dt <= 0.0:
            return

        # 비정상적으로 긴 주기가 발생하면
        # 큰 거리 점프를 방지한다.
        if dt > 0.2:
            dt = ODOM_UPDATE_PERIOD

        # -------------------------------------------------
        # 엔코더 PPS 가져오기
        # -------------------------------------------------

        left_pps, right_pps = (
            motor.get_signed_encoder_speed()
        )

        response_age = (
            motor.get_last_response_age()
        )

        # UART 데이터가 오래되었다면 정지 상태로 처리
        if response_age > ENCODER_DATA_TIMEOUT:
            left_pps = 0.0
            right_pps = 0.0

        # -------------------------------------------------
        # PPS -> 바퀴 선속도 변환 (거리 보정값 적용)
        # -------------------------------------------------

        v_left = (
            left_pps
            / PULSES_PER_REV
            * WHEEL_CIRCUMFERENCE
            * LEFT_DISTANCE_SCALE
        )

        v_right = (
            right_pps
            / PULSES_PER_REV
            * WHEEL_CIRCUMFERENCE
            * RIGHT_DISTANCE_SCALE
        )

        # -------------------------------------------------
        # 선속도는 엔코더 기반
        # -------------------------------------------------

        linear_velocity = (
            v_left + v_right
        ) / 2.0

        # -------------------------------------------------
        # 방향각은 MPU6050 yaw 사용
        # -------------------------------------------------

        with self.data_lock:
            current_theta = self.imu_yaw_rad

        delta_theta = math.atan2(
            math.sin(
                current_theta
                - self.previous_imu_theta
            ),
            math.cos(
                current_theta
                - self.previous_imu_theta
            )
        )

        angular_velocity = delta_theta / dt

        middle_theta = (
            self.previous_imu_theta
            + delta_theta / 2.0
        )

        self.previous_imu_theta = current_theta
        self.theta = current_theta

        # theta를 -pi ~ +pi로 정규화
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # -------------------------------------------------
        # 위치 적분 (엔코더 선속도 + IMU 각도)
        # -------------------------------------------------

        delta_distance = (
            linear_velocity * dt
        )

        delta_x = (
            delta_distance
            * math.cos(middle_theta)
        )

        delta_y = (
            delta_distance
            * math.sin(middle_theta)
        )

        self.x += delta_x
        self.y += delta_y

        # -------------------------------------------------
        # Quaternion 계산
        # -------------------------------------------------

        quaternion_z = math.sin(
            self.theta / 2.0
        )

        quaternion_w = math.cos(
            self.theta / 2.0
        )

        # -------------------------------------------------
        # TF: odom -> base_link
        # -------------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = (
            now.to_msg()
        )

        transform.header.frame_id = (
            "odom"
        )

        transform.child_frame_id = (
            "base_link"
        )

        transform.transform.translation.x = (
            self.x
        )

        transform.transform.translation.y = (
            self.y
        )

        transform.transform.translation.z = (
            0.0
        )

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0

        transform.transform.rotation.z = (
            quaternion_z
        )

        transform.transform.rotation.w = (
            quaternion_w
        )

        self.tf_broadcaster.sendTransform(
            transform
        )

        # -------------------------------------------------
        # /odom 메시지 생성
        # -------------------------------------------------

        odom = Odometry()

        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # 위치
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # 방향
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0

        odom.pose.pose.orientation.z = (
            quaternion_z
        )

        odom.pose.pose.orientation.w = (
            quaternion_w
        )

        # 속도
        odom.twist.twist.linear.x = (
            linear_velocity
        )

        odom.twist.twist.linear.y = 0.0

        odom.twist.twist.angular.z = (
            angular_velocity
        )

        # -------------------------------------------------
        # Covariance
        # -------------------------------------------------

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        # -------------------------------------------------
        # 디버그 출력
        # -------------------------------------------------

        if ODOM_DEBUG:
            debug_elapsed = (
                now - self.last_debug_time
            ).nanoseconds / 1e9

            if (
                debug_elapsed
                >= ODOM_DEBUG_INTERVAL
            ):
                self.last_debug_time = now

                left_cmd, right_cmd = (
                    motor.get_last_command()
                )

                print(
                    "[ODOM] "
                    f"CMD=({left_cmd}, {right_cmd}) | "
                    f"PPS=({left_pps:.1f}, {right_pps:.1f}) | "
                    f"VEL=({v_left:.3f}, {v_right:.3f}) m/s | "
                    f"v={linear_velocity:.3f} m/s | "
                    f"w={angular_velocity:.3f} rad/s | "
                    f"x={self.x:.3f}, y={self.y:.3f}, "
                    f"theta={math.degrees(self.theta):+.2f} deg | "
                    f"IMU yaw={self.imu_yaw_deg:+.2f} deg | "
                    f"UART age={response_age:.3f}s"
                )


    # =====================================================
    # 종료 처리
    # =====================================================

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = OdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
