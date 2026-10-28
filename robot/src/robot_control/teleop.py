import math
import re
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import serial
from mpu6050 import mpu6050


# =========================================================
# UART 설정
# =========================================================

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.02


# =========================================================
# 모터 명령 설정
# =========================================================

FORWARD_SPEED = 200
BACKWARD_SPEED = 200

TURN_SPEED = 200
TURN_DURATION = 1.0

# 모터 명령 및 yaw hold 제어 주기: 20 Hz
COMMAND_INTERVAL = 0.05

# ESP32에 보내는 명령 제한
MOTOR_COMMAND_LIMIT = 500


# =========================================================
# IMU 직진 보정 설정
# =========================================================

YAW_HOLD_KP = 2.0
YAW_HOLD_KI = 0.6
YAW_HOLD_INTEGRAL_LIMIT = 15.0
YAW_HOLD_DEADBAND_DEG = 0.5
YAW_HOLD_MAX_CORRECTION = 30.0
YAW_HOLD_MIN_COMMAND = 100
YAW_HOLD_LOG_INTERVAL = 0.5


# =========================================================
# 엔코더 거리 환산
# =========================================================

WHEEL_DIAMETER = 0.130
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
PULSES_PER_REV = 3793.0

DISTANCE_PER_PULSE = (
    WHEEL_CIRCUMFERENCE
    / PULSES_PER_REV
)

LEFT_DISTANCE_SCALE = 1.0729
RIGHT_DISTANCE_SCALE = 1.0729

# ESP32에서 signed PPS를 전송하는 경우 둘 다 1.0
LEFT_PPS_SIGN = 1.0
RIGHT_PPS_SIGN = 1.0


# =========================================================
# IMU 설정
# =========================================================

MPU6050_ADDRESS = 0x68

IMU_UPDATE_PERIOD = 0.01
IMU_CALIBRATION_SAMPLES = 500
IMU_CALIBRATION_INTERVAL = 0.005

# 현재 기준:
# 좌회전하면 yaw 증가
# 우회전하면 yaw 감소
IMU_YAW_SIGN = 1.0

# 작은 자이로 값은 정지 노이즈로 처리
GYRO_DEAD_ZONE = 0.15


# =========================================================
# IMU Low Pass Filter / Debug 설정
# =========================================================

# LPF 차단 주파수 [Hz]
IMU_LPF_CUTOFF_HZ = 8.0

# IMU 디버그 로그 출력 주기 [s]
IMU_DEBUG_INTERVAL = 0.2

# False: LPF는 계산/출력만 하고 실제 yaw에는 기존 보정 gyro 사용
# True : LPF 값을 실제 yaw 적분에 사용
# 처음 테스트에서는 반드시 False로 사용
USE_FILTERED_GYRO_FOR_YAW = False


# =========================================================
# ROS 설정
# =========================================================

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

ODOM_UPDATE_PERIOD = 0.05
ODOM_LOG_INTERVAL = 0.5


# =========================================================
# 보조 함수
# =========================================================

def normalize_angle(angle):
    """라디안 각도를 -pi ~ +pi 범위로 정규화합니다."""
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def normalize_angle_deg(angle_deg):
    """도 단위 각도를 -180 ~ +180 범위로 정규화합니다."""
    return math.degrees(
        normalize_angle(
            math.radians(angle_deg)
        )
    )


def clamp(value, minimum, maximum):
    """값을 minimum ~ maximum 범위로 제한합니다."""
    return max(minimum, min(maximum, value))


def yaw_to_quaternion(yaw):
    """2차원 yaw를 quaternion으로 변환합니다."""
    half_yaw = yaw * 0.5

    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)

    return 0.0, 0.0, qz, qw


# =========================================================
# Teleop + Odometry 노드
# =========================================================

class RobotTeleop(Node):

    def __init__(self):
        super().__init__("robot_teleop")

        # -------------------------------------------------
        # 스레드 및 동기화
        # -------------------------------------------------

        self.running = True

        self.serial_lock = threading.Lock()
        self.data_lock = threading.Lock()

        # -------------------------------------------------
        # 모터 명령 상태
        # -------------------------------------------------

        self.left_command = 0
        self.right_command = 0

        self.turning = False

        # stop / forward / backward
        self.drive_mode = "stop"

        # 직진 또는 후진 시작 시 저장하는 기준 yaw
        self.target_yaw_deg = None

        # yaw hold PI 제어의 적분 누적값
        self.yaw_hold_integral = 0.0
        self.last_yaw_hold_log_time = 0.0

        # -------------------------------------------------
        # UART 수신 상태
        # -------------------------------------------------

        self.left_pps = 0.0
        self.right_pps = 0.0
        self.last_uart_time = 0.0

        # -------------------------------------------------
        # IMU 상태
        # -------------------------------------------------

        self.imu = None
        self.gyro_z_bias = 0.0

        self.imu_yaw_deg = 0.0
        self.imu_yaw_rad = 0.0

        self.filtered_gyro_z = 0.0
        self.last_imu_debug_time = 0.0

        self.last_imu_time = time.monotonic()

        # -------------------------------------------------
        # Odometry 상태
        # -------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.last_odom_time = time.monotonic()
        self.previous_imu_theta = 0.0

        self.last_odom_log_time = 0.0

        # -------------------------------------------------
        # ROS Publisher
        # -------------------------------------------------

        self.odom_publisher = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # -------------------------------------------------
        # UART 연결
        # -------------------------------------------------

        self.serial = self.open_serial()

        # -------------------------------------------------
        # MPU6050 초기화 및 보정
        # -------------------------------------------------

        self.initialize_imu()

        # -------------------------------------------------
        # UART 수신 스레드
        # -------------------------------------------------

        self.uart_thread = threading.Thread(
            target=self.uart_receive_loop,
            daemon=True
        )
        self.uart_thread.start()

        # -------------------------------------------------
        # IMU 적분 스레드
        # -------------------------------------------------

        self.imu_thread = threading.Thread(
            target=self.imu_update_loop,
            daemon=True
        )
        self.imu_thread.start()

        # -------------------------------------------------
        # ROS Timer
        # -------------------------------------------------

        self.odom_timer = self.create_timer(
            ODOM_UPDATE_PERIOD,
            self.update_odometry
        )

        self.drive_control_timer = self.create_timer(
            COMMAND_INTERVAL,
            self.update_drive_control
        )

        self.get_logger().info("Robot teleop started")

        print(
            "[ODOM CONFIG] "
            f"wheel diameter={WHEEL_DIAMETER:.3f} m | "
            f"circumference={WHEEL_CIRCUMFERENCE:.6f} m | "
            f"PPR={PULSES_PER_REV:.1f} | "
            f"distance/pulse={DISTANCE_PER_PULSE:.9f} m"
        )

        print(
            "[IMU CONFIG] "
            f"LPF cutoff={IMU_LPF_CUTOFF_HZ:.1f} Hz | "
            f"use filtered gyro for yaw={USE_FILTERED_GYRO_FOR_YAW}"
        )

        self.print_help()

    # =====================================================
    # UART 초기화
    # =====================================================

    def open_serial(self):
        try:
            ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUDRATE,
                timeout=SERIAL_TIMEOUT
            )

            time.sleep(1.0)

            ser.reset_input_buffer()
            ser.reset_output_buffer()

            print(
                f"[SERIAL] Connected: "
                f"{SERIAL_PORT}, {SERIAL_BAUDRATE}"
            )

            return ser

        except serial.SerialException as error:
            print(f"[SERIAL ERROR] {error}")
            raise

    # =====================================================
    # IMU 초기화 및 bias 보정
    # =====================================================

    def initialize_imu(self):
        try:
            self.imu = mpu6050(MPU6050_ADDRESS)

            print("[IMU] MPU6050 connected")
            print("[IMU] Keep the robot completely still")
            print("[IMU] Calibrating gyro Z bias...")

            bias_sum = 0.0
            valid_samples = 0

            for _ in range(IMU_CALIBRATION_SAMPLES):
                try:
                    gyro_data = self.imu.get_gyro_data()

                    bias_sum += gyro_data["z"]
                    valid_samples += 1

                except Exception as error:
                    print(f"[IMU CALIBRATION WARNING] {error}")

                time.sleep(IMU_CALIBRATION_INTERVAL)

            if valid_samples == 0:
                raise RuntimeError(
                    "No valid MPU6050 samples received"
                )

            self.gyro_z_bias = bias_sum / valid_samples

            with self.data_lock:
                self.imu_yaw_deg = 0.0
                self.imu_yaw_rad = 0.0
                self.filtered_gyro_z = 0.0

            self.last_imu_time = time.monotonic()
            self.last_imu_debug_time = 0.0

            print(
                "[IMU] Calibration complete | "
                f"gyro Z bias={self.gyro_z_bias:+.4f} deg/s"
            )

        except Exception as error:
            print(f"[IMU ERROR] {error}")
            raise

    # =====================================================
    # IMU yaw 적분 + LPF 진단
    # =====================================================

    def imu_update_loop(self):
        while self.running:
            loop_start = time.monotonic()

            try:
                gyro_data = self.imu.get_gyro_data()

                current_time = time.monotonic()

                dt = current_time - self.last_imu_time
                self.last_imu_time = current_time

                # 비정상적으로 긴/잘못된 샘플 간격은 적분하지 않음
                if dt <= 0.0 or dt > 0.1:
                    if (
                        current_time - self.last_imu_debug_time
                        >= IMU_DEBUG_INTERVAL
                    ):
                        self.last_imu_debug_time = current_time
                        print(
                            "[IMU WARNING] "
                            f"abnormal dt={dt:.4f}s - sample skipped"
                        )
                else:
                    # -----------------------------------------
                    # 1. MPU6050 원본 Gyro Z
                    # -----------------------------------------

                    raw_gyro_z = (
                        gyro_data["z"]
                        * IMU_YAW_SIGN
                    )

                    # -----------------------------------------
                    # 2. 초기 bias 제거
                    # -----------------------------------------

                    corrected_gyro_z = (
                        gyro_data["z"]
                        - self.gyro_z_bias
                    ) * IMU_YAW_SIGN

                    # -----------------------------------------
                    # 3. 1차 Low Pass Filter
                    # -----------------------------------------

                    rc = (
                        1.0
                        / (
                            2.0
                            * math.pi
                            * IMU_LPF_CUTOFF_HZ
                        )
                    )

                    alpha = dt / (rc + dt)

                    self.filtered_gyro_z += (
                        alpha
                        * (
                            corrected_gyro_z
                            - self.filtered_gyro_z
                        )
                    )

                    # -----------------------------------------
                    # 4. 실제 yaw 적분에 사용할 gyro 선택
                    # -----------------------------------------

                    if USE_FILTERED_GYRO_FOR_YAW:
                        gyro_for_yaw = self.filtered_gyro_z
                    else:
                        gyro_for_yaw = corrected_gyro_z

                    # -----------------------------------------
                    # 5. Dead zone
                    # -----------------------------------------

                    if abs(gyro_for_yaw) < GYRO_DEAD_ZONE:
                        gyro_for_yaw = 0.0

                    # -----------------------------------------
                    # 6. yaw 적분
                    # -----------------------------------------

                    with self.data_lock:
                        self.imu_yaw_deg += (
                            gyro_for_yaw * dt
                        )

                        self.imu_yaw_rad = math.radians(
                            self.imu_yaw_deg
                        )

                        current_yaw = self.imu_yaw_deg

                    # -----------------------------------------
                    # 7. 진단 출력
                    # -----------------------------------------

                    if (
                        current_time - self.last_imu_debug_time
                        >= IMU_DEBUG_INTERVAL
                    ):
                        self.last_imu_debug_time = current_time

                        print(
                            "[IMU DEBUG] "
                            f"RAW={raw_gyro_z:+8.3f} deg/s | "
                            f"BIAS={self.gyro_z_bias:+8.3f} | "
                            f"CORR={corrected_gyro_z:+8.3f} deg/s | "
                            f"LPF={self.filtered_gyro_z:+8.3f} deg/s | "
                            f"USED={gyro_for_yaw:+8.3f} deg/s | "
                            f"YAW={current_yaw:+9.2f} deg | "
                            f"dt={dt * 1000.0:6.1f} ms"
                        )

            except Exception as error:
                print(f"[IMU READ WARNING] {error}")

            elapsed = time.monotonic() - loop_start
            sleep_time = IMU_UPDATE_PERIOD - elapsed

            if sleep_time > 0.0:
                time.sleep(sleep_time)

    # =====================================================
    # UART 모터 명령 전송
    # =====================================================

    def send_motor_command(self, left, right):
        left = int(round(left))
        right = int(round(right))

        left = int(
            clamp(
                left,
                -MOTOR_COMMAND_LIMIT,
                MOTOR_COMMAND_LIMIT
            )
        )

        right = int(
            clamp(
                right,
                -MOTOR_COMMAND_LIMIT,
                MOTOR_COMMAND_LIMIT
            )
        )

        command = f"{left},{right}\n"

        try:
            with self.serial_lock:
                self.serial.write(
                    command.encode("utf-8")
                )
                self.serial.flush()

            with self.data_lock:
                self.left_command = left
                self.right_command = right

            return True

        except serial.SerialException as error:
            print(f"[UART TX ERROR] {error}")
            return False

        except Exception as error:
            print(f"[UART TX WARNING] {error}")
            return False

    # =====================================================
    # UART 응답 파싱
    # =====================================================

    def parse_uart_response(self, line):
        """
        ESP32 응답 형식:
        OK,left,right,m1_pps,m2_pps
        """

        if not line:
            return

        if not line.startswith("OK"):
            if line == "ERR":
                print("[UART RX] ESP32 returned ERR")
            else:
                print(f"[UART RX] Invalid response: {line}")
            return

        numbers = re.findall(
            r"[-+]?\d+(?:\.\d+)?",
            line
        )

        if len(numbers) < 4:
            print(f"[UART RX] Parse failed: {line}")
            return

        try:
            received_left = int(float(numbers[0]))
            received_right = int(float(numbers[1]))

            m1_pps = float(numbers[2])
            m2_pps = float(numbers[3])

            m1_pps *= LEFT_PPS_SIGN
            m2_pps *= RIGHT_PPS_SIGN

            with self.data_lock:
                # 현재 기존 teleop.py 동작을 그대로 유지
                self.left_pps = m1_pps
                self.right_pps = m2_pps
                self.last_uart_time = time.monotonic()

            print(
                "[UART OK] "
                f"CMD L={received_left:5d}, "
                f"R={received_right:5d} | "
                f"M1={m1_pps:8.1f} pps, "
                f"M2={m2_pps:8.1f} pps"
            )

        except (ValueError, IndexError) as error:
            print(
                f"[UART RX PARSE ERROR] "
                f"{error} | data={line}"
            )

    # =====================================================
    # UART 수신 스레드
    # =====================================================

    def uart_receive_loop(self):
        receive_buffer = ""

        while self.running:
            try:
                with self.serial_lock:
                    available = self.serial.in_waiting

                    if available > 0:
                        raw_data = self.serial.read(available)
                    else:
                        raw_data = b""

                if raw_data:
                    receive_buffer += raw_data.decode(
                        "utf-8",
                        errors="ignore"
                    )

                    while "\n" in receive_buffer:
                        line, receive_buffer = (
                            receive_buffer.split("\n", 1)
                        )

                        line = line.strip()

                        if line:
                            self.parse_uart_response(line)

                else:
                    time.sleep(0.002)

            except serial.SerialException as error:
                print(f"[UART RX ERROR] {error}")
                time.sleep(0.1)

            except Exception as error:
                print(f"[UART THREAD ERROR] {error}")
                time.sleep(0.1)

    # =====================================================
    # 전진 시작
    # =====================================================

    def move_forward(self):
        if self.turning:
            print("[WARNING] Turning is in progress")
            return

        with self.data_lock:
            current_yaw = self.imu_yaw_deg

            if self.drive_mode != "forward":
                self.target_yaw_deg = current_yaw
                self.yaw_hold_integral = 0.0

            self.drive_mode = "forward"
            target_yaw = self.target_yaw_deg

        print(
            "[FORWARD START] "
            f"base={FORWARD_SPEED} | "
            f"target yaw={target_yaw:+.2f} deg"
        )

    # =====================================================
    # 후진 시작
    # =====================================================

    def move_backward(self):
        if self.turning:
            print("[WARNING] Turning is in progress")
            return

        with self.data_lock:
            current_yaw = self.imu_yaw_deg

            if self.drive_mode != "backward":
                self.target_yaw_deg = current_yaw
                self.yaw_hold_integral = 0.0

            self.drive_mode = "backward"
            target_yaw = self.target_yaw_deg

        print(
            "[BACKWARD START] "
            f"base={-BACKWARD_SPEED} | "
            f"target yaw={target_yaw:+.2f} deg"
        )

    # =====================================================
    # IMU yaw hold 제어
    # =====================================================

    def update_drive_control(self):
        if not self.running:
            return

        if self.turning:
            return

        with self.data_lock:
            drive_mode = self.drive_mode
            target_yaw_deg = self.target_yaw_deg
            current_yaw_deg = self.imu_yaw_deg

        if drive_mode == "stop":
            return

        if target_yaw_deg is None:
            return

        raw_yaw_error_deg = normalize_angle_deg(
            target_yaw_deg - current_yaw_deg
        )

        if abs(raw_yaw_error_deg) < YAW_HOLD_DEADBAND_DEG:
            yaw_error_deg = 0.0
        else:
            yaw_error_deg = raw_yaw_error_deg

        self.yaw_hold_integral += (
            raw_yaw_error_deg * COMMAND_INTERVAL
        )

        self.yaw_hold_integral = clamp(
            self.yaw_hold_integral,
            -YAW_HOLD_INTEGRAL_LIMIT,
            YAW_HOLD_INTEGRAL_LIMIT
        )

        correction = (
            YAW_HOLD_KP * yaw_error_deg
            + YAW_HOLD_KI * self.yaw_hold_integral
        )

        correction = clamp(
            correction,
            -YAW_HOLD_MAX_CORRECTION,
            YAW_HOLD_MAX_CORRECTION
        )

        if drive_mode == "forward":
            left_command = FORWARD_SPEED - correction
            right_command = FORWARD_SPEED + correction

            left_command = clamp(
                left_command,
                YAW_HOLD_MIN_COMMAND,
                MOTOR_COMMAND_LIMIT
            )

            right_command = clamp(
                right_command,
                YAW_HOLD_MIN_COMMAND,
                MOTOR_COMMAND_LIMIT
            )

        elif drive_mode == "backward":
            left_command = -BACKWARD_SPEED + correction
            right_command = -BACKWARD_SPEED - correction

            left_command = clamp(
                left_command,
                -MOTOR_COMMAND_LIMIT,
                -YAW_HOLD_MIN_COMMAND
            )

            right_command = clamp(
                right_command,
                -MOTOR_COMMAND_LIMIT,
                -YAW_HOLD_MIN_COMMAND
            )

        else:
            return

        self.send_motor_command(
            left_command,
            right_command
        )

        current_time = time.monotonic()

        if (
            current_time - self.last_yaw_hold_log_time
            >= YAW_HOLD_LOG_INTERVAL
        ):
            self.last_yaw_hold_log_time = current_time

            print(
                "[YAW HOLD] "
                f"mode={drive_mode} | "
                f"target={target_yaw_deg:+.2f} deg | "
                f"current={current_yaw_deg:+.2f} deg | "
                f"error={yaw_error_deg:+.2f} deg | "
                f"integral={self.yaw_hold_integral:+.2f} | "
                f"correction={correction:+.1f} | "
                f"CMD=({int(round(left_command))}, "
                f"{int(round(right_command))})"
            )

    # =====================================================
    # 모터 정지
    # =====================================================

    def stop_motor(self):
        with self.data_lock:
            self.drive_mode = "stop"
            self.target_yaw_deg = None
            self.yaw_hold_integral = 0.0

        self.send_motor_command(0, 0)

        with self.data_lock:
            self.left_command = 0
            self.right_command = 0

            self.left_pps = 0.0
            self.right_pps = 0.0

    # =====================================================
    # 시간 기반 제자리 회전
    # =====================================================

    def perform_turn(self, direction):
        if self.turning:
            return

        with self.data_lock:
            self.drive_mode = "stop"
            self.target_yaw_deg = None
            self.yaw_hold_integral = 0.0

        self.turning = True

        try:
            if direction == "left":
                left = -TURN_SPEED
                right = TURN_SPEED
                label = "LEFT"
            else:
                left = TURN_SPEED
                right = -TURN_SPEED
                label = "RIGHT"

            with self.data_lock:
                start_yaw = self.imu_yaw_deg

            print()
            print(
                f"[TURN {label}] "
                f"L={left}, R={right} | "
                f"duration={TURN_DURATION:.2f}s"
            )

            start_time = time.monotonic()

            while (
                self.running
                and time.monotonic() - start_time < TURN_DURATION
            ):
                self.send_motor_command(left, right)
                time.sleep(COMMAND_INTERVAL)

            self.stop_motor()

            with self.data_lock:
                end_yaw = self.imu_yaw_deg

            yaw_change = normalize_angle_deg(
                end_yaw - start_yaw
            )

            print(
                f"[TURN {label} END] "
                f"yaw change={yaw_change:+.2f} deg | "
                f"total yaw={end_yaw:+.2f} deg"
            )

        finally:
            self.turning = False

    # =====================================================
    # Odometry 업데이트
    # =====================================================

    def update_odometry(self):
        current_time = time.monotonic()

        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time

        if dt <= 0.0 or dt > 0.2:
            return

        with self.data_lock:
            left_command = self.left_command
            right_command = self.right_command

            left_pps = self.left_pps
            right_pps = self.right_pps

            imu_theta = self.imu_yaw_rad
            imu_yaw_deg = self.imu_yaw_deg

            last_uart_time = self.last_uart_time

        if last_uart_time == 0.0:
            uart_age = float("inf")
        else:
            uart_age = current_time - last_uart_time

        if uart_age > 0.3:
            left_pps = 0.0
            right_pps = 0.0

        if left_command == 0:
            left_pps = 0.0

        if right_command == 0:
            right_pps = 0.0

        left_velocity = (
            left_pps
            * DISTANCE_PER_PULSE
            * LEFT_DISTANCE_SCALE
        )

        right_velocity = (
            right_pps
            * DISTANCE_PER_PULSE
            * RIGHT_DISTANCE_SCALE
        )

        linear_velocity = (
            left_velocity + right_velocity
        ) * 0.5

        current_theta = imu_theta

        delta_theta = normalize_angle(
            current_theta - self.previous_imu_theta
        )

        angular_velocity = delta_theta / dt

        self.previous_imu_theta = current_theta
        self.theta = current_theta

        middle_theta = self.theta - delta_theta * 0.5

        self.x += (
            linear_velocity
            * math.cos(middle_theta)
            * dt
        )

        self.y += (
            linear_velocity
            * math.sin(middle_theta)
            * dt
        )

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        self.publish_odometry(
            linear_velocity,
            angular_velocity
        )

        if (
            current_time - self.last_odom_log_time
            >= ODOM_LOG_INTERVAL
        ):
            self.last_odom_log_time = current_time

            if math.isinf(uart_age):
                uart_age_text = "N/A"
            else:
                uart_age_text = f"{uart_age:.2f}s"

            print(
                "[ODOM] "
                f"CMD=({left_command}, {right_command}) | "
                f"PPS=({left_pps:.1f}, {right_pps:.1f}) | "
                f"VEL=({left_velocity:.3f}, "
                f"{right_velocity:.3f}) m/s | "
                f"v={linear_velocity:.3f} m/s | "
                f"w={angular_velocity:.3f} rad/s | "
                f"x={self.x:.3f}, "
                f"y={self.y:.3f}, "
                f"theta={math.degrees(self.theta):+.1f} deg | "
                f"IMU yaw={imu_yaw_deg:+.1f} deg | "
                f"UART age={uart_age_text}"
            )

    # =====================================================
    # Odometry 및 TF 발행
    # =====================================================

    def publish_odometry(
        self,
        linear_velocity,
        angular_velocity
    ):
        stamp = self.get_clock().now().to_msg()

        qx, qy, qz, qw = yaw_to_quaternion(
            normalize_angle(self.theta)
        )

        odom_message = Odometry()

        odom_message.header.stamp = stamp
        odom_message.header.frame_id = ODOM_FRAME
        odom_message.child_frame_id = BASE_FRAME

        odom_message.pose.pose.position.x = self.x
        odom_message.pose.pose.position.y = self.y
        odom_message.pose.pose.position.z = 0.0

        odom_message.pose.pose.orientation.x = qx
        odom_message.pose.pose.orientation.y = qy
        odom_message.pose.pose.orientation.z = qz
        odom_message.pose.pose.orientation.w = qw

        odom_message.twist.twist.linear.x = linear_velocity
        odom_message.twist.twist.linear.y = 0.0
        odom_message.twist.twist.linear.z = 0.0

        odom_message.twist.twist.angular.x = 0.0
        odom_message.twist.twist.angular.y = 0.0
        odom_message.twist.twist.angular.z = angular_velocity

        odom_message.pose.covariance[0] = 0.03
        odom_message.pose.covariance[7] = 0.03
        odom_message.pose.covariance[14] = 99999.0
        odom_message.pose.covariance[21] = 99999.0
        odom_message.pose.covariance[28] = 99999.0
        odom_message.pose.covariance[35] = 0.02

        odom_message.twist.covariance[0] = 0.05
        odom_message.twist.covariance[7] = 0.05
        odom_message.twist.covariance[14] = 99999.0
        odom_message.twist.covariance[21] = 99999.0
        odom_message.twist.covariance[28] = 99999.0
        odom_message.twist.covariance[35] = 0.03

        self.odom_publisher.publish(odom_message)

        transform = TransformStamped()

        transform.header.stamp = stamp
        transform.header.frame_id = ODOM_FRAME
        transform.child_frame_id = BASE_FRAME

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

    # =====================================================
    # Odometry 및 IMU yaw 초기화
    # =====================================================

    def reset_odometry(self):
        with self.data_lock:
            self.drive_mode = "stop"
            self.target_yaw_deg = None
            self.yaw_hold_integral = 0.0

            self.imu_yaw_deg = 0.0
            self.imu_yaw_rad = 0.0
            self.filtered_gyro_z = 0.0

            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

            self.previous_imu_theta = 0.0

            self.left_pps = 0.0
            self.right_pps = 0.0

        self.last_imu_time = time.monotonic()
        self.last_odom_time = time.monotonic()

        print(
            "[RESET] x=0, y=0, theta=0, "
            "IMU yaw=0, LPF state=0"
        )

    # =====================================================
    # 키보드 안내
    # =====================================================

    def print_help(self):
        print()
        print("==========================================")
        print(" Robot Teleoperation + IMU Yaw Hold")
        print("==========================================")
        print(" w : forward with yaw hold")
        print(" s : backward with yaw hold")
        print(" a : timed left pivot turn")
        print(" d : timed right pivot turn")
        print(" x / space : stop")
        print(" r : reset odometry and IMU yaw")
        print(" h : help")
        print(" q : quit")
        print("==========================================")
        print()

    # =====================================================
    # 키 입력 처리
    # =====================================================

    def handle_key(self, key):
        if key == "w":
            self.move_forward()

        elif key == "s":
            self.move_backward()

        elif key == "a":
            if not self.turning:
                turn_thread = threading.Thread(
                    target=self.perform_turn,
                    args=("left",),
                    daemon=True
                )
                turn_thread.start()

        elif key == "d":
            if not self.turning:
                turn_thread = threading.Thread(
                    target=self.perform_turn,
                    args=("right",),
                    daemon=True
                )
                turn_thread.start()

        elif key == "x" or key == " ":
            self.stop_motor()
            print("[STOP] Motor stopped")

        elif key == "r":
            self.stop_motor()
            self.reset_odometry()

        elif key == "h":
            self.print_help()

        elif key == "q":
            self.running = False

    # =====================================================
    # 종료
    # =====================================================

    def shutdown(self):
        self.running = False

        try:
            self.stop_motor()
            time.sleep(0.1)
        except Exception:
            pass

        try:
            if self.serial is not None:
                self.serial.close()
        except Exception:
            pass

        print("[SHUTDOWN] Motor stopped")


# =========================================================
# 키보드 입력 스레드
# =========================================================

def keyboard_loop(node):
    old_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok() and node.running:
            readable, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                0.05
            )

            if readable:
                key = sys.stdin.read(1).lower()
                node.handle_key(key)

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings
        )


# =========================================================
# 메인
# =========================================================

def main(args=None):
    rclpy.init(args=args)

    node = None
    keyboard_thread = None

    try:
        node = RobotTeleop()

        keyboard_thread = threading.Thread(
            target=keyboard_loop,
            args=(node,),
            daemon=True
        )
        keyboard_thread.start()

        while rclpy.ok() and node.running:
            rclpy.spin_once(
                node,
                timeout_sec=0.05
            )

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f"[FATAL ERROR] {error}")

    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
