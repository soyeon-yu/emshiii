# follower_safety_node.py

import os
import time
import requests

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

import motor


# =========================================================
# 안전 감지 설정
# =========================================================

DROPOUT_THRESHOLD_CM = 80.0

RECOVERY_CONFIRM_COUNT = 3

SENSOR_DATA_TIMEOUT = 3.0

CHECK_PERIOD = 0.1

ESTOP_REPUBLISH_INTERVAL = 1.0

PI_API_KEY = os.environ.get("PI_API_KEY", "")

ALERT_URL = "https://what-small-smart-3-1.onrender.com/alert"

ALERT_COOLDOWN_SEC = 3.0


class FollowerSafetyNode(Node):
    """
    노드가 실행되는 동안 초음파 센서를 계속 감시한다.

    - 노드 시작 시 ESP32에 START 전송
    - 거리 > 80cm 이면 /app/emergency_stop = True
    - 긴급정지 상태에서도 초음파 측정은 계속 유지
    - 거리 <= 80cm 상태가 연속 3회 확인되면
      /app/emergency_stop = False
    """

    def __init__(self):
        super().__init__("follower_safety_node")

        self.estop_pub = self.create_publisher(
            Bool,
            "/app/emergency_stop",
            10
        )

        self.timer = self.create_timer(
            CHECK_PERIOD,
            self.check_sensor
        )

        self.sensing_enabled = False
        self.is_dropped_out = False
        self.recovery_streak = 0

        self.last_processed_sample_time = None
        self.last_stale_log_time = 0.0
        self.last_estop_publish_time = 0.0
        self.last_alert_sent_at = 0.0

        self.get_logger().info(
            "Follower Safety Node Started"
        )

        self.get_logger().info(
            f"거리 기준: "
            f"{DROPOUT_THRESHOLD_CM:.1f}cm"
        )

        # -------------------------------------------------
        # 노드 시작과 동시에 초음파 측정 시작
        # -------------------------------------------------
        start_ok = motor.send_sensor_start()

        if start_ok:
            self.sensing_enabled = True

            self.get_logger().info(
                "ESP32 초음파 센서 START 전송 완료"
            )

            self.get_logger().info(
                "초음파 안전 감시 상시 활성화"
            )

        else:
            self.sensing_enabled = False

            self.get_logger().error(
                "ESP32 센서 START 명령 전송 실패"
            )

        if not PI_API_KEY:
            self.get_logger().warning(
                "PI_API_KEY 환경변수가 설정되지 않았습니다. "
                "로봇 E-stop은 동작하지만 웹 /alert 전송은 실패할 수 있습니다."
            )


    def publish_estop(self, value):
        msg = Bool()
        msg.data = bool(value)

        self.estop_pub.publish(msg)

        self.last_estop_publish_time = time.monotonic()

        if msg.data:
            self.get_logger().warn(
                "/app/emergency_stop = True"
            )
        else:
            self.get_logger().info(
                "/app/emergency_stop = False"
            )


    def send_alert(self, distance_cm):
        now = time.time()

        if (
            now - self.last_alert_sent_at
            < ALERT_COOLDOWN_SEC
        ):
            return

        self.last_alert_sent_at = now

        if not PI_API_KEY:
            self.get_logger().warning(
                "PI_API_KEY가 없어 웹 알림 전송을 건너뜁니다."
            )
            return

        try:
            response = requests.post(
                ALERT_URL,
                json={
                    "distance": float(distance_cm),
                    "api_key": PI_API_KEY
                },
                timeout=5.0
            )

            if response.ok:
                self.get_logger().info(
                    "Render 알림 전송 완료"
                )
            else:
                self.get_logger().warning(
                    f"Render 알림 전송 실패: "
                    f"status={response.status_code}, "
                    f"body={response.text}"
                )

        except requests.RequestException as error:
            self.get_logger().error(
                f"Render 알림 요청 오류: {error}"
            )


    def check_sensor(self):
        if not self.sensing_enabled:
            return

        # motor.py 수정본:
        # distance_cm, age_sec 두 값만 반환
        distance_cm, age_sec = (
            motor.get_sensor_data()
        )

        # 아직 SENSOR 데이터를 한 번도 받지 못한 상태
        if age_sec == float("inf"):
            return

        # 마지막 센서값이 너무 오래된 경우
        if age_sec > SENSOR_DATA_TIMEOUT:
            now = time.monotonic()

            if (
                now - self.last_stale_log_time
                >= 1.0
            ):
                self.last_stale_log_time = now

                self.get_logger().warning(
                    f"센서 데이터가 오래되었습니다: "
                    f"age={age_sec:.2f}s"
                )

            return

        # 같은 SENSOR 샘플을 0.1초 timer가 중복 처리하지 않도록 방지
        sample_time = (
            time.monotonic()
            - age_sec
        )

        if self.last_processed_sample_time is not None:
            sample_difference = abs(
                sample_time
                - self.last_processed_sample_time
            )

            if sample_difference < 0.05:
                return

        self.last_processed_sample_time = sample_time

        # ESP32에서 SENSOR,NA가 온 경우
        if distance_cm is None:
            self.get_logger().warning(
                "초음파 거리 측정 실패"
            )
            return

        self.get_logger().info(
            f"distance={distance_cm:.2f}cm"
        )

        dropout_detected = (
            distance_cm > DROPOUT_THRESHOLD_CM
        )

        # =================================================
        # 80cm 초과 → 즉시 긴급정지
        # =================================================
        if dropout_detected:
            self.recovery_streak = 0

            if not self.is_dropped_out:
                self.is_dropped_out = True

                self.get_logger().error(
                    f"추종자 이탈 감지: "
                    f"거리 초과 "
                    f"({distance_cm:.1f}cm)"
                )

                self.publish_estop(True)

                self.send_alert(
                    distance_cm
                )

            else:
                # 이미 E-stop 중이어도 주기적으로 True 재발행
                now = time.monotonic()

                if (
                    now - self.last_estop_publish_time
                    >= ESTOP_REPUBLISH_INTERVAL
                ):
                    self.publish_estop(True)

            # 여기서 모터만 정지 상태가 유지되고,
            # 초음파 check_sensor() timer는 계속 실행된다.
            return

        # =================================================
        # 80cm 이하 → 연속 확인 후 긴급정지 해제
        # =================================================
        if self.is_dropped_out:
            self.recovery_streak += 1

            self.get_logger().info(
                f"복귀 확인 "
                f"{self.recovery_streak}/"
                f"{RECOVERY_CONFIRM_COUNT}"
            )

            if (
                self.recovery_streak
                >= RECOVERY_CONFIRM_COUNT
            ):
                self.is_dropped_out = False
                self.recovery_streak = 0

                self.get_logger().info(
                    "추종자 정상 복귀 확인"
                )

                self.publish_estop(False)

        else:
            self.recovery_streak = 0


def main(args=None):
    rclpy.init(args=args)

    node = FollowerSafetyNode()

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
