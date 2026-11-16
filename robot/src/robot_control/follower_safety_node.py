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

# [수정] /app/follower_check_enable=True 신호가
# 이 시간(초) 이상 끊기면 감시를 OFF로 되돌린다.
# (bridge의 heartbeat 주기 0.5초보다 충분히 길게)
ENABLE_TIMEOUT = 1.5

PI_API_KEY = os.environ.get("PI_API_KEY", "")

ALERT_URL = "https://what-small-smart-3-1.onrender.com/alert"

ALERT_COOLDOWN_SEC = 3.0


class FollowerSafetyNode(Node):
    """
    초음파 측정은 항상 유지하지만,
    이탈 판정(E-stop / 앱 경고)은 감시가 ON일 때만 한다.

    - 기본값은 감시 OFF
    - /app/follower_check_enable = True 신호가 계속 들어오는 동안만 ON
      (신호가 ENABLE_TIMEOUT초 끊기면 자동으로 OFF)
    - 거리 > 80cm 이면 /app/emergency_stop = True
    - 거리 <= 80cm 상태가 연속 3회 확인되면
      /app/emergency_stop = False
    - 감시가 OFF로 바뀌는 순간, 걸려 있던 E-stop도 해제한다
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

        # 감시 ON/OFF 제어용 구독
        self.enable_sub = self.create_subscription(
            Bool,
            "/app/follower_check_enable",
            self.enable_callback,
            10
        )

        # [수정] 기본값 OFF
        self.enabled_until = 0.0
        self.check_disabled = True

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

        self.get_logger().info(
            "이탈 감시 기본값: OFF "
            "(follower_check_enable=True 수신 중에만 ON)"
        )

        # -------------------------------------------------
        # 노드 시작과 동시에 초음파 측정 시작
        # (측정은 항상 켜두고, 판정만 enable 신호로 제어)
        # -------------------------------------------------
        start_ok = motor.send_sensor_start()

        if start_ok:
            self.sensing_enabled = True

            self.get_logger().info(
                "ESP32 초음파 센서 START 전송 완료"
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


    def enable_callback(self, msg):
        # [수정] True 신호가 계속 와야 감시가 유지된다.
        if msg.data:
            self.enabled_until = (
                time.monotonic()
                + ENABLE_TIMEOUT
            )
        else:
            # False: 즉시 감시 OFF
            self.enabled_until = 0.0


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

        # =================================================
        # 감시 ON/OFF 확인
        # =================================================
        # [수정] enabled_until 이전일 때만 감시 ON
        disabled = (
            time.monotonic()
            >= self.enabled_until
        )

        # 상태가 바뀐 순간 한 번만 처리
        if disabled != self.check_disabled:
            self.check_disabled = disabled
            self.recovery_streak = 0
            self.last_processed_sample_time = None

            if disabled:
                self.get_logger().info(
                    "추종자 이탈 감시 OFF"
                )

                # 꺼지는 순간 한 번만, 내부 상태와 무관하게 E-stop 해제
                self.is_dropped_out = False
                self.publish_estop(False)
            else:
                self.get_logger().info(
                    "추종자 이탈 감시 ON"
                )

        if disabled:
            # 감시 OFF: 이탈 판정은 하지 않는다.
            # (초음파 측정 자체는 계속 유지됨)
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
