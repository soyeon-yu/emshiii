import rclpy

from rclpy.node import Node

from rclpy.executors import (
    MultiThreadedExecutor
)

from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import motor

from odom_publisher import (
    OdomPublisher
)

from follower_safety_node import (
    FollowerSafetyNode
)


# =========================================================
# 로봇 파라미터
# =========================================================

TRACK_WIDTH = 0.378


# =========================================================
# CMD <-> 속도 변환
# =========================================================

# 측정값:
# CMD=160 -> 약 0.089 m/s

CMD_PER_MPS = 1797.0

MOTOR_LIMIT = 500


class CmdVelToMotor(Node):

    """
    /cmd_vel을 받아 좌우 바퀴 명령으로 변환한다.

    /app/emergency_stop=True 상태에서는
    모든 주행 명령을 차단하고 모터를 정지시킨다.

    odom 계산과 발행은 OdomPublisher가 담당한다.
    """

    def __init__(self):

        super().__init__(
            "cmd_vel_to_motor"
        )

        # =================================================
        # Callback Group
        # =================================================
        #
        # cmd_vel과 emergency_stop을 같은
        # MutuallyExclusiveCallbackGroup에 넣는다.
        #
        # 긴급정지 callback과 모터 명령 callback이
        # 동시에 실행되는 race condition을 방지한다.
        # =================================================

        self.cmd_group = (
            MutuallyExclusiveCallbackGroup()
        )

        # =================================================
        # 긴급정지 상태
        # =================================================

        self.emergency_stop = False

        # =================================================
        # /cmd_vel Subscriber
        # =================================================

        self.cmd_sub = (
            self.create_subscription(
                Twist,
                "/cmd_vel",
                self.cmd_callback,
                10,
                callback_group=self.cmd_group
            )
        )

        # =================================================
        # /app/emergency_stop Subscriber
        # =================================================

        self.estop_sub = (
            self.create_subscription(
                Bool,
                "/app/emergency_stop",
                self.emergency_stop_callback,
                10,
                callback_group=self.cmd_group
            )
        )

        self.get_logger().info(
            "cmd_vel_to_motor started"
        )

        self.get_logger().info(
            "Emergency Stop subscriber: "
            "/app/emergency_stop"
        )

    # =====================================================
    # Emergency Stop Callback
    # =====================================================

    def emergency_stop_callback(
        self,
        msg
    ):

        enabled = bool(
            msg.data
        )

        # -------------------------------------------------
        # 긴급정지 활성화
        # -------------------------------------------------

        if enabled:

            # 처음 True로 전환된 경우
            if not self.emergency_stop:

                self.get_logger().error(
                    "EMERGENCY STOP ACTIVATED"
                )

            self.emergency_stop = True

            # 즉시 실제 모터에 0,0 전송
            stop_ok = (
                motor.stop_robot()
            )

            if not stop_ok:

                self.get_logger().error(
                    "긴급정지 모터 명령 전송 실패"
                )

            return

        # -------------------------------------------------
        # 긴급정지 해제
        # -------------------------------------------------

        if self.emergency_stop:

            self.get_logger().warning(
                "EMERGENCY STOP RELEASED"
            )

        self.emergency_stop = False

        # 해제 시 바로 움직이지 않는다.
        #
        # 이후 새 /cmd_vel 명령이 들어와야
        # 다시 주행한다.

    # =====================================================
    # /cmd_vel Callback
    # =====================================================

    def cmd_callback(
        self,
        msg
    ):

        # =================================================
        # 긴급정지 상태 확인
        # =================================================
        #
        # Nav2가 계속 cmd_vel을 발행하더라도
        # emergency_stop이 True이면
        # 모든 주행 명령을 차단한다.
        # =================================================

        if self.emergency_stop:

            motor.stop_robot()

            self.get_logger().warning(
                "[E-STOP] /cmd_vel blocked"
            )

            return

        # =================================================
        # 정상 주행
        # =================================================

        linear = (
            msg.linear.x
        )

        angular = (
            msg.angular.z
        )

        # =================================================
        # 차동구동 속도 계산
        # =================================================

        left_speed = (
            linear
            - angular
            * TRACK_WIDTH
            / 2.0
        )

        right_speed = (
            linear
            + angular
            * TRACK_WIDTH
            / 2.0
        )

        # =================================================
        # m/s -> 모터 명령
        # =================================================

        left_cmd = int(
            left_speed
            * CMD_PER_MPS
        )

        right_cmd = int(
            right_speed
            * CMD_PER_MPS
        )

        # =================================================
        # 모터 명령 제한
        # =================================================

        left_cmd = max(
            -MOTOR_LIMIT,
            min(
                MOTOR_LIMIT,
                left_cmd
            )
        )

        right_cmd = max(
            -MOTOR_LIMIT,
            min(
                MOTOR_LIMIT,
                right_cmd
            )
        )

        # =================================================
        # ESP32 전송
        # =================================================

        motor.send_motor(
            left_cmd,
            right_cmd
        )

        print(
            f"[CMD_VEL] "
            f"linear={linear:.3f} "
            f"angular={angular:.3f} "
            f"CMD=({left_cmd},{right_cmd})"
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    # =====================================================
    # ROS2 Node 생성
    # =====================================================

    cmd_node = (
        CmdVelToMotor()
    )

    odom_node = (
        OdomPublisher()
    )

    safety_node = (
        FollowerSafetyNode()
    )

    # =====================================================
    # MultiThreaded Executor
    # =====================================================

    executor = (
        MultiThreadedExecutor(
            num_threads=4
        )
    )

    executor.add_node(
        cmd_node
    )

    executor.add_node(
        odom_node
    )

    executor.add_node(
        safety_node
    )

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        # =================================================
        # 센서 OFF
        # =================================================

        if rclpy.ok():

            executor.spin_once(
                timeout_sec=0.2
            )

        # =================================================
        # 모터 강제 정지
        # =================================================

        motor.stop_robot()

        motor.close_motor()

        # =================================================
        # Node 정리
        # =================================================

        cmd_node.destroy_node()

        odom_node.destroy_node()

        safety_node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
