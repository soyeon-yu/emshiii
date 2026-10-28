#!/usr/bin/env python3

import json
import math
import os
import sys
import time

import rclpy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)


BASE_DIR = os.path.dirname(
    os.path.abspath(__file__)
)


# =========================================================
# 책 위치 로드
# =========================================================

def load_locations():

    with open(
        os.path.join(
            BASE_DIR,
            "book_locations.json"
        ),
        "r"
    ) as f:

        return json.load(f)


# =========================================================
# Yaw -> Quaternion
# =========================================================

def yaw_to_quat(yaw_deg):

    yaw = math.radians(
        yaw_deg
    )

    qz = math.sin(
        yaw / 2.0
    )

    qw = math.cos(
        yaw / 2.0
    )

    return qz, qw


# =========================================================
# Nav2 Pose 생성
# =========================================================

def make_pose(
    navigator,
    x,
    y,
    yaw
):

    pose = PoseStamped()

    pose.header.frame_id = "map"

    pose.header.stamp = (
        navigator
        .get_clock()
        .now()
        .to_msg()
    )

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    qz, qw = yaw_to_quat(
        yaw
    )

    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


# =========================================================
# Bool Topic Publish
# =========================================================

def publish_bool(
    publisher,
    value
):

    msg = Bool()

    msg.data = bool(
        value
    )

    publisher.publish(
        msg
    )


# =========================================================
# MAIN
# =========================================================

def main():

    rclpy.init()

    navigator = BasicNavigator()

    # =====================================================
    # 센서 ON/OFF Publisher
    # =====================================================

    sensor_power_pub = (
        navigator.create_publisher(
            Bool,
            "/app/sensor_power",
            10
        )
    )

    # =====================================================
    # Emergency Stop 상태
    # =====================================================

    emergency_state = {
        "active": False
    }

    # =====================================================
    # Emergency Stop Callback
    # =====================================================

    def emergency_stop_callback(
        msg
    ):

        new_state = bool(
            msg.data
        )

        old_state = (
            emergency_state[
                "active"
            ]
        )

        emergency_state[
            "active"
        ] = new_state

        # 상태가 변했을 때만 로그
        if new_state == old_state:
            return

        if new_state:

            print(
                "[SAFETY] 긴급정지 신호 감지"
            )

        else:

            print(
                "[SAFETY] 안전 조건 정상 복구"
            )

    # =====================================================
    # Emergency Stop Subscriber
    # =====================================================

    emergency_stop_sub = (
        navigator.create_subscription(
            Bool,
            "/app/emergency_stop",
            emergency_stop_callback,
            10
        )
    )

    navigation_running = False

    paused_by_emergency = False

    sensor_enabled = False

    final_result = None

    try:

        # =================================================
        # Nav2 활성화 대기
        # =================================================

        print(
            "Nav2 활성화 대기 중..."
        )

        navigator.waitUntilNav2Active()

        # =================================================
        # 책 위치 로드
        # =================================================

        locations = (
            load_locations()
        )

        # =================================================
        # Book ID
        # =================================================

        book_id = (
            sys.argv[1]
            if len(sys.argv) > 1
            else input(
                "책 ID 입력: "
            )
        )

        if book_id not in locations:

            print(
                "이동 실패: 존재하지 않는 ID"
            )

            return

        x, y, yaw = (
            locations[book_id]
        )

        print(
            f"\n이동 시작 : {book_id}"
        )

        print(
            f"x={x}, y={y}, yaw={yaw}"
        )

        # =================================================
        # 센서 안전 감시 시작
        # =================================================
        #
        # 중요한 부분:
        #
        # navigation action 하나의 상태가 아니라
        # 전체 "안내 세션" 동안 센서를 켜 둔다.
        #
        # 긴급정지로 Nav2 goal을 잠시 중단하더라도
        # 센서는 절대 끄지 않는다.
        #
        # 그래야 사람 복귀를 감지할 수 있다.
        # =================================================

        publish_bool(
            sensor_power_pub,
            True
        )

        sensor_enabled = True

        print(
            "[SENSOR] 안전 감시 시작"
        )

        # FollowerSafetyNode가 START를
        # 처리할 약간의 시간 확보
        time.sleep(
            0.2
        )

        # =================================================
        # 최초 Goal 생성
        # =================================================

        goal = make_pose(
            navigator,
            x,
            y,
            yaw
        )

        # =================================================
        # 최초 Nav2 Goal 전송
        # =================================================

        task = (
            navigator.goToPose(
                goal
            )
        )

        if task is None:

            print(
                "이동 실패: "
                "Nav2 goal 요청 거부"
            )

            final_result = (
                TaskResult.FAILED
            )

        else:

            navigation_running = True

        # =================================================
        # Navigation Main Loop
        # =================================================

        while (
            rclpy.ok()
            and final_result is None
        ):

            # =================================================
            # 정상 Navigation 실행 중
            # =================================================

            if navigation_running:

                # isTaskComplete() 내부에서
                # ROS callback도 처리됨.
                #
                # 따라서 /app/emergency_stop
                # callback 역시 여기서 실행된다.

                task_complete = (
                    navigator
                    .isTaskComplete()
                )

                # =============================================
                # Emergency Stop 발생
                # =============================================

                if (
                    emergency_state[
                        "active"
                    ]
                ):

                    # 아직 실제 Goal이 실행 중이면
                    # Nav2 navigation task 중단
                    if not task_complete:

                        print(
                            "[SAFETY] "
                            "현재 Nav2 goal "
                            "정지 처리"
                        )

                        navigator.cancelTask()

                    # 이 시점부터 navigation은
                    # 정지 상태로 전환
                    navigation_running = False

                    paused_by_emergency = True

                    print(
                        "[SAFETY] "
                        "추종자 복귀 대기 중"
                    )

                    # IMPORTANT:
                    #
                    # 여기에서
                    # /app/sensor_power=False를
                    # 보내면 안 됨.
                    #
                    # 센서는 계속 살아 있어야
                    # 사람 복귀를 감지할 수 있음.

                    continue

                # =============================================
                # 정상적으로 Navigation 종료
                # =============================================

                if task_complete:

                    final_result = (
                        navigator.getResult()
                    )

                    break

            # =================================================
            # Emergency Stop으로 정지 상태
            # =================================================

            else:

                # 현재 실행 중인 Nav2 task가 없으므로
                # 직접 spin_once해서
                # emergency_stop=False를 기다린다.

                rclpy.spin_once(
                    navigator,
                    timeout_sec=0.1
                )

                # 아직 사람이 복귀하지 않음
                if (
                    emergency_state[
                        "active"
                    ]
                ):

                    continue

                # =============================================
                # 사람 정상 복귀
                # =============================================

                if paused_by_emergency:

                    print(
                        "[SAFETY] "
                        "동일 목적지로 안내 재개"
                    )

                    # 기존 PoseStamped의 timestamp를
                    # 그대로 재사용하지 않고
                    # 현재 시간으로 다시 생성한다.

                    goal = make_pose(
                        navigator,
                        x,
                        y,
                        yaw
                    )

                    task = (
                        navigator.goToPose(
                            goal
                        )
                    )

                    if task is None:

                        print(
                            "이동 실패: "
                            "Nav2 재요청 거부"
                        )

                        final_result = (
                            TaskResult.FAILED
                        )

                        break

                    paused_by_emergency = False

                    navigation_running = True

                    print(
                        "[SAFETY] "
                        "안내 주행 재개"
                    )

        # =================================================
        # 최종 결과
        # =================================================

        if (
            final_result
            == TaskResult.SUCCEEDED
        ):

            print(
                "도착 완료!"
            )

        elif (
            final_result
            == TaskResult.CANCELED
        ):

            # 이 문구는 app_nav_bridge.py가
            # 실제 canceled 상태를 판단하는 데 사용함.
            print(
                "이동 취소됨"
            )

        elif (
            final_result
            == TaskResult.FAILED
        ):

            print(
                "이동 실패"
            )

        elif final_result is not None:

            print(
                "이동 실패: "
                "알 수 없는 Navigation 결과"
            )

    except KeyboardInterrupt:

        # =================================================
        # 프로그램 수동 종료
        # =================================================

        if navigation_running:

            try:

                navigator.cancelTask()

            except Exception:

                pass

        # app_nav_bridge가 canceled로
        # 판단할 수 있도록 최종 종료 때만 사용
        print(
            "이동 취소됨"
        )

    except Exception as error:

        print(
            f"이동 실패: {error}"
        )

        # 만약 예외 발생 시에도
        # Nav2 task가 실행 중이면 정지
        if navigation_running:

            try:

                navigator.cancelTask()

            except Exception:

                pass

    finally:

        # =================================================
        # 전체 안내 세션 종료
        # =================================================
        #
        # 여기서만 센서를 OFF한다.
        #
        # 성공 / 실패 / 실제 취소 /
        # 프로그램 종료 모두 포함.
        # =================================================

        if sensor_enabled:

            publish_bool(
                sensor_power_pub,
                False
            )

            print(
                "[SENSOR] 안전 감시 종료"
            )

            # ROS DDS가 메시지를 전송할 시간을
            # 조금 확보
            time.sleep(
                0.2
            )

        navigator.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
