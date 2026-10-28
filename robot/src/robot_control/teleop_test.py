import threading
import time

import rclpy

import teleop
from teleop import (
    RobotTeleop, keyboard_loop, clamp, normalize_angle_deg,
    COMMAND_INTERVAL
)

# ---------- 테스트용 설정 (여기만 고치면 됨) ----------
TURN_ANGLE_DEG = 30.0      # 한 번 누를 때 도는 각도
TURN_MAX_SPEED = 220
TURN_MIN_SPEED = 110       # 안 돌면 140으로 올려
TURN_KP = 6.0
TURN_TOLERANCE_DEG = 1.5
TURN_TIMEOUT = 4.0


class RobotTeleopTest(RobotTeleop):

    def perform_turn(self, direction):
        if self.turning:
            return

        with self.data_lock:
            self.drive_mode = "stop"
            self.target_yaw_deg = None
            self.yaw_hold_integral = 0.0
            start_yaw = self.imu_yaw_deg

        self.turning = True

        try:
            sign = 1.0 if direction == "left" else -1.0
            label = "LEFT" if direction == "left" else "RIGHT"
            target_yaw = start_yaw + sign * TURN_ANGLE_DEG

            print()
            print(f"[TEST TURN {label}] target={TURN_ANGLE_DEG:.1f} deg")

            start_time = time.monotonic()
            reached = False

            while (
                self.running
                and time.monotonic() - start_time < TURN_TIMEOUT
            ):
                with self.data_lock:
                    current_yaw = self.imu_yaw_deg

                error = target_yaw - current_yaw

                if abs(error) <= TURN_TOLERANCE_DEG:
                    reached = True
                    break

                speed = clamp(
                    abs(error) * TURN_KP,
                    TURN_MIN_SPEED,
                    TURN_MAX_SPEED
                )

                if error > 0:
                    left, right = -speed, speed
                else:
                    left, right = speed, -speed

                self.send_motor_command(left, right)
                time.sleep(COMMAND_INTERVAL)

            self.stop_motor()
            time.sleep(0.2)

            with self.data_lock:
                end_yaw = self.imu_yaw_deg

            yaw_change = normalize_angle_deg(end_yaw - start_yaw)

            print(
                f"[TEST TURN {label} END] reached={reached} | "
                f"yaw change={yaw_change:+.2f} deg | "
                f"total yaw={end_yaw:+.2f} deg"
            )

        finally:
            self.turning = False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = RobotTeleopTest()

        keyboard_thread = threading.Thread(
            target=keyboard_loop, args=(node,), daemon=True
        )
        keyboard_thread.start()

        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
