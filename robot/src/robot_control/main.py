# main.py
import time
import rclpy

from lidar import LidarReader
from navigation import decide, FRONT_SAFE
from motor import (
    drive_forward,
    drive_backward,
    turn_left_arc,
    turn_right_arc,
    stop_robot,
    close_motor
)
from odom_publisher import OdomPublisher

STATE_DRIVE = "DRIVE"
STATE_STOP = "STOP"
STATE_BACKUP = "BACKUP"
STATE_TURN = "TURN"

STOP_TIME = 0.3
BACKUP_TIME = 0.8
BACKUP_TIME_DEAD_END = 1.5
TURN_TIME_MAX = 3.0
MAX_ESCAPE_LOOPS = 8


def main():
    rclpy.init()
    lidar = LidarReader()
    odom = OdomPublisher()

    print("Autonomous Drive Start")

    state = STATE_DRIVE
    state_start = time.time()
    turn_direction = "TURN_RIGHT"
    escape_count = 0
    is_dead_end = False

    try:
        while rclpy.ok():
            rclpy.spin_once(lidar, timeout_sec=0.05)
            rclpy.spin_once(odom, timeout_sec=0.01)

            front, left, right = lidar.get_regions()
            now = time.time()

            if state == STATE_DRIVE:
                cmd = decide(front, left, right)

                if cmd == "FORWARD":
                    drive_forward(160)

                elif cmd in ("OBSTACLE_FRONT", "DEAD_END"):
                    stop_robot()
                    turn_direction = "TURN_LEFT" if left > right else "TURN_RIGHT"
                    is_dead_end = (cmd == "DEAD_END")
                    state = STATE_STOP
                    state_start = now

                elif cmd == "OBSTACLE_LEFT":
                    turn_right_arc(260, 0)

                elif cmd == "OBSTACLE_RIGHT":
                    turn_left_arc(260, 0)

                print(f"Front: {front:.2f} | Left: {left:.2f} | Right: {right:.2f} | STATE: {state} | CMD: {cmd}")

            elif state == STATE_STOP:
                stop_robot()
                if now - state_start >= STOP_TIME:
                    state = STATE_BACKUP
                    state_start = now
                print(f"STATE: {state} (stopping)")

            elif state == STATE_BACKUP:
                drive_backward(150)
                backup_time = BACKUP_TIME_DEAD_END if is_dead_end else BACKUP_TIME
                if now - state_start >= backup_time:
                    stop_robot()
                    state = STATE_TURN
                    state_start = now
                print(f"STATE: {state} (backing up)")

            elif state == STATE_TURN:
                if turn_direction == "TURN_LEFT":
                    turn_left_arc(260, 0)
                else:
                    turn_right_arc(260, 0)

                turn_elapsed = now - state_start
                front_cleared = front > FRONT_SAFE

                if front_cleared or turn_elapsed >= TURN_TIME_MAX:
                    stop_robot()

                    if front <= FRONT_SAFE and escape_count < MAX_ESCAPE_LOOPS:
                        escape_count += 1
                        turn_direction = "TURN_LEFT" if left > right else "TURN_RIGHT"
                        state = STATE_BACKUP
                    else:
                        escape_count = 0
                        is_dead_end = False
                        state = STATE_DRIVE
                    state_start = now

                print(f"Front: {front:.2f} | STATE: {state} (turning {turn_direction})")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Interrupted")

    finally:
        stop_robot()
        close_motor()
        lidar.destroy_node()
        odom.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Autonomous Drive End")


if __name__ == "__main__":
    main()

