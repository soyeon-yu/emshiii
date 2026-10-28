import time
import serial
from mpu6050 import mpu6050

UART_PORT = "/dev/serial0"
BAUDRATE = 115200

STOP_ERROR = 2.0
TIMEOUT = 10.0

TURN_SPEED_FAST = 180
TURN_SPEED_MID = 130
TURN_SPEED_SLOW = 90

ser = serial.Serial(UART_PORT, BAUDRATE, timeout=0.1)
sensor = mpu6050(0x68)

def send_motor(left_speed, right_speed):
    cmd = f"{int(left_speed)},{int(right_speed)}\n"
    ser.write(cmd.encode("utf-8"))
    ser.readline()

def stop_robot():
    send_motor(0, 0)
    time.sleep(0.2)

def get_gyro_bias():
    print("[*] Calculating gyro bias. Keep robot still.")

    bias = 0.0
    samples = 50

    for _ in range(samples):
        bias += sensor.get_gyro_data()["z"]
        time.sleep(0.01)

    bias /= samples
    print(f"[*] Gyro Z Bias: {bias:.4f}")
    return bias

def select_turn_speed(error):
    abs_error = abs(error)

    if abs_error > 35:
        return TURN_SPEED_FAST
    elif abs_error > 12:
        return TURN_SPEED_MID
    else:
        return TURN_SPEED_SLOW

def pivot_turn(target_angle):
    print("======================================")
    print(f"Target Rotation: {target_angle} deg")
    print("======================================")

    relative_yaw = 0.0
    gyro_bias = get_gyro_bias()

    last_time = time.time()
    start_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        if now - start_time > TIMEOUT:
            print("[!] Timeout. Stop.")
            stop_robot()
            break

        gyro_z = sensor.get_gyro_data()["z"] - gyro_bias

        # 이전 테스트 기준: 실제 회전 방향과 gyro_z 부호가 반대였으므로 -= 사용
        relative_yaw -= gyro_z * dt

        error = target_angle - relative_yaw

        print(
            f"Yaw: {relative_yaw:.2f} | "
            f"Target: {target_angle:.2f} | "
            f"Error: {error:.2f}"
        )

        if abs(error) <= STOP_ERROR:
            print(f"[OK] Target reached. Final yaw: {relative_yaw:.2f}")
            stop_robot()
            break

        speed = select_turn_speed(error)

        if error > 0:
            # +방향 회전
            # M1 전진, M2 후진
            send_motor(speed, -int(speed * 0.7))
        else:
            # -방향 회전
            # M1 후진, M2 전진
            send_motor(-speed, int(speed * 0.7))

        time.sleep(0.05)

if __name__ == "__main__":
    try:
        pivot_turn(-90.0)
        # 반대 방향 테스트는 아래 사용
        # pivot_turn(-90.0)

    except KeyboardInterrupt:
        print("[!] Interrupted.")

    finally:
        stop_robot()
        ser.close()
        print("Program terminated.")
