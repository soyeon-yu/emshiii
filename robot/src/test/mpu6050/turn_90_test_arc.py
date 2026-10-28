import time
import serial
from mpu6050 import mpu6050

UART_PORT = "/dev/serial0"
BAUDRATE = 115200

STOP_ERROR = 2.0
TIMEOUT = 15.0

# 아크턴 속도
FAST_OUTER = 180
FAST_INNER = 80

MID_OUTER = 140
MID_INNER = 60

SLOW_OUTER = 100
SLOW_INNER = 40

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

def select_arc_speed(error):
    abs_error = abs(error)

    if abs_error > 35:
        return 360, 0
    elif abs_error > 12:
        return 270, 0
    else:
        return 220, 0

def arc_turn(target_angle):
    print("======================================")
    print(f"Arc Turn Target: {target_angle} deg")
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

        # 이전 로그 기준: target -90에서 yaw가 +로 갔으므로 += 사용
        relative_yaw += gyro_z * dt

        error = target_angle - relative_yaw

        print(
            f"Yaw: {relative_yaw:.2f} | "
            f"Target: {target_angle:.2f} | "
            f"Error: {error:.2f}"
        )

        if abs(error) <= STOP_ERROR:
            print(f"[OK] Arc turn complete. Final yaw: {relative_yaw:.2f}")
            stop_robot()
            break

        outer_speed, inner_speed = select_arc_speed(error)

        if error > 0:
            # +방향 회전
            # 왼쪽 바퀴가 더 빠름, 오른쪽 바퀴가 느림
            send_motor(inner_speed, outer_speed)
        else:
            # -방향 회전
            # 오른쪽 바퀴가 더 빠름, 왼쪽 바퀴가 느림
            send_motor(outer_speed, inner_speed)

        time.sleep(0.05)

if __name__ == "__main__":
    try:
        arc_turn(-90.0)
        # 반대 방향 테스트:
        # arc_turn(-90.0)

    except KeyboardInterrupt:
        print("[!] Interrupted.")

    finally:
        stop_robot()
        ser.close()
        print("Program terminated.")
