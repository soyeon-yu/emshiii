import smbus2
import serial
import time

# =========================================================
# MPU6050 설정
# =========================================================
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

bus = smbus2.SMBus(1)

# =========================================================
# UART 설정
# =========================================================
ser = serial.Serial('/dev/serial0', 115200, timeout=0.01)

# =========================================================
# 회전 설정
# =========================================================
TARGET_ANGLE = 90.0     # +90: 한 방향, -90: 반대 방향
STOP_ERROR = 3.0        # 목표 각도 ±3도 안에 들어오면 정지

FAST_SPEED = 120
MID_SPEED = 80
SLOW_SPEED = 50

yaw = 0.0
cal_gz = 0.0


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def to_short(high, low):
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value


def init_mpu6050():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    print("MPU6050 init success")


def read_gyro_z():
    data = bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
    gyro_z_raw = to_short(data[12], data[13])
    return gyro_z_raw / 131.0


def calibrate_gyro(samples=500):
    global cal_gz

    print("Calibrating... Do not move robot.")

    total = 0.0

    for _ in range(samples):
        total += read_gyro_z()
        time.sleep(0.005)

    cal_gz = total / samples

    print("Calibration done")
    print("cal_gz:", cal_gz)


def send_motor(left_speed, right_speed):
    msg = "{:.1f},{:.1f}\n".format(left_speed, right_speed)
    ser.write(msg.encode("utf-8"))


def stop_motor():
    for _ in range(10):
        ser.write(b"0,0\n")
        time.sleep(0.05)


try:
    init_mpu6050()
    time.sleep(0.5)

    calibrate_gyro(500)
    time.sleep(0.5)

    input("Place robot straight. Press Enter to start pivot turn...")

    target_yaw = normalize_angle(yaw + TARGET_ANGLE)

    print("Pivot turn start")
    print("start yaw:", round(yaw, 2))
    print("target yaw:", round(target_yaw, 2))

    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        if dt <= 0 or dt > 0.1:
            dt = 0.02

        gz = read_gyro_z() - cal_gz

        if abs(gz) < 1.0:
            gz = 0.0

        yaw += gz * dt
        yaw = normalize_angle(yaw)

        error = normalize_angle(target_yaw - yaw)
        abs_error = abs(error)

        if abs_error <= STOP_ERROR:
            break

        # 목표에 가까워질수록 감속
        if abs_error > 45:
            turn_speed = FAST_SPEED
        elif abs_error > 15:
            turn_speed = MID_SPEED
        else:
            turn_speed = SLOW_SPEED

        # =================================================
        # 피벗 회전 방식
        # +90 방향: 왼쪽 정지, 오른쪽 전진
        # -90 방향: 왼쪽 전진, 오른쪽 정지
        # =================================================
        if error > 0:
            left_speed = 0
            right_speed = turn_speed
        else:
            left_speed = turn_speed
            right_speed = 0

        send_motor(left_speed, right_speed)

        print(
            "yaw:", round(yaw, 2),
            "target:", round(target_yaw, 2),
            "err:", round(error, 2),
            "speed:", turn_speed,
            "L:", left_speed,
            "R:", right_speed
        )

        time.sleep(0.02)

    stop_motor()

    print("Pivot turn done")
    print("final yaw:", round(yaw, 2))
    print("target yaw:", round(target_yaw, 2))

except KeyboardInterrupt:
    print("\nStop")
    stop_motor()
    ser.close()
