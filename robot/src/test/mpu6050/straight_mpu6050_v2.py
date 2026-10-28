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
# 직진 제어 설정
# =========================================================
base_speed = 200

Kp_yaw = 20.0

MAX_SPEED = 350
MIN_SPEED = -350

MAX_CORRECTION = 70

yaw = 0.0
target_yaw = 0.0
cal_gz = 0.0


def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def init_mpu6050():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    print("MPU6050 init success")


def to_short(high, low):
    value = (high << 8) | low

    if value > 32767:
        value -= 65536

    return value


def read_gyro_z():
    data = bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)

    gyro_z_raw = to_short(data[12], data[13])
    gyro_z = gyro_z_raw / 131.0

    return gyro_z


def calibrate_gyro(samples=500):
    global cal_gz

    print("Calibrating gyro... Do not move robot.")

    total_gz = 0.0

    for _ in range(samples):
        gz = read_gyro_z()
        total_gz += gz
        time.sleep(0.005)

    cal_gz = total_gz / samples

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

    print("Ready.")
    print("Place the robot straight.")
    input("Press Enter to start straight control...")

    # 출발 순간 현재 방향을 목표 방향으로 설정
    target_yaw = yaw

    print("Straight control start")
    print("target_yaw:", round(target_yaw, 2))
    print("base_speed:", base_speed)
    print("Kp_yaw:", Kp_yaw)
    print("MAX_CORRECTION:", MAX_CORRECTION)

    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        if dt <= 0 or dt > 0.1:
            dt = 0.02

        gz = read_gyro_z()
        gz -= cal_gz

        # 작은 노이즈 제거
        if abs(gz) < 1.0:
            gz = 0.0

        yaw += gz * dt
        yaw = normalize_angle(yaw)

        error_yaw = target_yaw - yaw
        error_yaw = normalize_angle(error_yaw)

        correction = Kp_yaw * error_yaw
        correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION)

        left_speed = base_speed - correction
        right_speed = base_speed + correction

        left_speed = constrain(left_speed, MIN_SPEED, MAX_SPEED)
        right_speed = constrain(right_speed, MIN_SPEED, MAX_SPEED)

        send_motor(left_speed, right_speed)

        print(
            "yaw:", round(yaw, 2),
            "target:", round(target_yaw, 2),
            "err:", round(error_yaw, 2),
            "corr:", round(correction, 1),
            "L:", round(left_speed, 1),
            "R:", round(right_speed, 1)
        )

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nStop")
    stop_motor()
    ser.close()
