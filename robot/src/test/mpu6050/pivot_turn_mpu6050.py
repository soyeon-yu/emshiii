import smbus2
import serial
import time

# =========================================================
# MPU6050 설정
# =========================================================
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# =========================================================
# UART 설정
# =========================================================
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200

# =========================================================
# 회전 설정
# =========================================================
TARGET_ANGLE = 90.0

STOP_ERROR = 9.0
TIMEOUT = 15.0

FAST_SPEED = 250
MID_SPEED = 150
SLOW_SPEED = 60

yaw = 0.0
cal_gz = 0.0

bus = None
ser = None


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def to_short(high, low):
    value = (high << 8) | low

    if value > 32767:
        value -= 65536

    return value


def open_i2c():
    global bus

    for i in range(5):
        try:
            bus = smbus2.SMBus(1)
            time.sleep(0.2)
            print("I2C bus open success")
            return
        except OSError as e:
            print("I2C open failed retry", i + 1, e)
            time.sleep(0.5)

    raise RuntimeError("I2C bus open failed")


def init_mpu6050():
    global bus

    for i in range(10):
        try:
            bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
            time.sleep(0.2)

            # 읽기 테스트까지 확인
            bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)

            print("MPU6050 init success")
            return

        except OSError as e:
            print("MPU6050 init failed retry", i + 1, e)

            try:
                bus.close()
            except:
                pass

            time.sleep(0.5)

            try:
                bus = smbus2.SMBus(1)
                time.sleep(0.2)
            except:
                pass

    raise RuntimeError("MPU6050 init failed after retries")


def read_mpu_block():
    global bus

    for i in range(5):
        try:
            return bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)

        except OSError as e:
            print("MPU6050 read failed retry", i + 1, e)

            try:
                bus.close()
            except:
                pass

            time.sleep(0.2)

            try:
                bus = smbus2.SMBus(1)
                time.sleep(0.1)
            except:
                pass

    return None


def read_gyro_z():
    data = read_mpu_block()

    if data is None:
        return None

    gyro_z_raw = to_short(data[12], data[13])
    gyro_z = gyro_z_raw / 131.0

    return gyro_z


def calibrate_gyro(samples=500):
    global cal_gz

    print("Calibrating... Do not move robot.")

    total = 0.0
    valid_count = 0

    while valid_count < samples:
        gz = read_gyro_z()

        if gz is not None:
            total += gz
            valid_count += 1

        time.sleep(0.005)

    cal_gz = total / valid_count

    print("Calibration done")
    print("cal_gz:", cal_gz)


def open_serial():
    global ser

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    time.sleep(0.5)

    print("Serial open success")


def send_motor(left_speed, right_speed):
    global ser

    msg = "{},{}\n".format(int(left_speed), int(right_speed))

    try:
        ser.write(msg.encode("utf-8"))
    except Exception as e:
        print("UART send failed:", e)


def stop_motor():
    for _ in range(10):
        send_motor(0, 0)
        time.sleep(0.03)


def pivot_turn(target_angle):
    global yaw

    yaw = 0.0
    target_yaw = normalize_angle(yaw + target_angle)

    print("Pivot turn start")
    print("start yaw:", round(yaw, 2))
    print("target yaw:", round(target_yaw, 2))

    last_time = time.time()
    start_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        if dt <= 0 or dt > 0.1:
            dt = 0.02

        gz = read_gyro_z()

        if gz is None:
            stop_motor()
            print("MPU read failed during turn")
            continue

        gz = gz - cal_gz

        if abs(gz) < 1.0:
            gz = 0.0

        # 현재 테스트 기준: yaw가 반대로 가면 + / -만 바꾸면 됨
        yaw -= gz * dt
        yaw = normalize_angle(yaw)

        error = normalize_angle(target_yaw - yaw)
        abs_error = abs(error)

        if abs_error <= STOP_ERROR:
            print("Target reached")
            break

        if time.time() - start_time > TIMEOUT:
            print("TIMEOUT: turn failed")
            break

        if abs_error > 45:
            turn_speed = FAST_SPEED
        elif abs_error > 15:
            turn_speed = MID_SPEED
        else:
            turn_speed = SLOW_SPEED

        # 순수 Pivot Turn
        if target_angle > 0:
            left_speed = turn_speed
            right_speed = -turn_speed
        else:
            left_speed = -turn_speed
            right_speed = turn_speed

        send_motor(left_speed, right_speed)

        print(
            "yaw:", round(yaw, 2),
            "target:", round(target_yaw, 2),
            "err:", round(error, 2),
            "gz:", round(gz, 2),
            "L:", left_speed,
            "R:", right_speed
        )

        time.sleep(0.02)

    stop_motor()

    print("Pivot turn done")
    print("final yaw:", round(yaw, 2))
    print("target yaw:", round(target_yaw, 2))
    print("final error:", round(normalize_angle(target_yaw - yaw), 2))


try:
    open_i2c()
    init_mpu6050()

    open_serial()

    calibrate_gyro(500)
    time.sleep(0.5)

    input("Place robot straight. Press Enter to start pivot turn...")

    pivot_turn(TARGET_ANGLE)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt Stop")

except Exception as e:
    print("ERROR:", e)

finally:
    stop_motor()

    try:
        ser.close()
    except:
        pass

    try:
        bus.close()
    except:
        pass

    print("Program End")
