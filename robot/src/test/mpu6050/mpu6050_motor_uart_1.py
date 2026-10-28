import smbus2 # I2C 통신 라이브러리
import serial
import time

# MPU6050 레지스터 주소 정의
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B  # 이 주소부터 14바이트를 한 번에 연속으로 읽습니다.

# I2C 버스 및 UART 포트 초기화
bus = smbus2.SMBus(1)
# 타임아웃을 짧게 잡아 ESP32 응답 지연이 제어 주기에 영향을 주지 않도록 설정
ser = serial.Serial('/dev/serial0', 115200, timeout=0.01) 

# 영점 보정 변수 초기화
cal_ax, cal_ay, cal_az = 0.0, 0.0, 0.0
cal_gx, cal_gy, cal_gz = 0.0, 0.0, 0.0

# 제어 및 각도 관련 변수
yaw = 0.0
last_time = 0.0

target_yaw = 0.0
base_speed = 300
Kp_yaw = 3.0

MAX_SPEED = 500
MIN_SPEED = -500

def init_mpu6050():
    try:
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0) # 센서 깨우기
        print("MPU6050 Initialize Success")
    except Exception as e:
        print(f"MPU6050 Initialize Failed: {e}")

def get_scaled_data():
    try:
        # 0x3B 레지스터부터 가속도, 온도, 자이로 데이터 총 14바이트를 버스트 읽기
        data = bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
        
        # 2개의 8비트 데이터를 하나의 16비트 정수로 결합 및 음수 처리
        def to_short(high, low):
            val = (high << 8) | low
            return val - 65536 if val > 32768 else val

        acc_x = to_short(data[0], data[1])
        acc_y = to_short(data[2], data[3])
        acc_z = to_short(data[4], data[5])
        # data[6], data[7]은 온도 데이터 (사용 안 함)
        gyro_x = to_short(data[8], data[9])
        gyro_y = to_short(data[10], data[11])
        gyro_z = to_short(data[12], data[13])

        # 스케일 팩터 적용 (가속도 +-2g -> 16384, 자이로 +-250 deg/s -> 131)
        ax = acc_x / 16384.0
        ay = acc_y / 16384.0
        az = acc_z / 16384.0

        gx = gyro_x / 131.0
        gy = gyro_y / 131.0
        gz = gyro_z / 131.0

        return ax, ay, az, gx, gy, gz

    except Exception as e:
        # 순간적인 I2C 통신 누락 시 프로그램 정지를 막기 위한 예외 처리
        # 정지 상태에 가까운 더미 데이터(Z축 중력 1.0)를 반환합니다.
        return 0.0, 0.0, 1.0, 0.0, 0.0, 0.0

def calibrate_sensor(samples):
    global cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz
    print(f"Zero point setting... DON'T MOVE the sensor. ({samples} measurements)")

    sum_ax, sum_ay, sum_az = 0, 0, 0
    sum_gx, sum_gy, sum_gz = 0, 0, 0

    for _ in range(samples):
        ax, ay, az, gx, gy, gz = get_scaled_data()
        sum_ax += ax
        sum_ay += ay
        sum_az += az
        sum_gx += gx
        sum_gy += gy
        sum_gz += gz
        time.sleep(0.005)

    cal_ax = sum_ax / samples
    cal_ay = sum_ay / samples
    cal_az = (sum_az / samples) - 1.0 # 정지 상태 기본 중력가속도 제거

    cal_gx = sum_gx / samples
    cal_gy = sum_gy / samples
    cal_gz = sum_gz / samples
    print("Zero point setting done!")

def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

def get_yaw_error(target_yaw, current_yaw):
    error = target_yaw - current_yaw
    return normalize_angle(error)

def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)

try:
    init_mpu6050()
    time.sleep(0.5)

    calibrate_sensor(500) # 500번 측정으로 최적화 (안정적)
    time.sleep(0.5)

    last_time = time.time()

    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # 리눅스 스케줄링 등으로 dt가 비정상적으로 튀는 현상 방지
        if dt <= 0 or dt > 0.1:
            dt = 0.02

        ax, ay, az, gx, gy, gz = get_scaled_data()

        # 오프셋 보정값 제거
        gz -= cal_gz

        # 자이로 데드밴드(노이즈 필터링) 설정
        if abs(gz) < 1.5:
            gz = 0.0

        # 현재 yaw 적분 계산
        yaw += gz * dt
        yaw = normalize_angle(yaw)

        # 목표 yaw와의 오차 및 P 제어 보정량 계산
        error_yaw = get_yaw_error(target_yaw, yaw)
        correction = Kp_yaw * error_yaw

        # 좌우 모터 명령값 계산
        left_target_speed = base_speed - correction
        right_target_speed = base_speed + correction

        # 속도 최소/최대값 제한
        left_target_speed = constrain(left_target_speed, MIN_SPEED, MAX_SPEED)
        right_target_speed = constrain(right_target_speed, MIN_SPEED, MAX_SPEED)

        # UART 전송 문자열 생성 및 송신
        send_msg = f"{left_target_speed:.2f},{right_target_speed:.2f}\n"
        ser.write(send_msg.encode())

        # ESP32 수신부 (수신 버퍼에 데이터가 있을 때만 논블로킹으로 처리)
        if ser.in_waiting > 0:
            try:
                # errors='ignore'를 추가하여 노이즈로 인한 디코딩 에러 방지
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print("ESP32 :", response)
            except Exception as uart_err:
                # 수신 과정에서 에러가 발생해도 전체 제어 루프는 계속 실행되도록 유지
                pass

        # 제어 주기 조절 (약 50Hz)
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nProgram End")
