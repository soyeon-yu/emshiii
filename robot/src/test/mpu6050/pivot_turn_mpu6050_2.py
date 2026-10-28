import time
import serial
from mpu6050 import mpu6050

# =========================================================
# 1. 설정 및 상수
# =========================================================
UART_PORT = '/dev/serial0' 
BAUDRATE = 115200

STOP_ERROR = 2.0     # 멈춤 판단 오차 범위 (2도 이내로 들어오면 정지)
TIMEOUT = 15.0       # 회전 제한 시간 (초)

# 피봇 턴 속도 (ESP32 속도 제어기 기준에 맞게 조정하세요)
# 이전 로그에서 사용하던 속도 프로파일(FAST/MID/SLOW) 대신 단순 고정 속도로 먼저 테스트합니다.
TURN_SPEED = 150     

# 시리얼 및 센서 객체 변수
ser = None
sensor = None

try:
    ser = serial.Serial(UART_PORT, BAUDRATE, timeout=0.1)
    print(f"[*] Serial Connected to {UART_PORT}")
except Exception as e:
    print(f"[!] Serial Connection Failed: {e}")
    exit()

try:
    sensor = mpu6050(0x68)
    print("[*] IMU Sensor Initialized Successfully.")
except Exception as e:
    print(f"[!] IMU Sensor Initialization Failed: {e}")
    exit()

print("[*] Robot Control Program Ready.")


# =========================================================
# 2. 하드웨어 연동 함수 (기존 ESP32 프로토콜 반영)
# =========================================================
def send_motor(left_speed, right_speed):
    """
    기존에 사용하시던 ESP32 수신 포맷 '좌측속도,우측속도\n' 데이터 송신 함수
    """
    if ser is None or not ser.is_open:
        return
        
    # 기존 규칙대로 포맷 생성 ("250,-250\n" 형태)
    cmd = f"{left_speed},{right_speed}\n"
    try:
        ser.write(cmd.encode('utf-8'))
        
        # ESP32의 응답(Ack)을 읽어서 버퍼 비워주기
        response = ser.readline() 
    except Exception as e:
        print(f"UART Tx/Rx Error: {e}")

def stop_robot():
    send_motor(0, 0)


# =========================================================
# 3. 제자리 회전 제어 루프 (부호 반전 및 에러 수정 반영)
# =========================================================
def pivot_turn(target_angle):
    print("\n=========================================================")
    print(f" ▶ Relative Rotation Start: Target {target_angle}°")
    print("=========================================================")

    relative_yaw = 0.0
    last_time = time.time()
    start_time = time.time()
    
    # MPU6050 센서 제로점(오프셋) 잡기
    print("[*] Calculating gyro bias... Keep the robot still.")
    gyro_bias = 0.0
    for _ in range(30):
        gyro_bias += sensor.get_gyro_data()['z']
        time.sleep(0.01)
    gyro_bias /= 30.0
    print(f"[*] Gyro Z Bias: {gyro_bias:.4f}")

    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # 1. 타임아웃 검사 (비상 정지)
        if (current_time - start_time) > TIMEOUT:
            print("[!] Timeout! Emergency Stop.")
            stop_robot()
            break

        # 2. 센서 읽기 및 누적 각도(Yaw) 계산
        gyro_z = sensor.get_gyro_data()['z'] - gyro_bias
        
        # ★ 이전 로그 분석 결과, 로봇이 돌 때 센서가 마이너스로 갔으므로 
        #    각도 누적 시 마이너스(-)를 곱하여 방향을 강제로 정방향 매칭합니다.
        relative_yaw -= gyro_z * dt

        # 3. 오차 계산
        error = target_angle - relative_yaw
        
        print(f"Relative Yaw: {relative_yaw:.2f}° | Target: {target_angle:.2f}° | Error: {error:.2f}°")

        # 4. 목표 도달 판정 (오차가 2도 이내면 정지)
        if abs(error) <= STOP_ERROR:
            print(f"[✓] Target reached! Final Relative Yaw: {relative_yaw:.2f}°")
            stop_robot()
            break
            
        # 5. 오차 방향에 따른 모터 제어 (제자리 회전)
        if error > 0:
            # 정방향 회전 (왼쪽은 전진, 오른쪽은 후진) -> ESP32로 전송
            send_motor(TURN_SPEED, -TURN_SPEED)
        else:
            # 역방향 회전 (왼쪽은 후진, 오른쪽은 전진) -> ESP32로 전송
            send_motor(-TURN_SPEED, TURN_SPEED)
            
        time.sleep(0.05)


# =========================================================
# 4. 메인 실행
# =========================================================
if __name__ == "__main__":
    try:
        # 테스트: +90도 회전
        pivot_turn(-90.0)
    except KeyboardInterrupt:
        print("\n[!] Program interrupted by user.")
    finally:
        stop_robot()
        ser.close()
        print("--- Program Terminated ---")
