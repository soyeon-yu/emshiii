import time
import serial
import math

# MPU6050 등 일반적인 자이로 센서 라이브러리가 있다면 상단에 임포트합니다.
# 예: from mpu6050 import mpu6050

# =========================================================
# 1. 설정 및 상수 (★ 세부 튜닝 포인트)
# =========================================================
UART_PORT = '/dev/serial0' 
BAUDRATE = 115200

STOP_ERROR = 5.0     # 멈춤 판단 오차 범위 (5도 이내로 들어오면 정지)
TIMEOUT = 15.0       # 회전 제한 시간 (초)

FAST_SPEED = 250     # 오차가 클 때 속도 (PPS 단위 기반)
MID_SPEED = 150      # 오차가 중간일 때 속도
SLOW_SPEED = 60      # 목표치 근처에서 감속할 때 속도

# -----------------------------------------------------------
# [★ 중요] 로봇 하드웨어 특성에 따른 부호 반전 설정
# -----------------------------------------------------------
# 로봇이 오른쪽으로 제자리 회전할 때, get_current_yaw() 값이 
# 줄어든다면(음수로 간다면) 이 값을 True로 바꾸세요.
INVERT_GYRO = False  

# 반대 방향 회전 시 직진하거나 한쪽만 도는 증상을 고치기 위한 모터 부호 반전
INVERT_LEFT_MOTOR = False
INVERT_RIGHT_MOTOR = False

# 시리얼 및 센서 초기화 전역 변수
ser = None
sensor = None

try:
    ser = serial.Serial(UART_PORT, BAUDRATE, timeout=0.1)
    print(f"[*] Serial Connected to {UART_PORT}")
except Exception as e:
    print(f"[!] Serial Connection Failed: {e}")

# 자이로 센서 하드웨어 연결 (예시)
try:
    # sensor = mpu6050(0x68) # 본인의 센서 초기화 코드로 대체하세요.
    print("[*] IMU Sensor Initialized Successfully.")
except Exception as e:
    print(f"[!] IMU Sensor Connection Failed: {e}")


# =========================================================
# 2. 하드웨어 연동 유틸 함수
# =========================================================
def get_current_yaw():
    """
    자이로 센서의 '실제 절대 각도(-180도 ~ 180도 또는 누적 각도)'를 리턴하는 함수
    """
    global sensor
    try:
        # -----------------------------------------------------------
        # [수정 필요] 여기에 본인의 실제 자이로 센서 각도 읽기 코드를 넣으세요!
        # 아래 코드는 예시용 더미 연산입니다. 실제 센서 객체 값을 대입해야 합니다.
        # -----------------------------------------------------------
        if sensor is not None:
            # 예시: 센서에서 가속도/자이로 데이터를 읽어 상보필터나 융합 알고리즘을 거친 Z축 각도(Yaw)
            # gyro_data = sensor.get_gyro_data()
            # raw_yaw = 정밀 계산된 Z축 각도값
            raw_yaw = 0.0 
        else:
            # 센서가 아직 연결되지 않은 테스트 상태라면 0.0 고정
            raw_yaw = 0.0 
        
        # 자이로 부호 반전 적용
        return -raw_yaw if INVERT_GYRO else raw_yaw
        
    except Exception as e:
        print(f"Gyro Read Error: {e}")
        return 0.0


def send_motor(left_speed, right_speed):
    """
    ESP32(모터 드라이버 제어기)로 좌우 목표 속도를 송신하고 Ack를 받는 함수
    """
    if ser is None or not ser.is_open:
        return
        
    # 모터 부호 반전 옵션 적용
    if INVERT_LEFT_MOTOR: left_speed = -left_speed
    if INVERT_RIGHT_MOTOR: right_speed = -right_speed
        
    cmd = f"{left_speed},{right_speed}\n"
    try:
        ser.write(cmd.encode('utf-8'))
        
        # ESP32가 보낸 응답(OK,L,R\n)을 읽어 버퍼를 비워줍니다.
        response = ser.readline() 
        # 필요한 경우 주석을 해제하여 수신 응답을 확인할 수 있습니다.
        # print(f"ESP32 ACK: {response.decode('utf-8').strip()}")
        
    except Exception as e:
        print(f"UART Tx/Rx Error: {e}")


# =========================================================
# 3. 상대 각도 회전 함수
# =========================================================
def rotate_relative(target_angle):
    """
    현재 각도를 기준으로 target_angle(도) 만큼 제자리 회전하는 함수
    (+값: 우회전, -값: 좌회전)
    """
    if ser is None:
        print("[!] Serial port not open. Aborting rotation.")
        return

    print(f"\n=========================================================")
    print(f" ▶ Relative Rotation Start: Target {target_angle}°")
    print(f"=========================================================")
    
    start_time = time.time()
    start_yaw = get_current_yaw() # 회전 시작 시점의 절대 각도 저장
    
    # 회전 방향 결정 (+: 우회전 -> 좌회전 모터 정회전/우회전 모터 역회전)
    direction = 1 if target_angle > 0 else -1
    
    while True:
        # 1. 타임아웃 검사 (안전을 위한 비상 정지장치)
        if time.time() - start_time > TIMEOUT:
            print("[!] Timeout! Emergency Stop.")
            send_motor(0, 0)
            break
            
        # 2. 현재 상대 각도 계산
        current_raw_yaw = get_current_yaw() 
        relative_yaw = current_raw_yaw - start_yaw
        
        # 3. 남은 오차 크기 계산 (오차 부호 제거)
        error = abs(target_angle - relative_yaw)
        
        print(f"Relative Yaw: {relative_yaw:.2f}° | Target: {target_angle:.2f}° | Error: {error:.2f}°")
        
        # 4. 목표 도달 및 멈춤 판단
        if error <= STOP_ERROR:
            print("[✓] Target reached! Stopping motors.")
            send_motor(0, 0) # 모터 정지 명령
            break
            
        # 5. 거리(오차)에 따른 단계별 감속 (Overshoot 방지 프로파일링)
        if error > 45.0:
            speed = FAST_SPEED
        elif error > 15.0:
            speed = MID_SPEED
        else:
            speed = SLOW_SPEED
            
        # 6. 제자리 회전(Tank Turn)을 위한 좌우 모터 차동 분배
        # 우회전시 좌모터(+), 우모터(-) / 좌회전시 좌모터(-), 우모터(+)
        left_speed = speed * direction
        right_speed = -speed * direction
        
        # ESP32로 명령 전송
        send_motor(left_speed, right_speed)
        
        # 너무 빠른 루프 반복 방지 및 통신 안정화 대기시간
        time.sleep(0.05) 

    final_relative_yaw = get_current_yaw() - start_yaw
    print(f"--- Rotation Done. Final Relative Yaw: {final_relative_yaw:.2f}° ---\n")


# =========================================================
# 4. 메인 실행 루프
# =========================================================
if __name__ == "__main__":
    try:
        print("[*] Robot Control Program Ready.")
        time.sleep(1) 
        
        # 테스트 실행: 제자리에서 딱 +90도(우회전) 회전 명령
        rotate_relative(90.0)
        
    except KeyboardInterrupt:
        print("\n[!] Program interrupted by user. Stopping motors.")
        if ser and ser.is_open:
            send_motor(0, 0)
    finally:
        if ser and ser.is_open:
            ser.close()
            print("[*] Serial Port Closed Cleanly.")
