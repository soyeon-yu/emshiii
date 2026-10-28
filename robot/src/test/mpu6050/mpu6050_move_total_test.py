import time
import serial

UART_PORT = "/dev/serial0"
BAUDRATE = 115200

ser = serial.Serial(UART_PORT, BAUDRATE, timeout=0.1)

# ==============================
# 기본 UART 송신
# ==============================
def send_motor(left_speed, right_speed):
    cmd = f"{int(left_speed)},{int(right_speed)}\n"
    ser.write(cmd.encode("utf-8"))

    # ESP32 응답 버퍼 비우기
    try:
        ser.readline()
    except:
        pass

# ==============================
# 주행 함수
# ==============================
def stop_robot():
    send_motor(0, 0)

def drive_forward(speed=220):
    send_motor(speed, speed)

def drive_backward(speed=180):
    send_motor(-speed, -speed)

def turn_right_arc(outer=360, inner=0):
    # 오른쪽 바퀴 빠르게, 왼쪽 바퀴 느리게/정지
    send_motor(inner, outer)

def turn_left_arc(outer=360, inner=0):
    # 왼쪽 바퀴 빠르게, 오른쪽 바퀴 느리게/정지
    send_motor(outer, inner)

# ==============================
# 테스트 루틴
# ==============================
def test_drive_commands():
    print("=== Drive Command Test Start ===")

    print("1. Forward")
    drive_forward(220)
    time.sleep(2)
    stop_robot()
    time.sleep(1)

    print("2. Backward")
    drive_backward(180)
    time.sleep(2)
    stop_robot()
    time.sleep(1)

    print("3. Left Arc Turn")
    turn_left_arc(360, 0)
    time.sleep(2)
    stop_robot()
    time.sleep(1)

    print("4. Right Arc Turn")
    turn_right_arc(360, 0)
    time.sleep(2)
    stop_robot()
    time.sleep(1)

    print("=== Test Done ===")
    stop_robot()

if __name__ == "__main__":
    try:
        test_drive_commands()

    except KeyboardInterrupt:
        print("Interrupted")

    finally:
        stop_robot()
        ser.close()
        print("Serial Closed")


