import serial
import time

# =========================================================
# UART 설정
# =========================================================
ser = serial.Serial('/dev/serial0', 115200, timeout=0.01)


def send_motor(left_speed, right_speed):
    msg = "{},{}\n".format(int(left_speed), int(right_speed))
    ser.write(msg.encode("utf-8"))


def stop_motor():
    for _ in range(10):
        ser.write(b"0,0\n")
        time.sleep(0.03)


def test_motion(name, left_speed, right_speed, duration=1.0):
    print()
    print("=================================")
    print(name)
    print("L:", left_speed, "R:", right_speed)
    print("로봇 움직임을 눈으로 확인하세요.")
    print("=================================")

    input("Enter 누르면 시작")

    start = time.time()

    while time.time() - start < duration:
        send_motor(left_speed, right_speed)
        time.sleep(0.05)

    stop_motor()

    input("움직임을 기록한 뒤 Enter")


try:
    print("Motor direction mapping test start")
    print("각 테스트마다 로봇이 어떻게 움직이는지 기록해줘.")
    print("예: 왼쪽 바퀴 전진 / 오른쪽 바퀴 전진 / 오른쪽으로 회전 / 직진 등")

    # A: 왼쪽 명령 +만 줌
    test_motion("A TEST: send_motor(150, 0)", 150, 0)

    # B: 오른쪽 명령 +만 줌
    test_motion("B TEST: send_motor(0, 150)", 0, 150)

    # C: 왼쪽 명령 -만 줌
    test_motion("C TEST: send_motor(-150, 0)", -150, 0)

    # D: 오른쪽 명령 -만 줌
    test_motion("D TEST: send_motor(0, -150)", 0, -150)

    # E: 양쪽 + 명령
    test_motion("E TEST: send_motor(150, 150)", 150, 150)

    # F: 양쪽 - 명령
    test_motion("F TEST: send_motor(-150, -150)", -150, -150)

    stop_motor()

    print()
    print("Test done")
    print("A~F 결과를 알려줘.")

except KeyboardInterrupt:
    print("\nKeyboardInterrupt Stop")

finally:
    stop_motor()

    try:
        ser.close()
    except:
        pass

    print("Program End")
