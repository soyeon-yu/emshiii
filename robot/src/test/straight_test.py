import time
import serial


# =========================================================
# UART 설정
# =========================================================

UART_PORT = "/dev/serial0"
UART_BAUDRATE = 115200
UART_TIMEOUT = 0.1


# =========================================================
# 직진 속도 설정
# =========================================================

# 기본 속도
BASE_SPEED = 200

# 현재 로봇이 오른쪽으로 휘므로 한쪽 속도를 낮춰 테스트
#
# 우선 아래 값으로 테스트:
# 왼쪽  = 200
# 오른쪽 = 170
#
# 실제 바퀴와 코드의 L/R 매핑이 반대라면
# LEFT_SPEED와 RIGHT_SPEED를 서로 바꿔서 테스트해야 함.
LEFT_SPEED = 0
RIGHT_SPEED = 200

# 명령 전송 주기
SEND_INTERVAL = 0.05  # 20 Hz


def limit_speed(speed):
    """
    속도 명령을 -255 ~ 255 범위로 제한한다.
    """
    speed = int(speed)

    if speed > 255:
        return 255

    if speed < -255:
        return -255

    return speed


def send_motor_command(uart, left_speed, right_speed):
    """
    ESP32에 왼쪽/오른쪽 모터 속도 명령을 전송한다.

    전송 형식:
        L,R\n

    예:
        200,170\n
    """
    left_speed = limit_speed(left_speed)
    right_speed = limit_speed(right_speed)

    command = "{},{}\n".format(left_speed, right_speed)
    uart.write(command.encode("utf-8"))

    print(
        "[TX] LEFT: {:4d} | RIGHT: {:4d}".format(
            left_speed,
            right_speed
        )
    )


def stop_motor(uart):
    """
    ESP32에 정지 명령을 여러 번 전송한다.
    """
    for _ in range(5):
        try:
            send_motor_command(uart, 0, 0)
            time.sleep(0.05)
        except Exception:
            pass


def main():
    uart = None

    try:
        uart = serial.Serial(
            port=UART_PORT,
            baudrate=UART_BAUDRATE,
            timeout=UART_TIMEOUT
        )

        time.sleep(1.0)

        print("========================================")
        print("직진 보정 테스트 시작")
        print("LEFT_SPEED :", LEFT_SPEED)
        print("RIGHT_SPEED:", RIGHT_SPEED)
        print("종료: Ctrl+C")
        print("========================================")

        while True:
            send_motor_command(
                uart,
                LEFT_SPEED,
                RIGHT_SPEED
            )

            # ESP32 응답이 있으면 출력
            if uart.in_waiting > 0:
                try:
                    response = uart.readline().decode(
                        "utf-8",
                        errors="ignore"
                    ).strip()

                    if response:
                        print("[RX]", response)

                except Exception as error:
                    print("[UART READ ERROR]", error)

            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("\n사용자 정지")

    except serial.SerialException as error:
        print("[UART ERROR]", error)

    except Exception as error:
        print("[ERROR]", error)

    finally:
        if uart is not None:
            stop_motor(uart)

            try:
                uart.close()
            except Exception:
                pass

        print("모터 정지 완료")


if __name__ == "__main__":
    main()

