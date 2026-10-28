# motor.py

import time
import threading
import serial


# =========================================================
# UART 설정
# =========================================================

UART_PORT = "/dev/serial0"
BAUDRATE = 115200

UART_DEBUG = True

# OK 응답을 매번 출력하면 로그가 너무 많으므로 기본 비활성화
UART_OK_DEBUG = False

# SENSOR 수신 로그
SENSOR_DEBUG = True


# =========================================================
# 좌우 속도 보정
# =========================================================

LEFT_TRIM = 0.98
RIGHT_TRIM = 1.00


# =========================================================
# UART 초기화
# =========================================================

ser = serial.Serial(
    port=UART_PORT,
    baudrate=BAUDRATE,
    timeout=0.05,
    write_timeout=0.1
)

# UART 쓰기 보호
_uart_write_lock = threading.Lock()

# 수신값 보호
_data_lock = threading.Lock()

# 수신 스레드 실행 상태
_uart_running = True
_uart_thread = None


# =========================================================
# 마지막 수신값 및 명령값
# =========================================================

_last_m1_speed = 0.0
_last_m2_speed = 0.0

_last_left_cmd = 0
_last_right_cmd = 0

_last_received_left_cmd = 0
_last_received_right_cmd = 0

_last_response_time = 0.0

_last_sensor_distance = None
_last_sensor_time = 0.0


# =========================================================
# UART 응답 파싱
# =========================================================

def _parse_ok_response(line):
    """
    ESP32 응답:
        OK,left_cmd,right_cmd,m1_speed,m2_speed
    """

    global _last_m1_speed
    global _last_m2_speed
    global _last_received_left_cmd
    global _last_received_right_cmd
    global _last_response_time

    parts = line.split(",")

    if len(parts) != 5:
        if UART_DEBUG:
            print(
                "\n[UART] 잘못된 OK 응답 형식: "
                f"{line!r}"
            )
        return

    try:
        received_left_cmd = int(float(parts[1]))
        received_right_cmd = int(float(parts[2]))

        m1_speed = float(parts[3])
        m2_speed = float(parts[4])

    except ValueError:
        if UART_DEBUG:
            print(
                "\n[UART] OK 응답 숫자 변환 실패: "
                f"{line!r}"
            )
        return

    with _data_lock:
        _last_received_left_cmd = received_left_cmd
        _last_received_right_cmd = received_right_cmd

        _last_m1_speed = m1_speed
        _last_m2_speed = m2_speed

        _last_response_time = time.monotonic()

    if UART_OK_DEBUG:
        print(
            f"\r[UART OK] "
            f"RX CMD L={received_left_cmd:5d}, "
            f"R={received_right_cmd:5d} | "
            f"M1={m1_speed:8.1f} pps, "
            f"M2={m2_speed:8.1f} pps",
            end="",
            flush=True
        )


def _parse_sensor_response(line):
    """
    ESP32 센서 응답:
        SENSOR,distance_cm

    예:
        SENSOR,45.20
        SENSOR,NA
    """

    global _last_sensor_distance
    global _last_sensor_time

    parts = line.split(",")

    if len(parts) != 2:
        if UART_DEBUG:
            print(
                "\n[SENSOR] 잘못된 응답 형식: "
                f"{line!r}"
            )
        return

    distance_text = parts[1].strip()

    try:
        if distance_text.upper() == "NA":
            distance_cm = None
        else:
            distance_cm = float(distance_text)

    except ValueError:
        if UART_DEBUG:
            print(
                "\n[SENSOR] 거리값 변환 실패: "
                f"{line!r}"
            )
        return

    with _data_lock:
        _last_sensor_distance = distance_cm
        _last_sensor_time = time.monotonic()

    if SENSOR_DEBUG:
        print(
            f"\n[SENSOR] distance={distance_cm}"
        )


# =========================================================
# UART 수신 전용 스레드
# =========================================================

def _uart_receive_loop():
    """
    ESP32에서 오는 모든 UART 메시지를 계속 읽는다.

    OK,...     → 엔코더/PPS 저장
    SENSOR,... → 초음파 거리 저장
    """

    while _uart_running:
        try:
            raw = ser.readline()

            if not raw:
                continue

            line = raw.decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if not line:
                continue

            if line.startswith("OK,"):
                _parse_ok_response(line)
                continue

            if line.startswith("SENSOR,"):
                _parse_sensor_response(line)
                continue

            if line == "SENSOR_STARTED":
                if UART_DEBUG:
                    print("\n[SENSOR] started")
                continue

            if line == "SENSOR_STOPPED":
                if UART_DEBUG:
                    print("\n[SENSOR] stopped")
                continue

            if UART_DEBUG:
                print(
                    f"\n[ESP32] {line}"
                )

        except serial.SerialException as error:
            if _uart_running:
                print(
                    f"\n[UART RX SERIAL ERROR] {error}"
                )
                time.sleep(0.1)

        except Exception as error:
            if _uart_running:
                print(
                    f"\n[UART RX ERROR] {error}"
                )
                time.sleep(0.05)


def _start_uart_receiver():
    global _uart_thread

    _uart_thread = threading.Thread(
        target=_uart_receive_loop,
        daemon=True,
        name="esp32_uart_receiver"
    )

    _uart_thread.start()


# =========================================================
# 모터 명령 전송
# =========================================================

# =========================================================
# 센서 시작/정지 명령 전송
# =========================================================

def send_sensor_start():
    """
    ESP32 센서 측정을 시작한다.
    """

    try:
        with _uart_write_lock:
            ser.write(b"START\n")
            ser.flush()

        if UART_DEBUG:
            print(
                "\n[SENSOR COMMAND] START 전송"
            )

        return True

    except serial.SerialTimeoutException as error:
        print(
            f"\n[SENSOR START WRITE TIMEOUT] {error}"
        )
        return False

    except serial.SerialException as error:
        print(
            f"\n[SENSOR START SERIAL ERROR] {error}"
        )
        return False

    except Exception as error:
        print(
            f"\n[SENSOR START ERROR] {error}"
        )
        return False


def send_sensor_stop():
    """
    ESP32 센서 측정을 정지한다.
    """

    try:
        with _uart_write_lock:
            ser.write(b"STOP\n")
            ser.flush()

        if UART_DEBUG:
            print(
                "\n[SENSOR COMMAND] STOP 전송"
            )

        return True

    except serial.SerialTimeoutException as error:
        print(
            f"\n[SENSOR STOP WRITE TIMEOUT] {error}"
        )
        return False

    except serial.SerialException as error:
        print(
            f"\n[SENSOR STOP SERIAL ERROR] {error}"
        )
        return False

    except Exception as error:
        print(
            f"\n[SENSOR STOP ERROR] {error}"
        )
        return False


def send_motor(left_speed, right_speed):
    """
    ESP32에 좌우 모터 명령만 전송한다.

    UART 응답은 별도의 수신 스레드가 처리한다.

    반환:
        True  : UART 쓰기 성공
        False : UART 쓰기 실패
    """

    global _last_left_cmd
    global _last_right_cmd

    left_speed = int(round(left_speed))
    right_speed = int(round(right_speed))

    # 두 바퀴가 같은 방향일 때만 직진 trim 적용
    if left_speed * right_speed > 0:
        left_speed = int(left_speed * LEFT_TRIM)
        right_speed = int(right_speed * RIGHT_TRIM)

    command = f"{left_speed},{right_speed}\n"

    try:
        with _uart_write_lock:
            ser.write(
                command.encode("utf-8")
            )
            ser.flush()

        with _data_lock:
            _last_left_cmd = left_speed
            _last_right_cmd = right_speed

        return True

    except serial.SerialTimeoutException as error:
        print(
            f"\n[UART WRITE TIMEOUT] {error}"
        )
        return False

    except serial.SerialException as error:
        print(
            f"\n[UART SERIAL ERROR] {error}"
        )
        return False

    except Exception as error:
        print(
            f"\n[UART WRITE ERROR] {error}"
        )
        return False


# =========================================================
# 엔코더 데이터 조회
# =========================================================

def get_encoder_speed():
    """
    ESP32 기준 M1, M2 PPS 반환.
    """

    with _data_lock:
        return (
            _last_m1_speed,
            _last_m2_speed
        )


def get_signed_encoder_speed():
    """
    실제 바퀴 기준 signed PPS 반환.

    M2 = 실제 왼쪽 바퀴
    M1 = 실제 오른쪽 바퀴
    """

    with _data_lock:
        return (
            _last_m2_speed,
            _last_m1_speed
        )


def get_last_command():
    """
    Raspberry Pi가 마지막으로 전송한 명령값 반환.
    """

    with _data_lock:
        return (
            _last_left_cmd,
            _last_right_cmd
        )


def get_last_received_command():
    """
    ESP32가 마지막으로 보고한 명령값 반환.
    """

    with _data_lock:
        return (
            _last_received_left_cmd,
            _last_received_right_cmd
        )


def get_last_response_age():
    """
    마지막 정상 OK 응답 이후 경과 시간.
    """

    with _data_lock:
        response_time = _last_response_time

    if response_time <= 0.0:
        return float("inf")

    return (
        time.monotonic()
        - response_time
    )


# =========================================================
# 센서 데이터 조회
# =========================================================

def get_sensor_data():
    """
    마지막 초음파 센서값 반환.

    반환:
        distance_cm : 거리(cm), 측정 실패 시 None
        age_sec     : 마지막 SENSOR 수신 이후 경과 시간
    """

    with _data_lock:
        distance_cm = _last_sensor_distance
        sensor_time = _last_sensor_time

    if sensor_time <= 0.0:
        return None, float("inf")

    age_sec = (
        time.monotonic()
        - sensor_time
    )

    return (
        distance_cm,
        age_sec
    )


# =========================================================
# 마지막 명령 재전송
# =========================================================

def resend_last_command():
    with _data_lock:
        left_cmd = _last_left_cmd
        right_cmd = _last_right_cmd

    return send_motor(
        left_cmd,
        right_cmd
    )


# =========================================================
# 기본 주행 함수
# =========================================================

def stop_robot():
    return send_motor(0, 0)


def drive_forward(speed=220):
    return send_motor(
        speed,
        speed
    )


def drive_backward(speed=180):
    return send_motor(
        -speed,
        -speed
    )


def turn_left_arc(outer=360, inner=0):
    return send_motor(
        inner,
        outer
    )


def turn_right_arc(outer=360, inner=0):
    return send_motor(
        outer,
        inner
    )


# =========================================================
# UART 종료
# =========================================================

def close_motor():
    global _uart_running

    try:
        if ser.is_open:
            stop_robot()
            time.sleep(0.05)

        _uart_running = False

        if (
            _uart_thread is not None
            and _uart_thread.is_alive()
        ):
            _uart_thread.join(
                timeout=0.3
            )

        if ser.is_open:
            ser.close()

    except Exception as error:
        print(
            f"\n[MOTOR CLOSE ERROR] {error}"
        )


# 모듈이 import되는 즉시 UART 수신 스레드 시작
_start_uart_receiver()
