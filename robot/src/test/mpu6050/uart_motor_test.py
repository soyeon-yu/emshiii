import serial
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=0.05)

left_speed = 200
right_speed = 200

print("Raspberry Pi UART Motor Test Start")

try:
    while True:
        msg = "{},{}\n".format(left_speed, right_speed)
        ser.write(msg.encode("utf-8"))

        print("SEND:", msg.strip())

        if ser.in_waiting > 0:
            response = ser.readline().decode("utf-8", errors="ignore").strip()
            if response:
                print("ESP32:", response)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nSTOP")

    for _ in range(10):
        ser.write(b"0,0\n")
        time.sleep(0.05)

    ser.close()
