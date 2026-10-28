import serial
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

try:
	print("UART start(quit Ctrl+C)")
	while True:
		send_msg = "Hello ESP32\n"
		ser.write(send_msg.encode('utf-8'))
		print(f"send msg :{send_msg.strip()}")

		response = ser.readline().decode('utf-8', errors='ignore').strip()
	      
		if response:
				print(f"receive msg :{response}")
		else:
			print("check your connect")

		print("-" * 30)
		time.sleep(2)

except KeyboardInterrupt:
				print("\ntest end")
				ser.close()
