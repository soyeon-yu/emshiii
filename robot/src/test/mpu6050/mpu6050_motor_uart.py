import smbus2 # I2C 통신 라이브러리
import serial
import time

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
#데이터 시트 참조

bus = smbus2.SMBus(1) # I2C bus 1번 사용

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

cal_ax, cal_ay, cal_az = 0.0, 0.0, 0.0
cal_gx, cal_gy, cal_gz = 0.0, 0.0, 0.0 # 미세오차 영점보정

yaw = 0.0
last_time = 0.0

target_yaw = 0.0
base_speed = 300
Kp_yaw = 3.0

MAX_SPEED = 500
MIN_SPEED = -500

def init_mpu6050():
	bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0) # 센서 깨우기
	print("MPU6050 Initialize")

def read_raw_data(addr): # 센서 데이터 16bit, i2c 데이터 8bit 따라서 상위랑 하위 나눠 받은 뒤에 합치기
	high = bus.read_byte_data(MPU6050_ADDR, addr)
	low = bus.read_byte_data(MPU6050_ADDR, addr+1)

	value = (high << 8) | low # 16bit로 만들어줌

	if value >= 32768:
		value = value - 65536 #음수 변환
	return value

def get_scaled_data(): # 축에 따른 레지스터 주소가 2씩 차이? 상승함
	acc_x = read_raw_data(ACCEL_XOUT_H) # H - 0x3B / L - 0x3C
	acc_y = read_raw_data(ACCEL_XOUT_H + 2) # H - 0x3D / L - 0x3E
	acc_z = read_raw_data(ACCEL_XOUT_H + 4)  # H - 0x3F / L - 0x40

	gyro_x = read_raw_data(GYRO_XOUT_H) # H - 0x43 / L - 0x44
	gyro_y = read_raw_data(GYRO_XOUT_H + 2) # H - 0x45 / L - 0x46
	gyro_z = read_raw_data(GYRO_XOUT_H + 4) # H - 0x47 / L - 0x48

	ax = acc_x / 16384.0
	ay = acc_y / 16384.0
	az = acc_z / 16384.0

	gx = gyro_x / 131.0
	gy = gyro_y / 131.0
	gz = gyro_z / 131.0

	return ax, ay, az, gx, gy, gz

	# raw 값을 실제 단위로 변환 | 가속도 범위 +-2[g] => 16384 [LSB/g] | 자이로 +-250°/s =>  131 LSB/(°/s)

def calibrate_sensor(samples):
	global cal_ax, cal_ay, cal_az
	global cal_gx, cal_gy, cal_gz #전역 변수로 선언한 보정값을 함수 안에서 수정하기 위해 global을 사용
	print(f"zero point setting... don't move the sensor. ({samples} measurement)")

	sum_ax, sum_ay, sum_az = 0, 0, 0
	sum_gx, sum_gy, sum_gz = 0, 0, 0 #1000번 측정해서 평균을 내기 위한 누적 변수

	for _ in range(samples):
		ax, ay, az, gx, gy, gz = get_scaled_data()
		sum_ax += ax
		sum_ay += ay
		sum_az += az
		sum_gx += gx
		sum_gy += gy
		sum_gz += gz
		time.sleep(0.01) #10초동안 보정

	cal_ax = sum_ax / samples
	cal_ay = sum_ay / samples
	cal_az = sum_az / samples - 1.0 #Default value of gravitational acceleration

	cal_gx = sum_gx / samples
	cal_gy = sum_gy / samples
	cal_gz = sum_gz / samples
	print("zero point setting done!")

def normalize_angle(angle):
	if angle > 180:
		angle -= 360
	elif angle < -180:
		angle += 360

	return angle

def get_yaw_error(target_yaw, current_yaw):
	error = target_yaw - current_yaw

	if error > 180:
		error -= 360
	elif error < -180:
        	error += 360

	return error

def constrain(value, min_value, max_value):
	if value > max_value:
		return max_value
	elif value < min_value:
		return min_value
	else:
		return value


try:
	init_mpu6050()
	time.sleep(0.5)

	calibrate_sensor(1000)
	time.sleep(0.5)

	last_time = time.time()

	while True:
		current_time = time.time()
		dt = current_time - last_time
		last_time = current_time

		ax, ay, az, gx, gy, gz = get_scaled_data()

		# 보정값 제거
		ax -= cal_ax
		ay -= cal_ay
		az -= cal_az

		gx -= cal_gx
		gy -= cal_gy
		gz -= cal_gz

		# 자이로 노이즈 제거
		if abs(gz) < 1.2:
			gz = 0.0

		# 현재 yaw 계산
		yaw += gz * dt

		yaw = normalize_angle(yaw)

		# 목표 yaw와 현재 yaw 오차 계산
		error_yaw = get_yaw_error(target_yaw, yaw)

		# yaw 오차에 따른 보정값 계산
		correction = Kp_yaw * error_yaw

		# 좌우 모터 명령값 계산
		left_target_speed = base_speed - correction
		right_target_speed = base_speed + correction

		# 속도 제한
		left_target_speed = constrain(left_target_speed, MIN_SPEED, MAX_SPEED)
		right_target_speed = constrain(right_target_speed, MIN_SPEED, MAX_SPEED)

		#print(f"TARGET YAW : {target_yaw:+7.2f}°")
		#print(f"CURRENT YAW: {yaw:+7.2f}°")
		#print(f"YAW ERROR  : {error_yaw:+7.2f}°")
		#print(f"CORRECTION : {correction:+7.2f}")
		#print(f"LEFT CMD   : {left_cmd:+7.2f} | RIGHT CMD : {right_cmd:+7.2f}")

		# UART
		send_msg = f"{left_target_speed:.2f},{right_target_speed:.2f}\n"

		ser.write(send_msg.encode())

		response = ser.readline().decode().strip()

		if response:
			print("ESP32 :", response)
		else:
			print("check your connect")


		print("-" * 60)
		time.sleep(0.2)

except KeyboardInterrupt:
	print("\nProgram End")
