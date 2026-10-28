import smbus2
import time

MPU6050_ADDR = 0x68

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus2.SMBus(1)

cal_ax, cal_ay, cal_az = 0.0, 0.0, 0.0
cal_gx, cal_gy, cal_gz = 0.0, 0.0, 0.0

def init_mpu6050():
	bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
	print("MPU6050 Initialize")

def read_raw_data(addr):
	high = bus.read_byte_data(MPU6050_ADDR, addr)
	low = bus.read_byte_data(MPU6050_ADDR, addr+1)

	value = (high << 8) | low

	if value > 32768:
		value = value - 65536
	return value

def get_scaled_data():
	acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_XOUT_H + 2)
	acc_z = read_raw_data(ACCEL_XOUT_H + 4)

	gyro_x = read_raw_data(GYRO_XOUT_H)
	gyro_y = read_raw_data(GYRO_XOUT_H + 2)
	gyro_z = read_raw_data(GYRO_XOUT_H + 4)

	return (acc_x / 16384.0, acc_y / 16384.0, acc_z / 16384.0, gyro_x / 131.0, gyro_y / 131.0, gyro_z / 131.0)

def calibrate_sensor(samples):
	global cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz
	print(f"zero point setting... don't move the sensor. ({samples}measurement)")

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
		time.sleep(0.01)

	cal_ax = sum_ax / samples
	cal_ay = sum_ay / samples
	cal_az = sum_az / samples - 1.0 #Default value of gravitational acceleration

	cal_gx = sum_gx / samples
	cal_gy = sum_gy / samples
	cal_gz = sum_gz / samples
	print("zero point setting done!")

try:
	init_mpu6050()
	time.sleep(0.5)

	calibrate_sensor(100)
	time.sleep(0.5)

	while True:
		ax, ay, az, gx, gy, gz = get_scaled_data()

		ax -= cal_ax
		ay -= cal_ay
		az -= cal_az

		gx -= cal_gx
		gy -= cal_gy
		gz -= cal_gz

		print(f"---ACCEL (g) --- X : {ax:+.2f}  Y : {ay:+.2f}  Z : {az:+.2f}")
		print(f"---GYRO (O/s) --- X : {gx:+.2f}  Y : {gy:+.2f}  Z : {gz:+.2f}")
		print("-" * 50)

		time.sleep(0.5)

except KeyboardInterrupt:
	print("\nProgram End")
