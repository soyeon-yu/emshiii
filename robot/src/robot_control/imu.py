import time
from mpu6050 import mpu6050


class MPU6050Yaw:
    def __init__(self, address=0x68):
        self.sensor = mpu6050(address)

        self.gyro_z_bias = 0.0
        self.yaw = 0.0
        self.last_time = time.monotonic()

    # -------------------------------------------------
    # 자이로 바이어스 측정
    # -------------------------------------------------
    def calibrate(self, samples=500):
        print("[IMU] Gyro Calibration...")
        print("[IMU] 로봇을 움직이지 마세요.")

        total = 0.0

        for _ in range(samples):
            gyro = self.sensor.get_gyro_data()
            total += gyro["z"]
            time.sleep(0.005)

        self.gyro_z_bias = total / samples

        self.yaw = 0.0
        self.last_time = time.monotonic()

        print(
            f"[IMU] Calibration Complete "
            f"(Bias={self.gyro_z_bias:.4f} deg/s)"
        )

    # -------------------------------------------------
    # 현재 Yaw 계산
    # -------------------------------------------------
    def update(self):

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        gyro = self.sensor.get_gyro_data()

        gyro_z = gyro["z"] - self.gyro_z_bias

        # 작은 노이즈 제거
        if abs(gyro_z) < 0.3:
            gyro_z = 0.0

        self.yaw += gyro_z * dt

        if self.yaw > 180:
            self.yaw -= 360

        elif self.yaw < -180:
            self.yaw += 360

        return self.yaw

    # -------------------------------------------------
    # 현재 방향을 0도로 설정
    # -------------------------------------------------
    def reset(self):
        self.yaw = 0.0
        self.last_time = time.monotonic()

    # -------------------------------------------------
    # 목표각과 현재각 오차
    # -------------------------------------------------
    @staticmethod
    def angle_error(target, current):

        error = target - current

        while error > 180:
            error -= 360

        while error < -180:
            error += 360

        return error

