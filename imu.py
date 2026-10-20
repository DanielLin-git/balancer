
import time
import smbus2

DEVICE_ADDRESS = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


def mpu6500_init(bus):
  bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0x00)
  time.sleep(0.1)


def read_raw_data(bus, addr):
  high = bus.read_byte_data(DEVICE_ADDRESS, addr)
  low = bus.read_byte_data(DEVICE_ADDRESS, addr + 1)
  value = (high << 8) | low
  if value > 32768:
    value -= 65536
  return value


def calibrate_gyro(bus, samples=100):
  print("正在校正陀螺儀，請保持感測器完全靜止...")
  gyro_x_offset = 0
  gyro_y_offset = 0
  gyro_z_offset = 0

  for _ in range(samples):
    gyro_x_offset += read_raw_data(bus, GYRO_XOUT_H) / 131.0
    gyro_y_offset += read_raw_data(bus, GYRO_XOUT_H + 2) / 131.0
    gyro_z_offset += read_raw_data(bus, GYRO_XOUT_H + 4) / 131.0
    time.sleep(0.01)

  # 計算平均偏差
  gyro_x_offset /= samples
  gyro_y_offset /= samples
  gyro_z_offset /= samples
  print("校正完成！")
  return gyro_x_offset, gyro_y_offset, gyro_z_offset


def main():
  bus = smbus2.SMBus(1)
  mpu6500_init(bus)

  # 執行開機校正
  g_off_x, g_off_y, g_off_z = calibrate_gyro(bus)

  print("開始讀取數值 (按 Ctrl+C 結束)...")
  try:
    while True:
      # 讀取加速度計 (±2g)
      acc_x = read_raw_data(bus, ACCEL_XOUT_H) / 16384.0
      acc_y = read_raw_data(bus, ACCEL_XOUT_H + 2) / 16384.0
      acc_z = read_raw_data(bus, ACCEL_XOUT_H + 4) / 16384.0

      # 讀取陀螺儀 (±250 dps) 並減去開機時的偏移量
      gyro_x = (read_raw_data(bus, GYRO_XOUT_H) / 131.0) - g_off_x
      gyro_y = (read_raw_data(bus, GYRO_XOUT_H + 2) / 131.0) - g_off_y
      gyro_z = (read_raw_data(bus, GYRO_XOUT_H + 4) / 131.0) - g_off_z

      print(
          f"Accel (g) -> X: {acc_x:+.2f}, Y: {acc_y:+.2f}, Z: {acc_z:+.2f} |"
          f" Gyro (°/s) -> X: {gyro_x:+.2f}, Y: {gyro_y:+.2f}, Z: {gyro_z:+.2f}"
      )

      time.sleep(0.2)

  except KeyboardInterrupt:
    print("\n程式已終止")
    bus.close()


if __name__ == "__main__":
  main()

