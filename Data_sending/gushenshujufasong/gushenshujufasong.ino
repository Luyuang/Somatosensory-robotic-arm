#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;
int16_t dx_raw, dy_raw, dz_raw; // 当前位移原始数据
int16_t prev_dx = 0, prev_dy = 0, prev_dz = 0; // 上一次的位移数据
float delta_dx, delta_dy, delta_dz; // 转换后的位移变化量

int16_t gx_raw, gy_raw, gz_raw; // 陀螺仪角速度原始数据
float gx_deg_per_sec, gy_deg_per_sec, gz_deg_per_sec; // 转换后的角速度

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  // 读取当前位移数据
  mpu.getAcceleration(&dx_raw, &dy_raw, &dz_raw);
  
  // 读取陀螺仪数据
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  // 计算位移变化量，并转换为g单位
  delta_dx = (dx_raw - prev_dx) / 16384.0;
  delta_dy = (dy_raw - prev_dy) / 16384.0;
  delta_dz = (dz_raw - prev_dz) / 16384.0;

  // 转换角速度为°/s单位
  gx_deg_per_sec = gx_raw / 131.072;
  gy_deg_per_sec = gy_raw / 131.072;
  gz_deg_per_sec = gz_raw / 131.072;

  // 输出位移变化量和角速度数据
  Serial.print(delta_dx); Serial.print(",");
  Serial.print(delta_dy); Serial.print(",");
  Serial.print(delta_dz); Serial.print(",");
  Serial.print(gx_deg_per_sec); Serial.print(",");
  Serial.print(gy_deg_per_sec); Serial.print(",");
  Serial.print(gz_deg_per_sec); Serial.println(";");

  // 更新上一次的位移数据
  prev_dx = dx_raw;
  prev_dy = dy_raw;
  prev_dz = dz_raw;

  delay(50); // 可根据需要调整延时
}
