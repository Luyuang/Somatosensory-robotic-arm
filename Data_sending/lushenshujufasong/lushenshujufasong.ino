#include "Wire.h"
#include "MPU6050.h"

// 初始化MPU6050
MPU6050 mpu;
int16_t ax, ay, az; // 加速度计原始数据
int16_t gx, gy, gz; // 陀螺仪原始数据

void setup() {
  // 初始化串口
  Serial.begin(9600);

  // 初始化I2C
  Wire.begin();

  // 初始化MPU6050
  mpu.initialize();

  // 检查MPU6050是否连接成功
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  // 读取加速度计数据
  mpu.getAcceleration(&ax, &ay, &az);
  // 读取陀螺仪数据
  mpu.getRotation(&gx, &gy, &gz);

  // 通过串口发送数据
// 加速度转换（以g为单位）
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  // 角速度转换（以°/s为单位）
  float gx_deg_per_sec = gx / 131.072;
  float gy_deg_per_sec = gy / 131.072;
  float gz_deg_per_sec = gz / 131.072;

  // 通过串口发送转换后的数据

  Serial.print(ax_g); Serial.print(",");
  Serial.print(ay_g); Serial.print(",");
  Serial.print(az_g); Serial.print(",");

  Serial.print(gx_deg_per_sec); Serial.print(",");
  Serial.print(gy_deg_per_sec); Serial.print(",");
  Serial.print(gz_deg_per_sec); Serial.println(";");


  // 稍作延时
  delay(100);
}
