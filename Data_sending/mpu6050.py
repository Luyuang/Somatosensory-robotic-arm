#!/usr/bin/env python


# # 设置串口参数
# port = 'COM3'  # 根据你的电脑更改串口号
# baudrate = 9600  # 与Arduino代码中的波特率相同

# # 初始化串口连接
# ser = serial.Serial(port, baudrate, timeout=1)

# # 解析串口数据并赋予变量名称
# def parse_data(data):
#     try:
#         # 分割字符串以获取加速度和角速度数据
#         accel_gyro_data = data.strip().split(" | ")
#         accel_data = accel_gyro_data[0].split(": ")[1].split(", ")
#         gyro_data = accel_gyro_data[1].split(": ")[1].split(", ")

#         # 提取加速度数据
#         ax_g = float(accel_data[0].split("g")[0])
#         ay_g = float(accel_data[1].split("g")[0])
#         az_g = float(accel_data[2].split("g")[0])

#         # 提取角速度数据
#         gx_deg_per_sec = float(gyro_data[0].split("°/s")[0])
#         gy_deg_per_sec = float(gyro_data[1].split("°/s")[0])
#         gz_deg_per_sec = float(gyro_data[2].split("°/s")[0])

#         print(f"ax_g: {ax_g}, ay_g: {ay_g}, az_g: {az_g}, gx_deg_per_sec: {gx_deg_per_sec}, gy_deg_per_sec: {gy_deg_per_sec}, gz_deg_per_sec: {gz_deg_per_sec}")

#         # mpu_data = [ax_g, ay_g, az_g, gx_deg_per_sec, gy_deg_per_sec, gz_deg_per_sec]

#     except Exception as e:
#         print(f"Error parsing data: {e}")
    
#     # return mpu_data

# # 主循环，持续读取串口数据
# try:
#     while True:
#         if ser.in_waiting > 0:
#             line = ser.readline().decode('utf-8')  # 读取一行数据
#             parse_data(line)
#         time.sleep(0.3)  # 稍作延时，避免CPU占用过高
# except KeyboardInterrupt:
#     print("Exiting program.")
# finally:
#     ser.close()  # 关闭串口连接




#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import Float32MultiArray

# 初始化ROS节点
rospy.init_node('serial_data_publisher')

# 串口配置，根据实际情况修改
serial_port = '/dev/ttyUSB0'  # 串口设备路径
baud_rate = 9600  # 波特率

# 创建一个serial对象
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# 创建一个publisher，发布Float32MultiArray类型的消息
publisher = rospy.Publisher('imu_data', Float32MultiArray, queue_size=10)

# 创建一个Float32MultiArray消息对象
imu_data_msg = Float32MultiArray()

def parse_data(data):
    # 去掉换行符和分号
    data = data.strip().strip(";")
    # 按逗号分割数据，并转成浮点数数组
    return [float(val) for val in data.split(',')]

# 循环读取串口数据
while not rospy.is_shutdown():
    try:
        # 读取一行数据
        line = ser.readline().decode('utf-8').strip()
        
        if ',' in line and ';' in line:
            # 解析串口数据
            values = parse_data(line)

            # 检查解析的数据长度是否正确（6个数据：加速度 x, y, z 和角速度 x, y, z）
            if len(values) == 6:
                ax_g, ay_g, az_g, gx_deg_per_sec, gy_deg_per_sec, gz_deg_per_sec = values

                # 将数据填充到消息对象中
                imu_data_msg.data = [ax_g, ay_g, az_g, gx_deg_per_sec, gy_deg_per_sec, gz_deg_per_sec]

                # 发布消息
                publisher.publish(imu_data_msg)
            rospy.loginfo("Published IMU data: %s", imu_data_msg.data)
            
    except Exception as e:
        rospy.logerr("Error reading from serial port: %s", e)

# 关闭串口
ser.close()


