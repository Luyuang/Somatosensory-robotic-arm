#!/usr/bin/env python3
# encoding: utf-8

import rospy
import serial
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

class ArmIK:
    def __init__(self):
        pass

    # 多项式拟合函数
    def poly5(self, x, p):
        return p[0] * x**5 + p[1] * x**4 + p[2] * x**3 + p[3] * x**2 + p[4] * x + p[5]

    # 将 roll、pitch、yaw 转换为舵机角度
    def calculate_servo_angles(self, roll, pitch, yaw):
        # Yaw -> Rotor 6
        yaw_coeffs = [-0.0000, -0.0000, 0.0013, 0.0232, -5.2741, 502.4103]
        servo6 = self.poly5(yaw, yaw_coeffs)

        # Roll -> Rotor 2
        roll_coeffs = [0.0000, -0.0000, -0.0009, 0.0053, 3.8751, 506.0956]
        servo2 = self.poly5(roll, roll_coeffs)

        # Pitch -> Rotor 3
        pitch_to_3_coeffs = [0.0000, 0.0000, -0.0011, -0.0210, 4.0276, 379.4103]
        servo3 = self.poly5(pitch, pitch_to_3_coeffs)

        # Pitch -> Rotor 4
        pitch_to_4_coeffs = [-0.0000, 0.0000, 0.0001, -0.0030, -1.0928, 699.2331]
        servo4 = self.poly5(pitch, pitch_to_4_coeffs)

        # Pitch -> Rotor 5
        pitch_to_5_coeffs = [-0.0000, -0.0000, 0.0003, 0.0021, 0.6219, 467.3407]
        servo5 = self.poly5(pitch, pitch_to_5_coeffs)

        return {
            "servo2": int(servo2),
            "servo3": int(servo3),
            "servo4": int(servo4),
            "servo5": int(servo5),
            "servo6": int(servo6),
        }

# 初始化串口
def init_serial(port='/dev/ttyUSB0', baudrate=19200, timeout=1):
    return serial.Serial(port, baudrate=baudrate, timeout=timeout)

# 读取串口数据
def read_data(serial_port):
    if serial_port.in_waiting:
        line = serial_port.readline().decode('utf-8').strip()
        try:
            # 将数据按逗号分割，并去掉最后的分号
            roll, pitch, yaw = map(float, line.rstrip(';').split(','))
            return roll, pitch, yaw
        except ValueError:
            rospy.logerr(f"数据解析错误: {line}") 
            return None
    return None

# 控制机械臂
def control_arm(joints_pub, servo_data):
    # 发送舵机目标角度到 ROS
    msg = MultiRawIdPosDur()
    msg.positions = [
        (2, servo_data['servo2']),
        (3, servo_data['servo3']),
        (4, servo_data['servo4']),
        (5, servo_data['servo5']),
        (6, servo_data['servo6']),
    ]
    msg.duration = 1000  # 1秒完成动作
    joints_pub.publish(msg)

def main():
    rospy.init_node('imu_to_arm', log_level=rospy.INFO)

    # 初始化串口和 ArmIK 类
    serial_port = init_serial()
    arm_ik = ArmIK()

    # 创建 ROS 发布者
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2)

    # 主循环
    while not rospy.is_shutdown():
        data = read_data(serial_port)
        if data:
            roll, pitch, yaw = data
            rospy.loginfo(f"Received: Roll={roll}, Pitch={pitch}, Yaw={yaw}")
            servo_data = arm_ik.calculate_servo_angles(roll, pitch, yaw)
            control_arm(joints_pub, servo_data)
        rospy.sleep(0.1)  # 控制频率，避免过度占用资源

if __name__ == "__main__":
    main()
