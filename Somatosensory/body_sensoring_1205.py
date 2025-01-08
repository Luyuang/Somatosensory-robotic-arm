#!/usr/bin/env python3
# encoding:utf-8
# 从串口接收 roll, pitch, yaw 数据，根据计算结果控制机械臂

import serial
import time
import rospy
import armpi_fpv.bus_servo_control as bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

class ArmController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=1200, update_rate=10):
        """
        初始化串口和 ROS 节点
        """
        # 串口初始化
        self.serial = serial.Serial(port, baudrate, timeout=1)
        if not self.serial.is_open:
            self.serial.open()
        
        rospy.init_node('arm_control', log_level=rospy.DEBUG)

        # ROS 发布器
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(0.02)

        # 更新速率（Hz）
        self.update_rate = update_rate

    def read_serial_data(self):
        """
        从串口读取 roll, pitch, yaw 数据。
        数据格式：roll, pitch, yaw; 每行以分号 ';' 结束。
        返回整数部分的 roll, pitch, yaw。
        """
        try:
            if self.serial.in_waiting > 0:  # 检查串口是否有可读取的数据
                line = self.serial.readline().decode('utf-8').strip()  # 读取一行并去除两端空格
                if line.endswith(';'):  # 检查数据是否以分号结束
                    line = line[:-1]  # 移除末尾的分号
                    values = [float(x) for x in line.split(',')]  # 按逗号分割并转换为浮点数
                    if len(values) == 3:  # 确保数据包含 roll, pitch, yaw 三个值
                        roll, pitch, yaw = map(int, values)  # 取整数部分
                        return roll, pitch, yaw
        except Exception as e:
            rospy.logerr(f"数据解析错误: {e}")
        return None  # 如果数据无效，返回 None

    def calculate_servo_angles(self, roll, pitch, yaw):
        """
        根据 roll, pitch, yaw 计算舵机脉宽值
        """
        # 使用多项式计算脉宽值
        yaw_coeffs = [-0.0000, -0.0000, 0.0013, 0.0232, -5.2741, 502.4103]
        servo6 = self.poly5(yaw, yaw_coeffs)

        roll_coeffs = [0.0000, -0.0000, -0.0009, 0.0053, 3.8751, 506.0956]
        servo2 = self.poly5(roll, roll_coeffs)

        pitch_to_3_coeffs = [0.0000, 0.0000, -0.0011, -0.0210, 4.0276, 379.4103]
        servo3 = self.poly5(pitch, pitch_to_3_coeffs)

        pitch_to_4_coeffs = [-0.0000, 0.0000, 0.0001, -0.0030, -1.0928, 699.2331]
        servo4 = self.poly5(pitch, pitch_to_4_coeffs)

        pitch_to_5_coeffs = [-0.0000, -0.0000, 0.0003, 0.0021, 0.6219, 467.3407]
        servo5 = self.poly5(pitch, pitch_to_5_coeffs)

        return servo2, servo3, servo4, servo5, servo6

    def poly5(self, x, p):
        """
        多项式映射函数，用于根据多项式系数计算舵机脉宽值
        """
        return p[0] * x**5 + p[1] * x**4 + p[2] * x**3 + p[3] * x**2 + p[4] * x + p[5]

    def control_arm(self, servo_pulses):
        """
        根据计算出的脉宽值，发布舵机指令
        """
        try:
            rospy.loginfo(f"发布舵机脉宽: {servo_pulses}")
            bus_servo_control.set_servos(
                self.joints_pub, 1000,
                (
                    (2, int(servo_pulses[0])),
                    (3, int(servo_pulses[1])),
                    (4, int(servo_pulses[2])),
                    (5, int(servo_pulses[3])),
                    (6, int(servo_pulses[4]))
                )
            )
        except ValueError as e:
            rospy.logerr(f"舵机脉宽转换失败: {e}")

    def run(self):
        """
        主运行函数，循环读取串口数据并控制机械臂
        """
        rate = rospy.Rate(self.update_rate)  # 控制循环速率
        rospy.loginfo("机械臂控制程序启动")
        while not rospy.is_shutdown():
            data = self.read_serial_data()
            if data:
                roll, pitch, yaw = data
                rospy.loginfo(f"接收到数据: roll={roll}, pitch={pitch}, yaw={yaw}")
                servo_pulses = self.calculate_servo_angles(roll, pitch, yaw)
                self.control_arm(servo_pulses)
            else:
                rospy.logwarn("未接收到有效数据")
            rate.sleep()  # 控制循环速率

if __name__ == "__main__":
    try:
        controller = ArmController(update_rate=10)  # 更新速率设为 10 Hz
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("程序终止")
