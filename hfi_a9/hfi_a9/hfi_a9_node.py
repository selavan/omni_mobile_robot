#!/usr/bin/env python3
import serial
import struct
import math
import serial.tools.list_ports
from sensor_msgs.msg import Imu, MagneticField
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3Stamped

def checkSum(list_data, check_data):
    data = bytearray(list_data)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])


# 16 进制转 ieee 浮点数
def hex_to_ieee(raw_data):
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        data2str =hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
        ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
    ieee_data.reverse()
    return ieee_data

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q

class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.yaw_publisher = self.create_publisher(Vector3Stamped, 'yaw_oem', 10)

        self.key = 0
        self.flag = 0
        self.buff = {}
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pub_flag= [True, True]
        self.data_right_count = 0

        self.gra_normalization = True

        self.setup_serial_port()

        self.timer = self.create_timer(0.01, self.read_from_serial_port)
        
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()
        self.yaw_msg = Vector3Stamped()

        self.full_imu_rotations = 0
        self.previous_imu_raw_yaw = None

    def setup_serial_port(self):
        try:
            self.hf_imu = serial.Serial(port="/dev/ttyUSB0", baudrate=921600, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info("Serial port opened successfully")
            else:
                self.hf_imu.open()
                self.get_logger().info("Opened serial port")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.destroy_node()
            exit(0)

    def read_from_serial_port(self):
        if self.hf_imu.inWaiting() > 0:
            buff_data = self.hf_imu.read(self.hf_imu.inWaiting())
            for byte_data in buff_data:
                self.handleSerialData(byte_data)
        
        self.imu_publisher.publish(self.imu_msg)
        self.mag_publisher.publish(self.mag_msg)
        self.yaw_publisher.publish(self.yaw_msg)

    def handleSerialData(self, raw_data):

        if self.data_right_count > 200000:
            print("该设备传输数据错误，退出")
            exit(0)

        self.buff[self.key] = raw_data

        self.key += 1
        if self.buff[0] != 0xaa:
            self.data_right_count += 1
            self.key = 0
            return
        if self.key < 3:
            return
        if self.buff[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buff[2] + 5:  # 根据数据长度位的判断, 来获取对应长度数据
            return

        else:
            self.data_right_count = 0
            data_buff = list(self.buff.values())  # 获取字典所以 value

            if self.buff[2] == 0x2c and self.pub_flag[0]:
                if checkSum(data_buff[2:47], data_buff[47:49]):
                    data = hex_to_ieee(data_buff[7:47])
                    self.angularVelocity = data[1:4]
                    self.acceleration = data[4:7]
                    self.magnetometer = data[7:10]
                else:
                    print('校验失败')
                self.pub_flag[0] = False
            elif self.buff[2] == 0x14 and self.pub_flag[1]:
                if checkSum(data_buff[2:23], data_buff[23:25]):
                    data = hex_to_ieee(data_buff[7:23])
                    self.angle_degree = data[1:4]
                else:
                    print('校验失败')
                self.pub_flag[1] = False
            else:
                print("该数据处理类没有提供该 " + str(self.buff[2]) + " 的解析")
                print("或数据错误")
                self.buff = {}
                self.key = 0

            self.buff = {}
            self.key = 0
            #if self.pub_flag[0] == True or self.pub_flag[1] == True:
            #    return
            self.pub_flag[0] = self.pub_flag[1] = True
            stamp = self.get_clock().now().to_msg()

            self.imu_msg = Imu()

            self.imu_msg.header.stamp = stamp
            self.imu_msg.header.frame_id = "base_link"

            self.mag_msg = MagneticField()

            self.mag_msg.header.stamp = stamp
            self.mag_msg.header.frame_id = "base_link"

            angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
            qua = quaternion_from_euler(angle_radian[0], -angle_radian[1], -angle_radian[2])

            self.imu_msg.orientation.x = qua.x
            self.imu_msg.orientation.y = qua.y
            self.imu_msg.orientation.z = qua.z
            self.imu_msg.orientation.w = qua.w


            self.imu_msg.angular_velocity.x = self.angularVelocity[0]
            self.imu_msg.angular_velocity.y = self.angularVelocity[1]
            self.imu_msg.angular_velocity.z = self.angularVelocity[2]
            
            acc_k = math.sqrt(self.acceleration[0] ** 2 + self.acceleration[1] ** 2 + self.acceleration[2] ** 2)
            if acc_k == 0:
                acc_k = 1
            
            if self.gra_normalization:
                self.imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8 / acc_k
                self.imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8 / acc_k
                self.imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8 / acc_k
            else:
                self.imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8
                self.imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8
                self.imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8

            self.mag_msg.magnetic_field.x = self.magnetometer[0]
            self.mag_msg.magnetic_field.y = self.magnetometer[1]
            self.mag_msg.magnetic_field.z = self.magnetometer[2]

            #self.imu_publisher.publish(imu_msg)
            #self.mag_publisher.publish(mag_msg)
            #self.yaw_publisher.publish(yaw_msg)

            self.imu_raw_yaw = self.quaternion_to_yaw(self.imu_msg.orientation)

            self.calculate_imu_yaw()

            self.yaw_msg = Vector3Stamped()
            self.yaw_msg.header.stamp = stamp
            self.yaw_msg.vector.z = self.imu_yaw
            #self.yaw_publisher.publish(yaw_msg)

    def calculate_imu_yaw(self):
        # Initialize previous_raw_yaw_mag on the first run
        if self.previous_imu_raw_yaw is None:
            self.previous_imu_raw_yaw = self.imu_raw_yaw

        yaw_diff = self.imu_raw_yaw - self.previous_imu_raw_yaw

        # Check for full rotation
        if yaw_diff > math.pi:
            self.full_imu_rotations -= 1
        elif yaw_diff < -math.pi:
            self.full_imu_rotations += 1

        self.previous_imu_raw_yaw = self.imu_raw_yaw

        # Update continuous yaw
        self.imu_yaw = self.imu_raw_yaw + (self.full_imu_rotations * 2 * math.pi)

    def quaternion_to_yaw(self, orientation):
        w = orientation.w
        x = orientation.x
        y = orientation.y
        z = orientation.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        #self.imu_raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
