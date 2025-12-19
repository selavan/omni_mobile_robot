import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import time
import math
import json
import os


def sign(value):
    return 1 if value >= 0 else -1

class ImuMagCalibrator(Node):
    def __init__(self):
        super().__init__('imu_mag_calibrator')

        # Subscribers
        self.imu_sub = Subscriber(self, Imu, '/HFI_A9_Imu')
        self.mag_sub = Subscriber(self, MagneticField, '/HFI_A9_Mag')
        
        self.yaw_publisher = self.create_publisher(Float32, '/HFI_A9_yaw', 10)

        # Synchronize IMU and Magnetometer messages
        ats = ApproximateTimeSynchronizer([self.imu_sub, self.mag_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.sync_callback)

        # Calibration data storage
        self.calibration_data = []
        self.mag_angle_resolution = 2 #store data at every 2 degrees if possible
        self.mag_calibration_data = np.zeros((int(720 / self.mag_angle_resolution), 3))
        self.mag_data_replace = np.zeros((int(720 / self.mag_angle_resolution), 3))

        self.calibration_time = 2.0  # 2 seconds for initial calibration
        self.start_time = time.perf_counter()

        self.last_time = time.perf_counter()
        self.yaw_gyro = 0.0  # Initialize yaw
        self.gyro_bias = np.array([0.0, 0.0, 0.0])  # Initialize gyro bias
        
        self.last_mag_calib_yaw = 0.0  # Last yaw angle at which mag data was collected

        self.gyro_calib_complete = False
        self.mag_calib_complete = False

        self.total_yaw_rotation = 0.0  # Track total yaw rotation
        self.mag_calib_trigger_rotation = 4 * math.pi

        self.hard_iron_offset = np.array([0.0, 0.0, 0.0])
        
        self.Ts = 1.0 / 300

        self.yaw_mag = 0.0
        self.yaw_rate = 0.0

        #EKF part
        self.x_est = np.array([0.0, 1.0, 0.0]).reshape(-1, 1)  # w, a, b, c
        self.new_x = np.copy(self.x_est)
        self.Fk = np.eye(3)
        self.Qk = np.diag([0.00001, 0.00001, 0.00001])
        self.Hk = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        self.P = np.eye(3)
        self.new_P = np.eye(3)
        self.Rk = np.array([10.0]).reshape(1, 1)

        #skip mag cal
        self.hard_iron_offset_file = 'hard_iron_offset.json'
        self.load_hard_iron_offset()

        #yaw continous
        self.full_rotations = 0
        self.previous_raw_yaw_mag = None

    def load_hard_iron_offset(self):
        if os.path.exists(self.hard_iron_offset_file):
            try:
                with open(self.hard_iron_offset_file, 'r') as file:
                    self.hard_iron_offset = np.array(json.load(file))
                    self.mag_calib_complete = True
                    self.get_logger().info('Loaded existing hard iron offset.')
            except Exception as e:
                self.get_logger().error(f'Failed to load hard iron offset: {e}')

    def sync_callback(self, imu_msg, mag_msg):
        if not self.gyro_calib_complete:
            self.calibrate_gyro(imu_msg)
        elif self.gyro_calib_complete:
            self.collect_mag_data(mag_msg)
            self.apply_gyro_bias_and_integrate_yaw(imu_msg)
            if self.all_mag_data_replace():
                self.calculate_hard_iron_offset()
                self.mag_data_replace = np.zeros((int(720 / self.mag_angle_resolution), 3))
            if self.mag_calib_complete:
                self.calculate_yaw_from_magnetometer(mag_msg)
                self.EKF()

    def EKF(self):
        self.new_x[0] += self.x_est[1] * self.yaw_rate * self.Ts + self.x_est[2]
        self.new_x[1] = self.x_est[1]
        self.new_x[2] = self.x_est[2]

        self.Fk[0, 0] = 1
        self.Fk[0, 1] = self.yaw_rate * self.Ts
        self.Fk[0, 2] = 1

        self.new_P = np.matmul(np.matmul(self.Fk, self.P), self.Fk.T) + self.Qk

        self.yk = self.yaw_mag - self.new_x[0]
        
        self.Kk = np.matmul(np.matmul(self.new_P, self.Hk.T), np.linalg.inv(np.matmul(np.matmul(self.Hk, self.new_P), self.Hk.T) + self.Rk))
                        
        self.new_x += self.yk * self.Kk

        self.new_P = np.matmul((np.eye(3) - np.matmul(self.Kk, self.Hk)), self.new_P)

        self.P = np.copy(self.new_P)

        self.x_est = np.copy(self.new_x)

        #print(self.new_x[0] * 180 / math.pi, self.x_est[1], self.x_est[2])
        yaw_msg = Float32()
        yaw_msg.data = float(self.new_x[0])
        self.yaw_publisher.publish(yaw_msg)

    def all_mag_data_replace(self):
        return np.all(self.mag_data_replace == 1)

    def collect_mag_data(self, mag_msg):
        yaw_degrees = (self.yaw_gyro * 180 / math.pi) % 720
        index = int(yaw_degrees / self.mag_angle_resolution)

        self.mag_calibration_data[index, :] = [mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z]
        self.mag_data_replace[index] = 1


    def calibrate_gyro(self, imu_msg):
        current_time = time.perf_counter()
        if current_time - self.start_time < self.calibration_time:
            gyro_data = (imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z)
            self.calibration_data.append(gyro_data)
        else:
            self.calculate_gyro_bias()
            self.gyro_calib_complete = True
            self.last_time = current_time  # Reset last_time for integration
            self.get_logger().info('Start mag calibration')
                
    def calculate_gyro_bias(self):
        data_array = np.array(self.calibration_data)
        self.gyro_bias = np.mean(data_array, axis=0)
        self.get_logger().info(f'Gyro Bias: {self.gyro_bias}')
        self.calibration_data.clear()

    def apply_gyro_bias_and_integrate_yaw(self, imu_msg):
        current_time = time.perf_counter()
        Ts = current_time - self.last_time  # Time step
        self.last_time = current_time

        # Correcting the gyro data with the calculated bias
        self.yaw_rate = imu_msg.angular_velocity.z - self.gyro_bias[2]

        # Integrating yaw (assuming only yaw changes significantly)
        self.yaw_gyro += self.yaw_rate * Ts

        self.total_yaw_rotation += abs(self.yaw_rate * Ts)

    def calculate_hard_iron_offset(self):
        data_array = np.array(self.mag_calibration_data)
        self.hard_iron_offset = np.mean(data_array, axis=0)
        self.mag_calib_complete = True
        self.get_logger().info(f'Hard Iron Offset: {self.hard_iron_offset}')
        self.save_hard_iron_offset()

    def save_hard_iron_offset(self):
        try:
            with open(self.hard_iron_offset_file, 'w') as file:
                json.dump(self.hard_iron_offset.tolist(), file)
                self.get_logger().info('Saved hard iron offset to file.')
        except Exception as e:
            self.get_logger().error(f'Failed to save hard iron offset: {e}')

    def calculate_yaw_from_magnetometer(self, mag_msg):
        corrected_mag_x = mag_msg.magnetic_field.x - self.hard_iron_offset[0]
        corrected_mag_y = mag_msg.magnetic_field.y - self.hard_iron_offset[1]

        raw_yaw_mag = math.atan2(-corrected_mag_y, corrected_mag_x)

        # Initialize previous_raw_yaw_mag on the first run
        if self.previous_raw_yaw_mag is None:
            self.previous_raw_yaw_mag = raw_yaw_mag

        yaw_diff = raw_yaw_mag - self.previous_raw_yaw_mag

        # Check for full rotation
        if yaw_diff > math.pi:
            self.full_rotations -= 1
        elif yaw_diff < -math.pi:
            self.full_rotations += 1

        # Update continuous yaw
        self.yaw_mag = raw_yaw_mag + (self.full_rotations * 2 * math.pi)
        self.previous_raw_yaw_mag = raw_yaw_mag


def main(args=None):
    rclpy.init(args=args)
    node = ImuMagCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
