#!/usr/bin/env python3
# -- coding: utf-8 --
import sys
from pathlib import Path

import rospy
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

from pyproj import Transformer

network_module_path = Path(__file__).resolve().parents[3] / 'MORAI-NetworkModule'
sys.path.append(str(network_module_path))

from lib.network.UDP import Receiver
from lib.define.GPS import GPS
from lib.define.IMU import IMU


# UDP 통신 정리 ============
# GPS_Data
GPS_IP = '127.0.0.1'
GPS_PORT = 1111    # Destination PORT로 받아야 함
# IMU_Data
IMU_IP = '127.0.0.1'
IMU_PORT = 1112    # Destination PORT로 받아야 함
#========================

transformer = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)

class SensorDataHub:
    def __init__(self):      
        self.countA = 0

        self.sensor_gps = Receiver(GPS_IP, GPS_PORT, GPS())
        self.sensor_imu = Receiver(IMU_IP, IMU_PORT, IMU())

        # 발행자 선언
        self.localization_pub = rospy.Publisher('/current_pose', Point, queue_size=10)
        self.accel_pub = rospy.Publisher('/accel',Point, queue_size=10)

        # UDP로 받은 데이터 저장 공간
        self.sub_imu_heading = 0.0
        self.pos = Point()
        self.pos1 = [0, 0]
        
        
        self.accel_x = 0
        self.accel_y = 0
        self.accel = Point()


    def update_udp_data(self):
        self.update_gps_data()
        self.update_imu_data()

    def update_gps_data(self):
        gps_data = self.sensor_gps.get_data()

        try:
            gps_data.parsing()
        except Exception:
            return

        lat = gps_data.gpgga.lat
        lon = gps_data.gpgga.lon

        if lat == 0.0 or lon == 0.0:
            lat = gps_data.gprmc.lat
            lon = gps_data.gprmc.lon

        if lat == 0.0 or lon == 0.0:
            return

        x,y = transformer.transform(lon,lat)
        self.pos1 = [x, y]
        self.countA +=1

    def update_imu_data(self):
        imu = self.sensor_imu.get_data()

        self.accel_x = imu.lin_acc_x
        self.accel_y = imu.lin_acc_y

        quaternion = (imu.ori_x, imu.ori_y, imu.ori_z, imu.ori_w)

        if sum(q * q for q in quaternion) <= 1e-12:
            return

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        self.sub_imu_heading = yaw
        
    
    def localization_update(self):
        x, y = self.pos1[0], self.pos1[1]
        self.pos.x = x
        self.pos.y = y
        self.pos.z = self.sub_imu_heading

        print(f" x=  {self.pos.x}, y = {self.pos.y}, heading = {self.pos.z}")


    def pub_pose(self):
        self.localization_pub.publish(self.pos)
        

    def pub_accel(self):
        self.accel.x = self.accel_x
        self.accel.y = self.accel_y

        print(f" accel_x=  {self.accel.x}, accel_y = {self.accel.y}")
        
        self.accel_pub.publish(self.accel)


    
def main():
    # 기본 설정
    rospy.init_node('data_hub', anonymous=True)
    
    Data = SensorDataHub()
    rate = rospy.Rate(10)  
    while Data.countA < 5 and not rospy.is_shutdown():
        Data.update_udp_data()
        rate.sleep()

    while not rospy.is_shutdown():

        # 각 정보 업데이트 후 발행
        Data.update_udp_data()
        Data.localization_update()
        Data.pub_pose()
        Data.pub_accel()
        
        rate.sleep()
        

if __name__ == '__main__':
    main()
