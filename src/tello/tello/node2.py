#!/usr/bin/env python3
import rclpy
import time
import threading
#import tf2_ros
from rclpy.node import Node
from .sockets_commands import TelloDrone 
from tello_msg.msg import TelloStatus, TelloID, TelloWifiConfig

from std_msgs.msg import Empty, UInt8, UInt8, Bool, String
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Quaternion
#from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo
#from nav_msgs.msg import Odometry

class DRONE2(Node): # MODIFY NAME
    def __init__(self):
       super().__init__("drone_1") # MODIFY NAME

       # Declare parameters
       #self.declare_parameter('connect_timeout', 10.0)
       self.declare_parameter('tello_ip', '192.168.0.102')
       #self.declare_parameter('tf_base', 'map')
       #self.declare_parameter('tf_drone', 'drone')
       #self.declare_parameter('tf_pub', False)
       #self.declare_parameter('camera_info_file', '')
       
       # Get parameters
       #self.connect_timeout = float(self.get_parameter('connect_timeout').value)
       self.tello_ip = str(self.get_parameter('tello_ip').value)
       #self.tf_base = str(self.get_parameter('tf_base').value)
       #self.tf_drone = str(self.get_parameter('tf_drone').value)
       #self.tf_pub = bool(self.get_parameter('tf_pub').value)
       #self.camera_info_file = str(self.get_parameter('camera_info_file').value)
       
       # IP and port of Tello
       tello_address = (self.tello_ip, 8889)
       # IP and port of local computer
       local_address = ('', 9011)
       
       self.tello = TelloDrone(tello_address,local_address)

       self.setup_publishers()
       self.setup_subscribers()

    # Setup ROS publishers of the node.
    def setup_publishers(self):
        #self.pub_image_raw = self.create_publisher(Image, 'image_raw', 10)
        #self.pub_camera_info = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.pub_status = self.create_publisher(TelloStatus, 'status', 10)
        #self.pub_id = self.create_publisher(TelloID, 'id', 10)
        #self.pub_imu = self.create_publisher(Imu, 'imu', 10)
        #self.pub_battery = self.create_publisher(BatteryState, 'battery', 10)
        #self.pub_temperature = self.create_publisher(Temperature, 'temperature', 10)
        #self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        #self.pub_pose_imu = self.create_publisher(PoseStamped, 'pose_imu',10)

        # TF broadcaster
        #if self.tf_pub:
            #self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def setup_subscribers(self):
        #self.sub_emergency = self.create_subscription(Empty, 'emergency', self.emergency, 10)
        self.sub_takeoff = self.create_subscription(Empty, 'takeoff', self.takeoff, 10)
        self.sub_land = self.create_subscription(Empty, 'land', self.land, 10)
        self.sub_control = self.create_subscription(Twist, 'control', self.control, 10)
        #self.sub_flip = self.create_subscription(String, 'flip', self.flip, 10)

    def Telemetry(self, delay):
        self.tello.parse_telemetry(delay=0.01)

    # Start drone info thread
    def start_tello_status(self, rate=1.0/2.0):
        def status_loop():
            while True:
                
                # Tello Status
                if self.pub_status.get_subscription_count() > 0:
                    msg = TelloStatus()
                    msg.acceleration.x = self.tello.get_acceleration_x()
                    msg.acceleration.y = self.tello.get_acceleration_y()
                    msg.acceleration.z = self.tello.get_acceleration_z()

                    msg.speed.x = float(self.tello.get_speed_x())
                    msg.speed.y = float(self.tello.get_speed_y())
                    msg.speed.z = float(self.tello.get_speed_z())

                    msg.pitch = self.tello.get_pitch()
                    msg.roll = self.tello.get_roll()
                    msg.yaw = self.tello.get_yaw()

                    msg.barometer = int(self.tello.get_barometer())
                    msg.distance_tof = self.tello.get_distance_tof()

                    msg.fligth_time = self.tello.get_flight_time()

                    msg.battery = self.tello.get_battery()

                    msg.highest_temperature = self.tello.get_highest_temperature()
                    msg.lowest_temperature = self.tello.get_lowest_temperature()
                    #msg.temperature = self.tello.get_temperature()

                    self.pub_status.publish(msg)
                
                # Sleep
                time.sleep(rate)

        thread = threading.Thread(target=status_loop)
        thread.start()
        return thread

    def command(self, delay):
        self.tello.command(delay)

    def takeoff(self, delay):
        self.tello.takeoff(delay)

    def land(self, delay):
        self.tello.land(delay)
    
    # Receives the linear and angular velocities to be applied from -100 to 100.
    def control(self, cmd):
        self.tello.send_rc_control(int(cmd.linear.x), int(cmd.linear.y), int(cmd.linear.z), int(cmd.angular.z))


def main(args=None):
    rclpy.init(args=args)
    drone1 = DRONE2() # MODIFY NAME
    drone1.command(1)
    rclpy.spin(drone1)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()