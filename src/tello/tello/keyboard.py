#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import time

from std_msgs.msg import Empty, UInt8, UInt8, Bool, String
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Quaternion

class DroneController(Node): 
    
    def __init__(self):
        super().__init__("drone_controller") 

        self.setup_publishers()
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Keyboard Control")
        self.timer = self.create_timer(0.1, self.update)  # 10 FPS

        # Estados de los drones
        self.drone1_active = False
        self.drone2_active = False
       
    # Setup ROS publishers of the node.
    def setup_publishers(self):
        self.pub_takeoff = self.create_publisher(Empty, 'takeoff', 1)
        self.pub_land = self.create_publisher(Empty, 'land', 1)
        self.pub_control = self.create_publisher(Twist, 'control', 1)
        self.pub_takeoff = self.create_publisher(Empty, 'takeoff', 1)
        self.pub_land = self.create_publisher(Empty, 'land', 1)
        self.pub_control = self.create_publisher(Twist, 'control', 1)

        #self.sub_flip = self.create_publisher(String, 'flip', 10)

    def update(self):
        # RC Speed
        manual_speed = float(30)
        msg = Twist()

        # Comprobar eventos
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
            elif event.type == pygame.KEYDOWN:  # Evento al presionar una tecla
                if event.key == pygame.K_1:
                    self.drone1_active = not self.drone1_active  # Cambiar el estado de drone1 al presionar '1'
                    if self.drone1_active == True:
                        self.get_logger().info("Drone 1 Activado")
                    elif self.drone1_active == False:
                        self.get_logger().info("Drone 1 Desactivado")
                elif event.key == pygame.K_2:
                    self.drone2_active = not self.drone2_active  # Cambiar el estado de drone2 al presionar '2'
                    if self.drone2_active == True:
                        self.get_logger().info("Drone 2 Activado")
                    elif self.drone2_active == False:
                        self.get_logger().info("Drone 2 Desactivado")
                
                if self.drone1_active or self.drone2_active:  # Solo verificar flechas si al menos uno de los drones está activo
                    if event.key == pygame.K_UP:
                        if self.drone1_active:
                            msg.linear.y = manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 1: Adelante')
                        if self.drone2_active:
                            msg.linear.y = manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 2: Adelante')
                    elif event.key == pygame.K_DOWN:
                        if self.drone1_active:
                            msg.linear.y = -manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 1: Atrás')
                        if self.drone2_active:
                            msg.linear.y = -manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 2: Atrás')
                    elif event.key == pygame.K_LEFT:
                        if self.drone1_active:
                            msg.linear.x = -manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 1: Izquierda')
                        if self.drone2_active:
                            msg.linear.x = -manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 2: Izquierda')
                    elif event.key == pygame.K_RIGHT:
                        if self.drone1_active:
                            msg.linear.x = manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 1: Derecha')
                        if self.drone2_active:
                            msg.linear.x = manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 2: Derecha')
                    elif event.key == pygame.K_w:
                        if self.drone1_active:
                            msg.linear.z = manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 1: Arriba')
                        if self.drone2_active:
                            msg.linear.z = manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 2: Arriba')
                    elif event.key == pygame.K_s:
                        if self.drone1_active:
                            msg.linear.z = -manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 1: Abajo')
                        if self.drone2_active:
                            msg.linear.z = -manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 2: Abajo')
                    elif event.key == pygame.K_d:
                        if self.drone1_active:
                            msg.angular.z = manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 1: Rotar derecha')
                        if self.drone2_active:
                            msg.angular.z = manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 2: Rotar derecha')
                    elif event.key == pygame.K_a:
                        if self.drone1_active:
                            msg.angular.z = -manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 1: Rotar izquierda')
                        if self.drone2_active:
                            msg.angular.z = -manual_speed
                            self.pub_control.publish(msg)
                            self.get_logger().info('Drone 2: Rotar izquierda')
                    elif event.key == pygame.K_t:
                        if self.drone1_active:
                            self.pub_takeoff.publish(Empty())
                            self.get_logger().info('Drone 1: Takeoff')
                        if self.drone2_active:
                            self.pub_takeoff.publish(Empty())
                            self.get_logger().info('Drone 2: Takeoff')                            
                    elif event.key == pygame.K_l:
                        if self.drone1_active:
                            self.pub_land.publish(Empty())
                            self.get_logger().info('Drone 1: Land')
                        if self.drone2_active:
                            self.pub_land.publish(Empty())
                            self.get_logger().info('Drone 2: Land')
            elif event.type == pygame.KEYUP:  # Evento al soltar una tecla
                if event.key in [pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_w, pygame.K_s, pygame.K_d, pygame.K_a]:
                    self.get_logger().info('stop')
                    msg.linear.x = float(0)
                    msg.linear.y = float(0)
                    msg.linear.z = float(0)
                    msg.angular.z = float(0)
                    self.pub_control.publish(msg)



        # Actualizar la pantalla
        pygame.display.flip()

def main(args=None):
   rclpy.init(args=args)
   node = DroneController() 
   rclpy.spin(node)
   rclpy.shutdown()

if __name__ == "__main__":
    main()