import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import tf_transformations
import time

HALF_DISTANCE_BETWEEN_WHEELS = 0.0355
WHEEL_RADIUS = 0.02

class DiffDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__supervisor = self.__robot

        self.__robot_node = self.__supervisor.getSelf()

        self.__trans_field = self.__robot_node.getField("translation")
        self.__rot_field = self.__robot_node.getField("rotation")

        self.__trans_values = self.__trans_field.getSFVec3f()
        self.__rot_values = self.__rot_field.getSFRotation()

        self.__x_pose = self.__trans_values[0]
        self.__y_pose = self.__trans_values[1]
        self.__rot_dir = self.__rot_values[2]
        self.__orient = (self.__rot_dir * self.__rot_values[3])  


        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        
        


        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('diff_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__publisher = self.__node.create_publisher(Pose,'epuck_pose',10) 

        self.__pose = Pose()
        #Define the PID controller params 
        self.__E_l = 0
        self.__E_r = 0
        self.__edot_l = 0
        self.__edot_r = 0
        self.__old_e_l = 0
        self.__old_e_r = 0




    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        
        #get the current pose and publish it

        self.__trans_field = self.__robot_node.getField("translation")
        self.__rot_field = self.__robot_node.getField("rotation")

        self.__trans_values = self.__trans_field.getSFVec3f()
        self.__rot_values = self.__rot_field.getSFRotation()


        self.__x_pose = self.__trans_values[0]
        self.__y_pose = self.__trans_values[1]
        self.__rot_dir = self.__rot_values[2]
        self.__orient = (self.__rot_dir * self.__rot_values[3]) 

        orientation = tf_transformations.quaternion_from_euler(0,0,self.__orient)


        self.__pose.position.x = self.__x_pose
        self.__pose.position.y = self.__y_pose
        self.__pose.position.z = 0.0
        self.__pose.orientation.x = orientation[0]
        self.__pose.orientation.y = orientation[1]
        self.__pose.orientation.z = orientation[2]
        self.__pose.orientation.w = orientation[3]

        self.__publisher.publish(self.__pose)
        

        forward_speed = self.__target_twist.linear.x
        if forward_speed > 0.13:
            forward_speed = 0.13
        elif forward_speed < -0.13:
            forward_speed = -0.13

        
        angular_speed = self.__target_twist.angular.z
        if angular_speed > 2.826:
            angular_speed = 2.826
        elif angular_speed < -2.826:
            angular_speed = -2.826

        command_motor_left = ((forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS) 
        command_motor_right = ((forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS )

        '''
        current_left = self.__left_motor.getVelocity()
        current_right = self.__right_motor.getVelocity()

        e0l = command_motor_left - current_left
        e0r = command_motor_right - current_right

        kp = 0.5
        ki = 0.17 
        kd = 0.1 
        
        edot_l = e0l - self.__old_e_l
        edot_r = e0r - self.__old_e_r

        self.__E_l = self.__E_l + e0l
        self.__E_r = self.__E_r + e0r 

        E_l = self.__E_l
        E_r = self.__E_r
        
        u_l = kp*e0l + ki * E_l + kd* edot_l
        u_r = kp * e0r + ki*E_r + kd* edot_r 

        self.__old_e_l = e0l        
        self.__old_e_r = e0r
        
        '''

        self.__left_motor.setVelocity(command_motor_left)
    
        self.__right_motor.setVelocity(command_motor_right)

        
        
        
       


        
        


    
