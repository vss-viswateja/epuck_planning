import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Range
import tf_transformations
import math


class BugZeroAlgorithm(Node):
    def __init__(self, x_goal, y_goal):
        super().__init__('bug_zero_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'epuck_pose', self.pose_callback, 1)
        self.ps0_subscription = self.create_subscription(Range, '/e_puck/ps0', self.ps0_callback, 1)
        self.ps7_subscription = self.create_subscription(Range, '/e_puck/ps7', self.ps7_callback, 1)
        self.ps5_subscription = self.create_subscription(Range, '/e_puck/ps5', self.ps5_callback, 1)
        
        self.current_pose = Pose()
        self.ps0_val = 0.0
        self.ps7_val = 0.0
        self.ps5_val = 0.0
        
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.goal_reached = False

    def ps0_callback(self, msg):
        self.ps0_val = msg.range

    def ps7_callback(self, msg):
        self.ps7_val = msg.range

    def ps5_callback(self, msg):
        self.ps5_val = msg.range

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_orientation(self):
        quat = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ]
        return tf_transformations.euler_from_quaternion(quat)[2]

    def go_to_goal(self):
        msg = Twist()
        p_linear, p_angular = 0.5, 0.2
        i_angular, d_angular = 0.005, 0.001
        ang_int, old_ang = 0, 0

        while not self.goal_reached:
            rclpy.spin_once(self)
            x_pose, y_pose = self.current_pose.position.x, self.current_pose.position.y
            robot_orientation = self.get_orientation()
            
            goal_angle = math.atan2(self.y_goal - y_pose, self.x_goal - x_pose)
            distance = math.sqrt((x_pose - self.x_goal) ** 2 + (y_pose - self.y_goal) ** 2)
            
            if distance < 0.01:
                self.goal_reached = True
                break

            angle_error = goal_angle - robot_orientation
            ang_int += angle_error
            ang_dot = angle_error - old_ang
            old_ang = angle_error

            msg.linear.x = min(distance * p_linear, 0.25)
            msg.angular.z = angle_error * p_angular + ang_int * i_angular + ang_dot * d_angular

            if (self.ps7_val + self.ps0_val) / 2 <= 0.065:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                print("Obstacle detected. Switching to obstacle avoidance.")
                self.avoid_obstacle()
                return

            self.publisher.publish(msg)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        print("Goal reached!")

    def set_orientation(self, target_orientation):
        msg = Twist()
        p_angular, i_angular, d_angular = 0.2, 0.005, 0.001
        ang_int, old_ang = 0, 0
        
        while True:
            rclpy.spin_once(self)
            robot_orientation = self.get_orientation()
            angle_error = target_orientation - robot_orientation
            ang_int += angle_error
            ang_dot = angle_error - old_ang
            old_ang = angle_error

            if abs(angle_error) <= 0.1:
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                break
            else:
                msg.angular.z = angle_error * p_angular + ang_int * i_angular + ang_dot * d_angular
                self.publisher.publish(msg)

    def avoid_obstacle(self):
        msg = Twist()
        self.set_orientation(self.get_orientation() - math.pi / 2)
        
        while self.ps5_val <= 0.065:
            rclpy.spin_once(self)
            msg.linear.x = 0.07
            self.publisher.publish(msg)
            
            if (self.ps7_val + self.ps0_val) / 2 <= 0.065:
                self.avoid_obstacle()
                return

    def run(self):
        while not self.goal_reached:
            self.go_to_goal()
        rclpy.shutdown()


def main():
    rclpy.init()
    x_goal = float(input("Enter x goal: "))
    y_goal = float(input("Enter y goal: "))
    
    bug_zero = BugZeroAlgorithm(x_goal, y_goal)
    bug_zero.run()


if __name__ == '__main__':
    main()
