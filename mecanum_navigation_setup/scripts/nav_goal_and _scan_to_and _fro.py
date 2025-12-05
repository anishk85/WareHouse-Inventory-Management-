import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    Input: roll, pitch, yaw
    Output: geometry_msgs.msg.Quaternion
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class MotionController(Node):
    """
    A simple node to handle the 'to and fro' motion publishing
    after navigation is complete.
    """
    def __init__(self):
        super().__init__('motion_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def move_straight(self, distance, speed):
        """
        Moves the robot straight for a specific distance at a specific speed.
        Positive speed = Forward, Negative speed = Backward.
        """
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0

        # Calculate duration needed: time = distance / speed
        duration_sec = abs(distance / speed)
        
        self.get_logger().info(f"Moving {'forward' if speed > 0 else 'backward'} {distance}m...")
        
        # We assume a loop rate of 10Hz
        start_time = time.time()
        
        while (time.time() - start_time) < duration_sec:
            self.publisher_.publish(msg)
            # We spin briefly to allow callbacks (though strictly not needed for just publishing)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop the robot
        self.stop()

    def stop(self):
        msg = Twist()
        self.publisher_.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)

class GoalReceiver(Node):
    """
    Listens for goals from RViz (2D Nav Goal tool usually publishes to /goal_pose).
    """
    def __init__(self):
        super().__init__('goal_receiver')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/wiggle_nav_goal',
            self.goal_callback,
            10)
        self.current_goal = None
        self.new_goal_received = False

    def goal_callback(self, msg):
        self.get_logger().info('Received new goal from RViz!')
        print(f'moving to: x={msg.pose.position.x}, y={msg.pose.position.y} qz={msg.pose.orientation.z}, qw={msg.pose.orientation.w}')
        self.current_goal = msg
        self.new_goal_received = True

def main():
    rclpy.init()

    # --- 1. Setup Navigation & Goal Receiver ---
    navigator = BasicNavigator()
    goal_receiver = GoalReceiver()
    motion_node = MotionController()

    # Wait for Nav2 to fully launch (optional, but good for cold starts)
    # navigator.waitUntilNav2Active() 

    print("------------------------------------------------")
    print("Waiting for a '2D Nav Goal' from RViz...")
    print("------------------------------------------------")

    try:
        while rclpy.ok():
            # Spin the goal receiver to check for messages
            rclpy.spin_once(goal_receiver, timeout_sec=0.1)

            if goal_receiver.new_goal_received:
                # Reset flag
                goal_receiver.new_goal_received = False
                target_pose = goal_receiver.current_goal

                # Send the goal
                print(f"Navigating to received goal...")
                navigator.goToPose(target_pose)

                # Monitor progress
                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    
                    # OPTIONAL: Allow redirecting if a NEW goal is clicked while moving
                    rclpy.spin_once(goal_receiver, timeout_sec=0.1)
                    if goal_receiver.new_goal_received:
                        print("New goal received during navigation! Redirecting...")
                        target_pose = goal_receiver.current_goal
                        navigator.goToPose(target_pose)
                        goal_receiver.new_goal_received = False

                # Check Result
                result = navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    print('Goal reached! Starting "To and Fro" motion...')
                    
                    # --- 2. Execute To and Fro Motion ---
                    # Move Forward 1m
                    motion_node.move_straight(distance=1, speed=0.2)
                    time.sleep(0.5)
                    # Move Backward 1m
                    motion_node.move_straight(distance=1, speed=-0.2)
                    
                    print("Motion sequence complete. Waiting for next goal...")

                elif result == TaskResult.CANCELED:
                    print('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    print('Goal failed!')
                
                # Reset current goal logic
                goal_receiver.current_goal = None

    except KeyboardInterrupt:
        pass
    finally:
        motion_node.destroy_node()
        goal_receiver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()