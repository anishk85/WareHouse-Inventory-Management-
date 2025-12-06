import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

class ReturnToSpawnNode(Node):
    """
    A ROS 2 node that cancels all active navigation goals and then commands 
    the robot to navigate back to its spawn (origin) position (0.0, 0.0, 0.0).
    """

    def __init__(self):
        super().__init__('return_to_spawn_node')
        self.get_logger().info('ReturnToSpawnNode initialized. Preparing to navigate...')
        
        self.navigator = BasicNavigator()


        self.navigator.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Nav2 services are available.')
        

        self.spawn_pose = self._create_spawn_pose()

        self.timer = self.create_timer(1.0, self.execute_return_sequence)
        self.executed = False 

    def _create_spawn_pose(self):
        """Creates a PoseStamped message for the spawn location (0.0, 0.0, 0.0)."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = 0.8
        pose.pose.position.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0  # Identity quaternion for 0 rotation
        return pose

    def execute_return_sequence(self):
        """Executes the goal cancellation and navigation to spawn."""
        if self.executed:
            # Stop the timer once the sequence has started
            self.timer.cancel()
            return

        self.executed = True
        
        self.get_logger().info('🛑 **Canceling all active navigation goals...**')
        # The BasicNavigator.cancel_all_goals() method handles terminating all current goals
        self.navigator.cancel_all_goals() 
        self.get_logger().info('All active goals canceled.')

        # Wait a moment for cancellation to take effect
        rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('🏠 **Sending new goal: Return to spawn (0, 0, 0)...**')
        self.navigator.goToPose(self.spawn_pose)

        # Monitor the goal progress until it's finished
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(
                    f'Distance remaining to spawn: {feedback.distance_remaining:.2f} meters'
                )
            
            # Allow for termination in case the user wants to stop it manually
            if not rclpy.ok():
                break

        # Goal reached or failed
        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info('✅ Robot successfully returned to spawn!')
        elif result == BasicNavigator.TaskResult.CANCELED:
            self.get_logger().warn('Task was canceled externally.')
        elif result == BasicNavigator.TaskResult.FAILED:
            self.get_logger().error('❌ Goal failed! Could not reach spawn.')
        else:
            self.get_logger().info(f'Task completed with result: {result}')

        # Terminate the node after the task is complete
        self.get_logger().info('ReturnToSpawnNode terminating.')
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToSpawnNode()
    
    # Use single spin to let the node execute its sequence and then shut down
    rclpy.spin(node) 

    rclpy.shutdown()

if __name__ == '__main__':
    main()