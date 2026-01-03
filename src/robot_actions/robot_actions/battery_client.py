#!/usr/bin/env python3
"""
Battery Charging Client
========================
This is like the battery indicator on your phone.
It requests charging and shows progress.
"""

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_interfaces.action import ChargeBattery


class BatteryClient(Node):
    """
    Client that requests battery charging.
    """

    def __init__(self):
        super().__init__('battery_client')
        
        # Create action client
        self.action_client = ActionClient(
            self,
            ChargeBattery,
            'charge_battery'
        )

    def send_goal(self, target):
        """
        Request charging to a specific percentage.
        """
        self.get_logger().info(f'🔌 Requesting charge to {target}%...')
        
        # Wait for server
        self.get_logger().info('⏳ Waiting for charging station...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Charging station not available!')
            return False
        
        # Create goal
        goal_msg = ChargeBattery.Goal()
        goal_msg.target_percentage = target
        
        # Send goal asynchronously
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Register callbacks
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True

    def goal_response_callback(self, future):
        """
        Called when server accepts/rejects our goal.
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Charging request rejected!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('✅ Request accepted!')
        self.get_logger().info('⚡ Charging in progress...')
        self.get_logger().info('')
        
        # Get result when done
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Called when charging is complete.
        """
        result = future.result().result
        status = future.result().status
        
        self.get_logger().info('')
        self.get_logger().info('=================================')
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('🎉 CHARGING COMPLETE!')
            self.get_logger().info(f'   Final battery: {result.final_percentage}%')
            self.get_logger().info(f'   Time taken: {result.charging_time:.1f}s')
            avg_rate = result.final_percentage / result.charging_time
            self.get_logger().info(f'   Avg rate: {avg_rate:.1f}%/s')
        elif status == 5:  # CANCELED
            self.get_logger().warn('🛑 CHARGING CANCELED')
            self.get_logger().info(f'   Battery: {result.final_percentage}%')
            self.get_logger().info(f'   Time: {result.charging_time:.1f}s')
        else:
            self.get_logger().error(f'❌ CHARGING FAILED (status: {status})')
        
        self.get_logger().info('=================================')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Called repeatedly during charging.
        Shows a nice progress bar!
        """
        feedback = feedback_msg.feedback
        
        # Create progress bar
        percent = feedback.current_percentage
        bar_length = 20
        filled = int(bar_length * percent / 100)
        bar = '█' * filled + '░' * (bar_length - filled)
        
        # Display progress
        self.get_logger().info(
            f'🔋 [{bar}] {percent}% | '
            f'⏱️  {feedback.time_remaining:.1f}s left | '
            f'⚡ {feedback.charging_rate:.1f}%/s'
        )


def main(args=None):
    rclpy.init(args=args)
    
    # Check command line arguments
    if len(sys.argv) != 2:
        print('Usage: ros2 run robot_actions battery_client <target_%>')
        print('Example: ros2 run robot_actions battery_client 80')
        return
    
    try:
        target = int(sys.argv[1])
        if target < 0 or target > 100:
            print('Error: Target must be between 0 and 100')
            return
    except ValueError:
        print('Error: Please provide a valid number')
        return
    
    # Create client
    client = BatteryClient()
    
    # Send charging request
    if not client.send_goal(target):
        return
    
    # Keep running until done
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('🛑 Interrupted by user')
    
    client.destroy_node()


if __name__ == '__main__':
    main()