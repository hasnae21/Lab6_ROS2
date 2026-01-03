#!/usr/bin/env python3
"""
Battery Charging Action Server - QUEUE STRATEGY
================================================
Multiple goals are queued and executed in order.

Real-world example: Coffee shop queue
- Customer 1 orders coffee (charging to 60%)
- Customer 2 orders coffee (charging to 80%)
- Customer 1's coffee finishes first
- Then Customer 2's coffee is made
"""

import time
from collections import deque
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from custom_interfaces.action import ChargeBattery


class BatteryChargerQueue(Node):
    """
    Charging station with QUEUE strategy.
    Multiple charging requests are handled in order.
    """

    def __init__(self):
        super().__init__('battery_charger_queue')
        
        # Battery state
        self.current_battery = 20
        self.charge_rate = 5.0
        
        # Queue for pending goals
        self.goal_queue = deque()
        self.current_goal_handle = None
        self.is_executing = False
        
        self.get_logger().info('=================================')
        self.get_logger().info('🔋 Charging Station (QUEUE MODE)')
        self.get_logger().info(f'   Current battery: {self.current_battery}%')
        self.get_logger().info(f'   Charge rate: {self.charge_rate}%/sec')
        self.get_logger().info('   📋 Multiple goals will be queued')
        self.get_logger().info('=================================')

        # Create action server with reentrant callback group
        # This allows multiple goals to be accepted while one is executing
        callback_group = ReentrantCallbackGroup()
        
        self.action_server = ActionServer(
            self,
            ChargeBattery,
            'charge_battery',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group
        )

    def goal_callback(self, goal_request):
        """
        Accept all valid goals and queue them.
        """
        target = goal_request.target_percentage
        
        self.get_logger().info(f'🔥 New charging request: {target}%')
        
        # Validation
        if target < 0 or target > 100:
            self.get_logger().warn(f'❌ Invalid target: {target}%')
            return GoalResponse.REJECT
        
        if self.current_battery >= target:
            self.get_logger().warn(
                f'❌ Already at {self.current_battery}% (target: {target}%)'
            )
            return GoalResponse.REJECT
        
        # Count queued goals
        queue_position = len(self.goal_queue)
        if self.is_executing:
            queue_position += 1
        
        if queue_position > 0:
            self.get_logger().info(
                f'📋 Goal queued (position {queue_position + 1} in queue)'
            )
        else:
            self.get_logger().info('✅ Goal accepted - Starting immediately!')
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Allow cancellation of any goal (executing or queued).
        """
        self.get_logger().info('🛑 Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute goals in queue order.
        """
        # Add to queue
        self.goal_queue.append(goal_handle)
        
        # If already executing, this goal will wait
        if self.is_executing:
            # Wait in queue
            while len(self.goal_queue) > 0 and self.goal_queue[0] != goal_handle:
                if goal_handle.is_cancel_requested:
                    # Canceled while waiting in queue
                    self.goal_queue.remove(goal_handle)
                    goal_handle.canceled()
                    
                    result = ChargeBattery.Result()
                    result.success = False
                    result.final_percentage = self.current_battery
                    result.charging_time = 0.0
                    
                    self.get_logger().info('🛑 Canceled while in queue')
                    return result
                
                time.sleep(0.5)
            
            # Check if we were canceled or removed while waiting
            if goal_handle not in self.goal_queue:
                result = ChargeBattery.Result()
                result.success = False
                result.final_percentage = self.current_battery
                result.charging_time = 0.0
                return result
        
        # Now it's our turn - remove from queue and execute
        self.is_executing = True
        self.current_goal_handle = goal_handle
        if goal_handle in self.goal_queue:
            self.goal_queue.remove(goal_handle)
        
        self.get_logger().info(f'⚡ Starting charge to {goal_handle.request.target_percentage}%')
        if len(self.goal_queue) > 0:
            self.get_logger().info(f'   ({len(self.goal_queue)} goal(s) waiting)')
        
        # Execute charging logic
        result = self._charge_battery(goal_handle)
        
        # Done executing
        self.is_executing = False
        self.current_goal_handle = None
        
        return result

    def _charge_battery(self, goal_handle):
        """
        The actual charging logic (extracted for clarity).
        """
        target = goal_handle.request.target_percentage
        start_time = time.time()
        feedback_msg = ChargeBattery.Feedback()
        
        # Charging loop
        while self.current_battery < target:
            # Check cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                
                result = ChargeBattery.Result()
                result.success = False
                result.final_percentage = self.current_battery
                result.charging_time = time.time() - start_time
                
                self.get_logger().info('🛑 Charging canceled!')
                return result
            
            # Charge
            charge_amount = self.charge_rate * 0.5
            self.current_battery = min(self.current_battery + charge_amount, target)
            
            # Feedback
            remaining_percent = target - self.current_battery
            time_remaining = remaining_percent / self.charge_rate
            
            feedback_msg.current_percentage = int(self.current_battery)
            feedback_msg.time_remaining = time_remaining
            feedback_msg.charging_rate = self.charge_rate
            
            self.get_logger().info(
                f'🔋 {feedback_msg.current_percentage}% '
                f'(~{time_remaining:.1f}s left)'
            )
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.5)
        
        # Success!
        goal_handle.succeed()
        
        total_time = time.time() - start_time
        result = ChargeBattery.Result()
        result.success = True
        result.final_percentage = int(self.current_battery)
        result.charging_time = total_time
        
        self.get_logger().info('=================================')
        self.get_logger().info('✅ Charging complete!')
        self.get_logger().info(f'   Final: {result.final_percentage}%')
        self.get_logger().info(f'   Time: {result.charging_time:.1f}s')
        self.get_logger().info('=================================')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    
    charger = BatteryChargerQueue()
    
    # Use MultiThreadedExecutor for concurrent goal acceptance
    executor = MultiThreadedExecutor()
    executor.add_node(charger)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        charger.get_logger().info('Shutting down...')
    
    charger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()