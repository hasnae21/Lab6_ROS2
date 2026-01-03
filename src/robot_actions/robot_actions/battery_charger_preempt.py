#!/usr/bin/env python3
"""
Battery Charging Action Server - PREEMPT STRATEGY
==================================================
New goals abort the current goal immediately.

Real-world example: Emergency room
- Patient 1 is being treated (broken arm)
- Patient 2 arrives (heart attack)
- Treatment switches immediately to Patient 2
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from custom_interfaces.action import ChargeBattery


class BatteryChargerPreempt(Node):
    """
    Charging station with PREEMPT strategy.
    New goals abort the current goal.
    """

    def __init__(self):
        super().__init__('battery_charger_preempt')
        
        self.current_battery = 20
        self.charge_rate = 5.0
        self.current_goal_handle = None
        
        self.get_logger().info('=================================')
        self.get_logger().info('🔋 Charging Station (PREEMPT MODE)')
        self.get_logger().info(f'   Current battery: {self.current_battery}%')
        self.get_logger().info(f'   Charge rate: {self.charge_rate}%/sec')
        self.get_logger().info('   ⚡ New goals abort current goal')
        self.get_logger().info('=================================')

        self.action_server = ActionServer(
            self,
            ChargeBattery,
            'charge_battery',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """
        Accept new goal and abort current one.
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
        
        # Preemption: Abort current goal if exists
        if self.current_goal_handle is not None and self.current_goal_handle.is_active:
            old_target = self.current_goal_handle.request.target_percentage
            self.get_logger().warn(
                f'⚠️ PREEMPTING current goal (was charging to {old_target}%)'
            )
            # Abort the old goal
            self.current_goal_handle.abort()
            self.current_goal_handle = None
        
        self.get_logger().info('✅ Goal accepted - Starting immediately!')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow manual cancellation."""
        self.get_logger().info('🛑 Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the charging goal."""
        self.current_goal_handle = goal_handle
        target = goal_handle.request.target_percentage
        
        self.get_logger().info(f'⚡ Starting charge to {target}%')
        
        start_time = time.time()
        feedback_msg = ChargeBattery.Feedback()
        
        # Charging loop
        while self.current_battery < target:
            # Check if aborted or canceled
            if not goal_handle.is_active:
                # Goal was aborted (preempted by new goal)
                result = ChargeBattery.Result()
                result.success = False
                result.final_percentage = self.current_battery
                result.charging_time = time.time() - start_time
                
                self.get_logger().warn('⚠️ Goal was aborted (preempted)')
                return result
            
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
        
        self.current_goal_handle = None
        return result


def main(args=None):
    rclpy.init(args=args)
    charger = BatteryChargerPreempt()
    
    try:
        rclpy.spin(charger)
    except KeyboardInterrupt:
        charger.get_logger().info('Shutting down...')
    
    charger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
