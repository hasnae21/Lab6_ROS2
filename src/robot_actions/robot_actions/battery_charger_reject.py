#!/usr/bin/env python3
"""
Battery Charging Action Server - REJECT STRATEGY
=================================================
Rejects new goals when busy.

Real-world example: Single-user bathroom
- Person 1 enters and locks door
- Person 2 tries to enter: REJECTED (door locked)
- Person 2 must wait until Person 1 finishes
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from custom_interfaces.action import ChargeBattery


class BatteryChargerReject(Node):
    """
    Charging station with REJECT strategy.
    Only one goal at a time - others are rejected.
    """

    def __init__(self):
        super().__init__('battery_charger_reject')
        
        self.current_battery = 20
        self.charge_rate = 5.0
        self.is_busy = False
        
        self.get_logger().info('=================================')
        self.get_logger().info('🔋 Charging Station (REJECT MODE)')
        self.get_logger().info(f'   Current battery: {self.current_battery}%')
        self.get_logger().info(f'   Charge rate: {self.charge_rate}%/sec')
        self.get_logger().info('   🚫 Rejects new goals when busy')
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
        Reject if busy, accept if free.
        """
        target = goal_request.target_percentage
        
        self.get_logger().info(f'🔥 New charging request: {target}%')
        
        # Check if busy FIRST
        if self.is_busy:
            self.get_logger().warn('🚫 REJECTED - Already charging! Try again later.')
            return GoalResponse.REJECT
        
        # Validation
        if target < 0 or target > 100:
            self.get_logger().warn(f'❌ Invalid target: {target}%')
            return GoalResponse.REJECT
        
        if self.current_battery >= target:
            self.get_logger().warn(
                f'❌ Already at {self.current_battery}% (target: {target}%)'
            )
            return GoalResponse.REJECT
        
        self.get_logger().info('✅ Goal accepted - Starting charge!')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow manual cancellation."""
        self.get_logger().info('🛑 Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the charging goal."""
        # Mark as busy
        self.is_busy = True
        target = goal_handle.request.target_percentage
        
        self.get_logger().info(f'⚡ Starting charge to {target}%')
        self.get_logger().info('🔒 Station locked - rejecting new requests')
        
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
                
                self.is_busy = False
                self.get_logger().info('🛑 Charging canceled!')
                self.get_logger().info('🔓 Station unlocked')
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
        
        # Mark as not busy
        self.is_busy = False
        
        self.get_logger().info('=================================')
        self.get_logger().info('✅ Charging complete!')
        self.get_logger().info(f'   Final: {result.final_percentage}%')
        self.get_logger().info(f'   Time: {result.charging_time:.1f}s')
        self.get_logger().info('🔓 Station unlocked')
        self.get_logger().info('=================================')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    charger = BatteryChargerReject()
    
    try:
        rclpy.spin(charger)
    except KeyboardInterrupt:
        charger.get_logger().info('Shutting down...')
    
    charger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
