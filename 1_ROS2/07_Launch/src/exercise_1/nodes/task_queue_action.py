#!/usr/bin/env python3
"""
Task Queue Action Server - Item Picking Action
Real-world use: Process warehouse orders with detailed progress tracking
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from ce_robot_interfaces.action import PickItems
import time
import random


class TaskQueueAction(Node):
    def __init__(self):
        super().__init__('task_queue_action')
        
        # Declare robot parameters
        self.declare_parameter('robot_id', 'AMR-WH-A-001')
        self.declare_parameter('robot_type', 'picker')
        self.declare_parameter('zone_id', 'WAREHOUSE-A-PICKING-ZONE')
        self.declare_parameter('max_items_per_trip', 50)
        self.declare_parameter('battery_level', 100.0)
        self.declare_parameter('picking_speed_items_per_min', 12.0)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.max_items = self.get_parameter('max_items_per_trip').value
        self.battery_level = self.get_parameter('battery_level').value
        self.picking_speed = self.get_parameter('picking_speed_items_per_min').value
        
        self.tasks_completed = 0
        self.total_items_picked = 0
        
        # Create action server
        self._action_server = ActionServer(
            self,
            PickItems,
            '/pick_items',
            self.execute_callback
        )
        
        self.get_logger().info('🟢 Item Picking Action Server READY')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Robot Type: {self.robot_type}')
        self.get_logger().info(f'Operating Zone: {self.zone_id}')
        self.get_logger().info(f'Max Items/Trip: {self.max_items}')
        self.get_logger().info(f'Battery Level: {self.battery_level:.1f}%')
        self.get_logger().info(f'Picking Speed: {self.picking_speed:.1f} items/min')
        self.get_logger().info('Action: /pick_items')
        self.get_logger().info('Purpose: Warehouse order fulfillment with detailed progress tracking')
        
    def execute_callback(self, goal_handle):
        """
        Process warehouse order with item picking
        Real-world: Pick items from shelves with detailed tracking
        
        Args:
            target_items: Total items to pick
            time_per_item: Time per item (seconds)
            order_id: Order tracking ID
            zone_id: Picking zone
            priority: Priority level
            max_weight_kg: Maximum weight limit
        
        Returns:
            items_picked: Items successfully picked
            actual_time_taken: Actual time taken
            battery_consumed: Battery consumed
            total_weight_kg: Total weight
            order_completed: Completion status
            completion_status: Status message
            items_damaged: Damaged items count
            items_missing: Missing items count
        """
        start_time = time.time()
        initial_battery = self.battery_level
        
        # Initialize variables
        items_to_pick = goal_handle.request.target_items
        items_damaged = 0
        items_missing = 0
        total_weight = 0.0
        
        # Calculate battery consumption per item (0.5% per item)
        battery_per_item = 0.5
        estimated_battery_use = items_to_pick * battery_per_item
        
        # Check battery level
        if self.battery_level < estimated_battery_use:
            self.get_logger().warn(
                f'⚠️ Insufficient battery!\n'
                f'   Order: {goal_handle.request.order_id}\n'
                f'   Current: {self.battery_level:.1f}%\n'
                f'   Required: {estimated_battery_use:.1f}%\n'
                f'   Status: Return to charging station'
            )
            goal_handle.abort()
            return PickItems.Result(
                items_picked=0,
                actual_time_taken=0.0,
                battery_consumed=0.0,
                total_weight_kg=0.0,
                order_completed=False,
                completion_status="FAILED",
                items_damaged=0,
                items_missing=0
            )
        
        # Check if request exceeds capacity
        if items_to_pick > self.max_items:
            self.get_logger().warn(
                f'⚠️ Order exceeds capacity!\n'
                f'   Requested: {items_to_pick} items\n'
                f'   Max capacity: {self.max_items} items\n'
                f'   Order: {goal_handle.request.order_id}'
            )
            goal_handle.abort()
            return PickItems.Result(
                items_picked=0,
                actual_time_taken=0.0,
                battery_consumed=0.0,
                total_weight_kg=0.0,
                order_completed=False,
                completion_status="FAILED",
                items_damaged=0,
                items_missing=0
            )
        
        estimated_time = items_to_pick * goal_handle.request.time_per_item
        target_zone = goal_handle.request.zone_id if goal_handle.request.zone_id else self.zone_id
        
        self.get_logger().info(
            f'🎯 New Order [{self.robot_id}]:\n'
            f'   Order ID: {goal_handle.request.order_id}\n'
            f'   Zone: {target_zone}\n'
            f'   Items to pick: {items_to_pick}\n'
            f'   Priority: {goal_handle.request.priority}\n'
            f'   Max weight: {goal_handle.request.max_weight_kg:.1f}kg\n'
            f'   Time per item: {goal_handle.request.time_per_item:.1f}s\n'
            f'   Estimated time: {estimated_time:.1f}s ({estimated_time/60:.1f} min)\n'
            f'   Battery: {self.battery_level:.1f}%'
        )
        
        # Item picking loop
        feedback_msg = PickItems.Feedback()
        shelf_locations = ['A1-B3', 'A2-B1', 'B3-C2', 'C1-D3', 'D2-E1', 'E3-F2']
        
        for i in range(1, items_to_pick + 1):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                elapsed = time.time() - start_time
                goal_handle.canceled()
                self.get_logger().warn(
                    f'❌ Order cancelled by supervisor\n'
                    f'   Order: {goal_handle.request.order_id}\n'
                    f'   Items picked: {i-1}/{items_to_pick}\n'
                    f'   Battery remaining: {self.battery_level:.1f}%'
                )
                return PickItems.Result(
                    items_picked=i-1,
                    actual_time_taken=elapsed,
                    battery_consumed=initial_battery - self.battery_level,
                    total_weight_kg=total_weight,
                    order_completed=False,
                    completion_status="CANCELLED",
                    items_damaged=items_damaged,
                    items_missing=items_missing
                )
            
            # Simulate battery consumption
            self.battery_level -= battery_per_item
            
            # Determine item details
            item_types = ['Box-A4', 'Envelope', 'Package-M', 'Pallet-S', 'Container', 'Crate-L']
            item_weights = [2.5, 0.3, 5.0, 15.0, 8.0, 12.0]  # kg
            item_type = item_types[i % len(item_types)]
            item_weight = item_weights[i % len(item_weights)]
            
            # Simulate occasional issues (5% chance)
            if random.random() < 0.05:
                if random.random() < 0.5:
                    items_damaged += 1
                    feedback_msg.status_message = f"Item damaged - skipping"
                    self.get_logger().warn(f'⚠️ Item {i} damaged - continuing to next item')
                else:
                    items_missing += 1
                    feedback_msg.status_message = f"Item not found - skipping"
                    self.get_logger().warn(f'⚠️ Item {i} missing from shelf - continuing')
            else:
                total_weight += item_weight
                feedback_msg.status_message = f"Picking {item_type}"
            
            # Check weight limit
            if total_weight > goal_handle.request.max_weight_kg:
                self.get_logger().error(
                    f'❌ Weight limit exceeded!\n'
                    f'   Current: {total_weight:.1f}kg\n'
                    f'   Limit: {goal_handle.request.max_weight_kg:.1f}kg\n'
                    f'   Stopping at item {i-1}/{items_to_pick}'
                )
                goal_handle.abort()
                return PickItems.Result(
                    items_picked=i-1,
                    actual_time_taken=time.time() - start_time,
                    battery_consumed=initial_battery - self.battery_level,
                    total_weight_kg=total_weight - item_weight,
                    order_completed=False,
                    completion_status="FAILED",
                    items_damaged=items_damaged,
                    items_missing=items_missing
                )
            
            # Update feedback
            feedback_msg.current_item = i
            feedback_msg.percentage_complete = (i / items_to_pick) * 100.0
            feedback_msg.current_item_type = item_type
            feedback_msg.current_item_weight = item_weight
            feedback_msg.current_location = shelf_locations[i % len(shelf_locations)]
            feedback_msg.battery_remaining = self.battery_level
            feedback_msg.elapsed_time = time.time() - start_time
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Log progress every 5 items
            if i % 5 == 0 or i == 1:
                self.get_logger().info(
                    f'📦 Item {i}/{items_to_pick} ({feedback_msg.percentage_complete:.1f}%) | '
                    f'{item_type} ({item_weight:.1f}kg) | '
                    f'Location: {feedback_msg.current_location} | '
                    f'Battery: {self.battery_level:.1f}%'
                )
            
            # Battery warning
            if self.battery_level < 20.0 and self.battery_level >= 19.5:
                self.get_logger().warn('⚠️ LOW BATTERY - Consider returning to charging station')
            
            # Simulate picking time
            time.sleep(goal_handle.request.time_per_item)
        
        # Order complete
        actual_time = time.time() - start_time
        battery_used = initial_battery - self.battery_level
        items_successfully_picked = items_to_pick - items_damaged - items_missing
        order_completed = (items_damaged + items_missing) == 0
        
        goal_handle.succeed()
        
        self.tasks_completed += 1
        self.total_items_picked += items_successfully_picked
        
        # Determine completion status
        if order_completed:
            status = "SUCCESS"
        elif items_successfully_picked >= (items_to_pick * 0.8):
            status = "PARTIAL"
        else:
            status = "FAILED"
        
        result = PickItems.Result()
        result.items_picked = items_successfully_picked
        result.actual_time_taken = actual_time
        result.battery_consumed = battery_used
        result.total_weight_kg = total_weight
        result.order_completed = order_completed
        result.completion_status = status
        result.items_damaged = items_damaged
        result.items_missing = items_missing
        
        self.get_logger().info(
            f'✅ Order Completed [{self.robot_id}]:\n'
            f'   Order ID: {goal_handle.request.order_id}\n'
            f'   Items picked: {result.items_picked}/{goal_handle.request.target_items}\n'
            f'   Items damaged: {items_damaged}\n'
            f'   Items missing: {items_missing}\n'
            f'   Total weight: {total_weight:.1f}kg\n'
            f'   Time taken: {actual_time:.1f}s ({actual_time/60:.1f} min)\n'
            f'   Battery consumed: {battery_used:.1f}%\n'
            f'   Battery remaining: {self.battery_level:.1f}%\n'
            f'   Completion: {status}\n'
            f'   Total orders today: {self.tasks_completed}\n'
            f'   Total items today: {self.total_items_picked}\n'
            f'   Status: {"Ready for next order" if self.battery_level > 20 else "⚠️ LOW BATTERY - Charging recommended"}'
        )
        
        return result


def main(args=None):
    rclpy.init(args=args)
    
    task_queue_action = TaskQueueAction()
    
    try:
        rclpy.spin(task_queue_action)
    except KeyboardInterrupt:
        pass
    finally:
        task_queue_action.get_logger().info('🔴 Task Queue Action Server shutting down')
        task_queue_action.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
