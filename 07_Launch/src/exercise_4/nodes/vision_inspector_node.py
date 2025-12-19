#!/usr/bin/env python3
"""
Exercise 4: Vision Inspector Node
Real-world scenario: Manufacturing quality control with camera-based defect detection
Criticality: CRITICAL - Defective products must not pass inspection
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
from datetime import datetime


class VisionInspectorNode(Node):
    """
    Vision-based inspection system for quality control
    Simulates camera-based defect detection in manufacturing
    CRITICAL: Missing defects could lead to product recalls
    """
    
    def __init__(self):
        super().__init__('vision_inspector')
        
        # Parameters
        self.declare_parameter('robot_id', 'QC-VISION-INSPECTOR-001')
        self.declare_parameter('robot_type', 'inspection')
        self.declare_parameter('zone_id', 'FACTORY-QC-STATION-1')
        self.declare_parameter('camera_resolution', '1920x1080')
        self.declare_parameter('inspection_rate_hz', 1.0)
        self.declare_parameter('defect_threshold', 0.85)  # Confidence threshold
        self.declare_parameter('simulate_failure', False)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.camera_resolution = self.get_parameter('camera_resolution').value
        self.inspection_rate = self.get_parameter('inspection_rate_hz').value
        self.defect_threshold = self.get_parameter('defect_threshold').value
        self.simulate_failure = self.get_parameter('simulate_failure').value
        
        # State variables
        self.items_inspected = 0
        self.defects_found = 0
        self.false_positives = 0
        self.inspection_accuracy = 98.5  # Start at 98.5%
        self.camera_health = 100.0
        self.lighting_quality = 95.0
        self.focus_quality = 100.0
        
        # Defect types
        self.defect_types = [
            'SCRATCH', 'DENT', 'COLOR_MISMATCH', 
            'DIMENSION_ERROR', 'SURFACE_DEFECT', 'MISSING_COMPONENT'
        ]
        
        # Publisher
        self.status_publisher = self.create_publisher(
            String,
            '/vision_status',
            10
        )
        
        # Timer for inspection cycle
        self.timer = self.create_timer(
            1.0 / self.inspection_rate,
            self.inspection_cycle
        )
        
        # Failure simulation counter
        self.cycle_count = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🎥 Vision Inspector Node Started')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Zone: {self.zone_id}')
        self.get_logger().info(f'Camera: {self.camera_resolution}')
        self.get_logger().info(f'Defect Threshold: {self.defect_threshold:.2f}')
        self.get_logger().info(f'Inspection Rate: {self.inspection_rate} Hz')
        self.get_logger().info('=' * 60)
    
    def inspection_cycle(self):
        """Execute one inspection cycle"""
        self.cycle_count += 1
        
        # Simulate failure after 15 cycles if enabled
        if self.simulate_failure and self.cycle_count > 15:
            self.get_logger().fatal('❌ CRITICAL: Camera hardware failure detected!')
            self.get_logger().fatal('🚨 Vision system offline - stopping inspection')
            raise SystemExit('Simulated camera failure')
        
        # Simulate inspection
        self.items_inspected += 1
        
        # Randomly detect defects (10% defect rate)
        has_defect = random.random() < 0.10
        confidence = random.uniform(0.75, 0.99)
        
        if has_defect and confidence >= self.defect_threshold:
            self.defects_found += 1
            defect_type = random.choice(self.defect_types)
            self.get_logger().warn(
                f'⚠️ DEFECT DETECTED: {defect_type} '
                f'(Confidence: {confidence:.2%})'
            )
        
        # Simulate gradual camera degradation
        if self.cycle_count % 5 == 0:
            self.camera_health = max(80.0, self.camera_health - 0.5)
            self.lighting_quality = 90.0 + random.uniform(-5, 5)
            self.focus_quality = max(85.0, self.focus_quality - 0.3)
            
            # Accuracy degrades with camera health
            self.inspection_accuracy = min(
                98.5,
                95.0 + (self.camera_health - 80.0) / 20.0 * 3.5
            )
        
        # Publish status
        self.publish_status(has_defect, confidence if has_defect else 0.0)
    
    def publish_status(self, defect_detected: bool, confidence: float):
        """Publish inspection status"""
        status_data = {
            'node_type': 'vision_inspector',
            'robot_id': self.robot_id,
            'zone_id': self.zone_id,
            'timestamp': datetime.now().isoformat(),
            'inspection_metrics': {
                'items_inspected': self.items_inspected,
                'defects_found': self.defects_found,
                'defect_rate': f'{(self.defects_found / max(1, self.items_inspected)) * 100:.2f}%',
                'inspection_accuracy': f'{self.inspection_accuracy:.2f}%',
                'false_positive_rate': f'{(self.false_positives / max(1, self.defects_found)) * 100:.1f}%'
            },
            'current_inspection': {
                'defect_detected': defect_detected,
                'confidence': f'{confidence:.2%}',
                'pass_fail': 'FAIL' if defect_detected else 'PASS'
            },
            'camera_health': {
                'overall_health': f'{self.camera_health:.1f}%',
                'lighting_quality': f'{self.lighting_quality:.1f}%',
                'focus_quality': f'{self.focus_quality:.1f}%',
                'resolution': self.camera_resolution
            },
            'alerts': self.get_alerts()
        }
        
        msg = String()
        msg.data = json.dumps(status_data, indent=2)
        self.status_publisher.publish(msg)
    
    def get_alerts(self) -> list:
        """Generate alerts based on system health"""
        alerts = []
        
        if self.camera_health < 85.0:
            alerts.append('CAMERA_DEGRADATION: Camera maintenance required')
        
        if self.lighting_quality < 85.0:
            alerts.append('LIGHTING_ISSUE: Check lighting system')
        
        if self.focus_quality < 90.0:
            alerts.append('FOCUS_WARNING: Recalibrate camera focus')
        
        if self.inspection_accuracy < 96.0:
            alerts.append('ACCURACY_WARNING: Inspection accuracy below threshold')
        
        if not alerts:
            alerts.append('ALL_SYSTEMS_NOMINAL')
        
        return alerts


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VisionInspectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        print(f'Vision Inspector Node: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
