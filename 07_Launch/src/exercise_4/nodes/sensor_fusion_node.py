#!/usr/bin/env python3
"""
Exercise 4: Sensor Fusion Node
Real-world scenario: Multi-sensor data fusion for comprehensive quality assessment
Criticality: HIGH - Data fusion improves inspection accuracy significantly
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
from datetime import datetime
import math


class SensorFusionNode(Node):
    """
    Multi-sensor fusion system for enhanced quality control
    Combines vision, thermal, ultrasonic, and weight sensors
    HIGH: Improves accuracy but not safety-critical
    """
    
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Parameters
        self.declare_parameter('robot_id', 'QC-SENSOR-FUSION-001')
        self.declare_parameter('robot_type', 'inspection')
        self.declare_parameter('zone_id', 'FACTORY-QC-STATION-1')
        self.declare_parameter('fusion_rate_hz', 2.0)
        self.declare_parameter('sensor_count', 4)
        self.declare_parameter('simulate_failure', False)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.fusion_rate = self.get_parameter('fusion_rate_hz').value
        self.sensor_count = self.get_parameter('sensor_count').value
        self.simulate_failure = self.get_parameter('simulate_failure').value
        
        # Sensor states
        self.sensors = {
            'thermal_camera': {'health': 100.0, 'readings': 0, 'temp_range': (20, 80)},
            'ultrasonic': {'health': 100.0, 'readings': 0, 'distance_cm': 0},
            'weight_sensor': {'health': 100.0, 'readings': 0, 'weight_g': 0},
            'laser_scanner': {'health': 100.0, 'readings': 0, 'scan_points': 0}
        }
        
        # Fusion statistics
        self.fusion_cycles = 0
        self.data_quality_score = 100.0
        self.sensor_agreement = 100.0
        self.anomalies_detected = 0
        
        # Publisher
        self.status_publisher = self.create_publisher(
            String,
            '/fusion_status',
            10
        )
        
        # Timer for fusion cycle
        self.timer = self.create_timer(
            1.0 / self.fusion_rate,
            self.fusion_cycle
        )
        
        # Failure simulation counter
        self.cycle_count = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🔬 Sensor Fusion Node Started')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Zone: {self.zone_id}')
        self.get_logger().info(f'Active Sensors: {self.sensor_count}')
        self.get_logger().info(f'Fusion Rate: {self.fusion_rate} Hz')
        self.get_logger().info('=' * 60)
    
    def fusion_cycle(self):
        """Execute sensor fusion cycle"""
        self.cycle_count += 1
        self.fusion_cycles += 1
        
        # Simulate failure after 20 cycles if enabled
        if self.simulate_failure and self.cycle_count > 20:
            self.get_logger().error('❌ Sensor fusion processor overheating!')
            self.get_logger().error('⚠️ Data synchronization lost')
            raise SystemExit('Simulated fusion processor failure')
        
        # Read all sensors
        sensor_data = {}
        for sensor_name, sensor in self.sensors.items():
            reading = self.read_sensor(sensor_name, sensor)
            sensor_data[sensor_name] = reading
            sensor['readings'] += 1
        
        # Perform fusion analysis
        fusion_result = self.analyze_fusion(sensor_data)
        
        # Detect anomalies
        if fusion_result['anomaly_detected']:
            self.anomalies_detected += 1
            self.get_logger().warn(
                f'⚠️ Anomaly detected: {fusion_result["anomaly_type"]}'
            )
        
        # Degrade sensor health gradually
        if self.cycle_count % 10 == 0:
            for sensor in self.sensors.values():
                sensor['health'] = max(85.0, sensor['health'] - random.uniform(0.3, 0.8))
            
            # Update data quality based on sensor health
            avg_health = sum(s['health'] for s in self.sensors.values()) / len(self.sensors)
            self.data_quality_score = min(100.0, avg_health + random.uniform(-2, 2))
            self.sensor_agreement = 90.0 + random.uniform(-5, 5)
        
        # Publish status
        self.publish_status(sensor_data, fusion_result)
    
    def read_sensor(self, sensor_name: str, sensor: dict) -> dict:
        """Simulate sensor reading"""
        if sensor_name == 'thermal_camera':
            temp_min, temp_max = sensor['temp_range']
            return {
                'temperature_c': round(random.uniform(temp_min, temp_max), 1),
                'hotspot_detected': random.random() < 0.15,
                'thermal_uniformity': round(random.uniform(85, 100), 1)
            }
        
        elif sensor_name == 'ultrasonic':
            return {
                'distance_cm': round(random.uniform(5, 50), 2),
                'surface_quality': random.choice(['smooth', 'rough', 'irregular']),
                'echo_strength': round(random.uniform(70, 100), 1)
            }
        
        elif sensor_name == 'weight_sensor':
            expected_weight = 250.0  # grams
            actual = expected_weight + random.uniform(-10, 10)
            return {
                'weight_g': round(actual, 2),
                'weight_deviation_g': round(actual - expected_weight, 2),
                'stability': round(random.uniform(95, 100), 1)
            }
        
        elif sensor_name == 'laser_scanner':
            return {
                'scan_points': random.randint(8000, 12000),
                'dimensional_accuracy_mm': round(random.uniform(0.01, 0.1), 3),
                'scan_completeness': round(random.uniform(95, 100), 1)
            }
        
        return {}
    
    def analyze_fusion(self, sensor_data: dict) -> dict:
        """Analyze fused sensor data"""
        # Check for anomalies across sensors
        anomaly_detected = False
        anomaly_type = 'NONE'
        
        # Thermal anomaly
        if sensor_data['thermal_camera']['hotspot_detected']:
            anomaly_detected = True
            anomaly_type = 'THERMAL_HOTSPOT'
        
        # Weight anomaly
        if abs(sensor_data['weight_sensor']['weight_deviation_g']) > 5.0:
            anomaly_detected = True
            anomaly_type = 'WEIGHT_DEVIATION'
        
        # Dimensional anomaly
        if sensor_data['laser_scanner']['dimensional_accuracy_mm'] > 0.08:
            anomaly_detected = True
            anomaly_type = 'DIMENSIONAL_ERROR'
        
        # Calculate confidence
        confidence = (
            sensor_data['thermal_camera']['thermal_uniformity'] * 0.25 +
            sensor_data['ultrasonic']['echo_strength'] * 0.20 +
            sensor_data['weight_sensor']['stability'] * 0.30 +
            sensor_data['laser_scanner']['scan_completeness'] * 0.25
        )
        
        return {
            'anomaly_detected': anomaly_detected,
            'anomaly_type': anomaly_type,
            'fusion_confidence': round(confidence, 1),
            'sensor_agreement': self.sensor_agreement
        }
    
    def publish_status(self, sensor_data: dict, fusion_result: dict):
        """Publish fusion status"""
        status_data = {
            'node_type': 'sensor_fusion',
            'robot_id': self.robot_id,
            'zone_id': self.zone_id,
            'timestamp': datetime.now().isoformat(),
            'fusion_metrics': {
                'fusion_cycles': self.fusion_cycles,
                'data_quality_score': f'{self.data_quality_score:.1f}%',
                'sensor_agreement': f'{self.sensor_agreement:.1f}%',
                'anomalies_detected': self.anomalies_detected
            },
            'sensor_health': {
                name: f"{data['health']:.1f}%"
                for name, data in self.sensors.items()
            },
            'current_readings': sensor_data,
            'fusion_result': fusion_result,
            'alerts': self.get_alerts()
        }
        
        msg = String()
        msg.data = json.dumps(status_data, indent=2)
        self.status_publisher.publish(msg)
    
    def get_alerts(self) -> list:
        """Generate alerts based on sensor health"""
        alerts = []
        
        for name, sensor in self.sensors.items():
            if sensor['health'] < 90.0:
                alerts.append(f'{name.upper()}_DEGRADATION: Calibration recommended')
        
        if self.data_quality_score < 95.0:
            alerts.append('DATA_QUALITY_LOW: Check sensor calibration')
        
        if self.sensor_agreement < 85.0:
            alerts.append('SENSOR_DISAGREEMENT: Verify sensor alignment')
        
        if not alerts:
            alerts.append('ALL_SENSORS_NOMINAL')
        
        return alerts


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SensorFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        print(f'Sensor Fusion Node: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
