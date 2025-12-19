#!/usr/bin/env python3
"""
Exercise 4: Report Generator Node
Real-world scenario: Quality control report generation and data logging
Criticality: NON-CRITICAL - Reports are important but not safety-critical
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
from datetime import datetime, timedelta


class ReportGeneratorNode(Node):
    """
    Automated report generation for quality control data
    Creates inspection summaries, trend analysis, and compliance reports
    NON-CRITICAL: Can continue operation without reports temporarily
    """
    
    def __init__(self):
        super().__init__('report_generator')
        
        # Parameters
        self.declare_parameter('robot_id', 'QC-REPORT-GENERATOR-001')
        self.declare_parameter('robot_type', 'inspection')
        self.declare_parameter('zone_id', 'FACTORY-QC-STATION-1')
        self.declare_parameter('report_rate_hz', 0.5)
        self.declare_parameter('report_format', 'JSON')
        self.declare_parameter('simulate_failure', False)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.report_rate = self.get_parameter('report_rate_hz').value
        self.report_format = self.get_parameter('report_format').value
        self.simulate_failure = self.get_parameter('simulate_failure').value
        
        # Report statistics
        self.reports_generated = 0
        self.total_items_logged = 0
        self.defective_items_logged = 0
        self.report_queue_size = 0
        self.storage_used_mb = 0.0
        self.database_health = 100.0
        
        # Report types
        self.report_types = [
            'HOURLY_SUMMARY',
            'SHIFT_REPORT',
            'DEFECT_ANALYSIS',
            'TREND_REPORT',
            'COMPLIANCE_AUDIT'
        ]
        
        # Publisher
        self.status_publisher = self.create_publisher(
            String,
            '/report_status',
            10
        )
        
        # Timer for report generation
        self.timer = self.create_timer(
            1.0 / self.report_rate,
            self.generate_report
        )
        
        # Failure simulation counter
        self.cycle_count = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📊 Report Generator Node Started')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Zone: {self.zone_id}')
        self.get_logger().info(f'Report Format: {self.report_format}')
        self.get_logger().info(f'Report Rate: {self.report_rate} Hz')
        self.get_logger().info('=' * 60)
    
    def generate_report(self):
        """Generate quality control report"""
        self.cycle_count += 1
        
        # Simulate failure after 25 cycles if enabled
        if self.simulate_failure and self.cycle_count > 25:
            self.get_logger().error('❌ Database connection lost!')
            self.get_logger().error('⚠️ Report generation suspended')
            raise SystemExit('Simulated database connection failure')
        
        # Generate report
        self.reports_generated += 1
        report_type = random.choice(self.report_types)
        
        # Simulate data collection
        items_in_report = random.randint(10, 50)
        defects_in_report = int(items_in_report * random.uniform(0.05, 0.15))
        
        self.total_items_logged += items_in_report
        self.defective_items_logged += defects_in_report
        
        # Simulate storage growth
        self.storage_used_mb += random.uniform(0.5, 2.0)
        self.report_queue_size = random.randint(0, 10)
        
        # Database health degrades slowly
        if self.cycle_count % 5 == 0:
            self.database_health = max(85.0, self.database_health - 0.5)
        
        # Log report generation
        self.get_logger().info(
            f'📄 Report generated: {report_type} '
            f'({items_in_report} items, {defects_in_report} defects)'
        )
        
        # Create detailed report data
        report_data = self.create_report_data(report_type, items_in_report, defects_in_report)
        
        # Publish status
        self.publish_status(report_data)
    
    def create_report_data(self, report_type: str, items: int, defects: int) -> dict:
        """Create detailed report data"""
        return {
            'report_id': f'RPT-{self.reports_generated:06d}',
            'report_type': report_type,
            'timestamp': datetime.now().isoformat(),
            'period': {
                'start': (datetime.now() - timedelta(hours=1)).isoformat(),
                'end': datetime.now().isoformat(),
                'duration_minutes': 60
            },
            'statistics': {
                'items_inspected': items,
                'defects_found': defects,
                'defect_rate': f'{(defects / items * 100):.2f}%',
                'pass_rate': f'{((items - defects) / items * 100):.2f}%'
            },
            'defect_breakdown': self.get_defect_breakdown(defects),
            'trend_analysis': self.get_trend_analysis(),
            'recommendations': self.get_recommendations(defects / items)
        }
    
    def get_defect_breakdown(self, total_defects: int) -> dict:
        """Generate defect breakdown"""
        defect_types = {
            'SCRATCH': int(total_defects * 0.30),
            'DENT': int(total_defects * 0.20),
            'COLOR_MISMATCH': int(total_defects * 0.15),
            'DIMENSION_ERROR': int(total_defects * 0.20),
            'SURFACE_DEFECT': int(total_defects * 0.10),
            'MISSING_COMPONENT': int(total_defects * 0.05)
        }
        return defect_types
    
    def get_trend_analysis(self) -> dict:
        """Generate trend analysis"""
        return {
            'defect_rate_trend': random.choice(['INCREASING', 'STABLE', 'DECREASING']),
            'quality_score_change': f'{random.uniform(-2.5, 2.5):+.2f}%',
            'production_efficiency': f'{random.uniform(85, 98):.1f}%',
            'predicted_maintenance': f'{random.randint(3, 14)} days'
        }
    
    def get_recommendations(self, defect_rate: float) -> list:
        """Generate recommendations based on defect rate"""
        recommendations = []
        
        if defect_rate > 0.12:
            recommendations.append('HIGH_DEFECT_RATE: Review production parameters')
        
        if self.database_health < 90.0:
            recommendations.append('DATABASE_MAINTENANCE: Schedule database optimization')
        
        if self.storage_used_mb > 500:
            recommendations.append('STORAGE_MANAGEMENT: Archive old reports')
        
        if self.report_queue_size > 5:
            recommendations.append('QUEUE_BACKLOG: Increase processing capacity')
        
        if not recommendations:
            recommendations.append('OPERATIONS_NORMAL: No action required')
        
        return recommendations
    
    def publish_status(self, report_data: dict):
        """Publish report generator status"""
        status_data = {
            'node_type': 'report_generator',
            'robot_id': self.robot_id,
            'zone_id': self.zone_id,
            'timestamp': datetime.now().isoformat(),
            'generator_metrics': {
                'reports_generated': self.reports_generated,
                'total_items_logged': self.total_items_logged,
                'defective_items_logged': self.defective_items_logged,
                'overall_defect_rate': f'{(self.defective_items_logged / max(1, self.total_items_logged) * 100):.2f}%'
            },
            'system_health': {
                'database_health': f'{self.database_health:.1f}%',
                'storage_used_mb': f'{self.storage_used_mb:.2f}',
                'report_queue_size': self.report_queue_size,
                'report_format': self.report_format
            },
            'latest_report': report_data,
            'alerts': self.get_alerts()
        }
        
        msg = String()
        msg.data = json.dumps(status_data, indent=2)
        self.status_publisher.publish(msg)
    
    def get_alerts(self) -> list:
        """Generate alerts based on system health"""
        alerts = []
        
        if self.database_health < 90.0:
            alerts.append('DATABASE_DEGRADATION: Performance optimization needed')
        
        if self.storage_used_mb > 800:
            alerts.append('STORAGE_WARNING: High storage usage')
        
        if self.report_queue_size > 8:
            alerts.append('QUEUE_WARNING: Report backlog building')
        
        defect_rate = self.defective_items_logged / max(1, self.total_items_logged)
        if defect_rate > 0.15:
            alerts.append('HIGH_DEFECT_RATE: Quality investigation required')
        
        if not alerts:
            alerts.append('ALL_SYSTEMS_NOMINAL')
        
        return alerts


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ReportGeneratorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        print(f'Report Generator Node: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
