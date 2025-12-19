#!/usr/bin/env python3
"""
Production Failure Counter
Tracks node failures with persistent storage for maintenance and analytics
Used in 24/7 warehouse robot deployments
"""

import json
import os
from datetime import datetime
from pathlib import Path


class FailureCounter:
    """
    Production-grade failure tracking system
    - Persistent JSON storage
    - Per-node failure counts
    - Timestamp tracking
    - Robot ID association
    - Maintenance logging
    """
    
    def __init__(self, storage_dir='/tmp/robot_failures'):
        """
        Initialize failure counter with persistent storage
        
        Args:
            storage_dir: Directory for failure data (default: /tmp/robot_failures)
        """
        self.storage_dir = Path(storage_dir)
        self.storage_dir.mkdir(parents=True, exist_ok=True)
        
        self.failure_file = self.storage_dir / 'failure_counts.json'
        self.log_file = self.storage_dir / 'failure_log.txt'
        
        # Load existing failures or initialize
        if self.failure_file.exists():
            with open(self.failure_file, 'r') as f:
                self.failures = json.load(f)
        else:
            self.failures = {}
        
        # Get robot ID from environment or use default
        self.robot_id = os.environ.get('ROBOT_ID', 'UNKNOWN')
        
        print(f"📊 Failure Counter initialized")
        print(f"   Storage: {self.storage_dir}")
        print(f"   Robot ID: {self.robot_id}")
    
    def get_failure_count(self, node_name):
        """
        Get current failure count for a node
        
        Args:
            node_name: Name of the node
            
        Returns:
            int: Number of failures
        """
        return self.failures.get(node_name, {}).get('count', 0)
    
    def increment_failure_count(self, node_name):
        """
        Increment failure count for a node and save to disk
        
        Args:
            node_name: Name of the node that failed
            
        Returns:
            int: New failure count
        """
        if node_name not in self.failures:
            self.failures[node_name] = {
                'count': 0,
                'first_failure': datetime.now().isoformat(),
                'last_failure': None,
                'robot_id': self.robot_id
            }
        
        self.failures[node_name]['count'] += 1
        self.failures[node_name]['last_failure'] = datetime.now().isoformat()
        self.failures[node_name]['robot_id'] = self.robot_id
        
        # Save to disk
        self._save_failures()
        
        count = self.failures[node_name]['count']
        print(f"⚠️ Failure count for {node_name}: {count}")
        
        return count
    
    def reset_failure_count(self, node_name):
        """
        Reset failure count for a node (e.g., after successful recovery)
        
        Args:
            node_name: Name of the node
        """
        if node_name in self.failures:
            self.failures[node_name]['count'] = 0
            self.failures[node_name]['last_reset'] = datetime.now().isoformat()
            self._save_failures()
            print(f"✅ Failure count reset for {node_name}")
    
    def log_failure(self, node_name, error_message):
        """
        Log failure details to maintenance log file
        
        Args:
            node_name: Name of the failed node
            error_message: Description of the failure
        """
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        count = self.get_failure_count(node_name)
        
        log_entry = (
            f"[{timestamp}] ROBOT:{self.robot_id} | "
            f"NODE:{node_name} | "
            f"COUNT:{count} | "
            f"ERROR:{error_message}\n"
        )
        
        with open(self.log_file, 'a') as f:
            f.write(log_entry)
        
        print(f"📝 Logged failure: {node_name} (count: {count})")
    
    def get_all_failures(self):
        """
        Get all failure data for analytics
        
        Returns:
            dict: All failure records
        """
        return self.failures
    
    def _save_failures(self):
        """Save failure data to JSON file"""
        with open(self.failure_file, 'w') as f:
            json.dump(self.failures, f, indent=2)
    
    def print_summary(self):
        """Print failure summary for debugging"""
        print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print("📊 FAILURE SUMMARY")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
        if not self.failures:
            print("✅ No failures recorded")
        else:
            for node_name, data in self.failures.items():
                print(f"\n🔧 {node_name}:")
                print(f"   Count: {data['count']}")
                print(f"   First: {data.get('first_failure', 'N/A')}")
                print(f"   Last: {data.get('last_failure', 'N/A')}")
                print(f"   Robot: {data.get('robot_id', 'N/A')}")
        
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")


# CLI for testing
if __name__ == '__main__':
    counter = FailureCounter()
    
    # Test operations
    print("\n🧪 Testing Failure Counter...")
    
    # Simulate failures
    for i in range(5):
        count = counter.increment_failure_count('test_node')
        counter.log_failure('test_node', f'Test failure #{i+1}')
    
    # Print summary
    counter.print_summary()
    
    # Reset
    counter.reset_failure_count('test_node')
    counter.print_summary()
