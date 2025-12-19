"""
Quality Control Failure Counter
Persistent failure tracking for manufacturing inspection system
"""

import json
import os
from datetime import datetime
from pathlib import Path


class QCFailureCounter:
    """
    Tracks node failures for quality control inspection system
    Stores failure counts with timestamps for maintenance analytics
    """
    
    def __init__(self, storage_dir: str = '/tmp/qc_failures'):
        """
        Initialize failure counter
        
        Args:
            storage_dir: Directory to store failure data
        """
        self.storage_dir = Path(storage_dir)
        self.storage_dir.mkdir(parents=True, exist_ok=True)
        
        self.counts_file = self.storage_dir / 'failure_counts.json'
        self.log_file = self.storage_dir / 'failure_log.txt'
        
        # Initialize or load existing data
        self.failure_data = self._load_failure_data()
    
    def _load_failure_data(self) -> dict:
        """Load failure data from JSON file"""
        if self.counts_file.exists():
            try:
                with open(self.counts_file, 'r') as f:
                    return json.load(f)
            except (json.JSONDecodeError, IOError):
                print(f"⚠️ Could not load failure data, starting fresh")
                return {}
        return {}
    
    def _save_failure_data(self):
        """Save failure data to JSON file"""
        try:
            with open(self.counts_file, 'w') as f:
                json.dump(self.failure_data, f, indent=2)
        except IOError as e:
            print(f"❌ Error saving failure data: {e}")
    
    def get_failure_count(self, node_name: str) -> int:
        """
        Get current failure count for a node
        
        Args:
            node_name: Name of the node
            
        Returns:
            Current failure count
        """
        if node_name in self.failure_data:
            return self.failure_data[node_name]['count']
        return 0
    
    def increment_failure_count(self, node_name: str) -> int:
        """
        Increment failure count for a node
        
        Args:
            node_name: Name of the node
            
        Returns:
            New failure count
        """
        if node_name not in self.failure_data:
            self.failure_data[node_name] = {
                'count': 0,
                'first_failure': datetime.now().isoformat(),
                'last_failure': None,
                'robot_id': 'QC-SYSTEM-001'
            }
        
        self.failure_data[node_name]['count'] += 1
        self.failure_data[node_name]['last_failure'] = datetime.now().isoformat()
        
        self._save_failure_data()
        
        return self.failure_data[node_name]['count']
    
    def reset_failure_count(self, node_name: str):
        """
        Reset failure count for a node (after successful recovery)
        
        Args:
            node_name: Name of the node
        """
        if node_name in self.failure_data:
            self.failure_data[node_name]['count'] = 0
            self.failure_data[node_name]['last_recovery'] = datetime.now().isoformat()
            self._save_failure_data()
    
    def log_failure(self, node_name: str, error_message: str):
        """
        Log failure to text file for maintenance review
        
        Args:
            node_name: Name of the failed node
            error_message: Description of the failure
        """
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        count = self.get_failure_count(node_name)
        
        log_entry = (
            f"[{timestamp}] {node_name} - Failure #{count}\n"
            f"  Error: {error_message}\n"
            f"  Robot ID: QC-SYSTEM-001\n"
            f"  Production Line: MAIN-LINE-A\n"
            f"{'-' * 80}\n"
        )
        
        try:
            with open(self.log_file, 'a') as f:
                f.write(log_entry)
        except IOError as e:
            print(f"❌ Error writing to log file: {e}")
    
    def get_all_failures(self) -> dict:
        """
        Get all failure data for analytics
        
        Returns:
            Complete failure data dictionary
        """
        return self.failure_data.copy()
