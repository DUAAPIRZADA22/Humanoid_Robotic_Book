#!/usr/bin/env python3
"""
Data collection service for Isaac Sim simulations.
Handles saving sensor data, robot states, and metadata to disk.
"""

import os
import json
import time
import numpy as np
from typing import Dict, Any, Optional

class DataCollectionService:
    """
    Service for persistent data storage during robotic simulations.
    """

    def __init__(self, output_dir: str = "simulation_data"):
        """
        Initialize the data collection service.

        Args:
            output_dir: Directory where data will be saved.
        """
        self.session_id = time.strftime("%Y%m%d-%H%M%S")
        self.base_dir = os.path.join(output_dir, self.session_id)
        self.images_dir = os.path.join(self.base_dir, "images")
        self.logs_path = os.path.join(self.base_dir, "telemetry.jsonl")
        
        self._ensure_dirs()
        self.metadata = {
            "session_id": self.session_id,
            "start_time": time.time(),
            "config": {}
        }
        print(f"📁 Data collection service initialized at {self.base_dir}")

    def _ensure_dirs(self):
        """Create necessary directories if they don't exist."""
        os.makedirs(self.images_dir, exist_ok=True)

    def save_metadata(self, config: Dict[str, Any]):
        """Save simulation configuration and metadata."""
        self.metadata["config"] = config
        with open(os.path.join(self.base_dir, "metadata.json"), "w") as f:
            json.dump(self.metadata, f, indent=4)

    def log_step(self, step: int, timestamp: float, data: Dict[str, Any]):
        """
        Log data for a single simulation step.

        Args:
            step: Current simulation step.
            timestamp: Simulation time.
            data: Data dictionary to log (should be JSON serializable).
        """
        log_entry = {
            "step": step,
            "timestamp": timestamp,
            "data": self._process_data_for_json(data)
        }
        
        with open(self.logs_path, "a") as f:
            f.write(json.dumps(log_entry) + "\n")

    def save_image(self, name: str, step: int, image_data: np.ndarray):
        """
        Save an image (e.g., from a camera sensor).

        Args:
            name: Sensor name.
            step: Current step.
            image_data: Numpy array containing image data.
        """
        filename = f"{name}_{step:06d}.npy"
        path = os.path.join(self.images_dir, filename)
        np.save(path, image_data)

    def _process_data_for_json(self, data: Any) -> Any:
        """Recursively convert numpy arrays to lists for JSON serialization."""
        if isinstance(data, dict):
            return {k: self._process_data_for_json(v) for k, v in data.items()}
        elif isinstance(data, list):
            return [self._process_data_for_json(v) for v in data]
        elif isinstance(data, np.ndarray):
            return data.tolist()
        elif isinstance(data, (np.float32, np.float64)):
            return float(data)
        elif isinstance(data, (np.int32, np.int64)):
            return int(data)
        return data

    def finalize(self):
        """Finalize the collection session."""
        self.metadata["end_time"] = time.time()
        self.metadata["total_duration"] = self.metadata["end_time"] - self.metadata["start_time"]
        self.save_metadata(self.metadata.get("config", {}))
        print(f"✅ Data collection session {self.session_id} finalized.")

# Example usage
if __name__ == "__main__":
    service = DataCollectionService()
    service.save_metadata({"robot": "humanoid_v1", "env": "warehouse"})
    for i in range(5):
        service.log_step(i, i * 0.1, {"pose": np.random.rand(3)})
        time.sleep(0.1)
    service.finalize()
