#!/usr/bin/env python3
"""
Configuration loader for Isaac Sim sensor suites.
"""

import json
import os
from typing import Dict, Any, List, Optional
from examples.isaac_sim.sensors import SensorFactory

class SensorConfigLoader:
    """
    Loads sensor configurations from JSON files and applies them to a robot.
    """

    def __init__(self, config_dir: str = None):
        """
        Initialize the loader.

        Args:
            config_dir: Directory containing JSON config files. 
                        Defaults to 'sensor_configs' in the same directory as this script.
        """
        if config_dir is None:
            base_path = os.path.dirname(os.path.abspath(__file__))
            self.config_dir = os.path.join(base_path, "sensor_configs")
        else:
            self.config_dir = config_dir

    def list_available_configs(self) -> List[str]:
        """List available configuration names."""
        if not os.path.exists(self.config_dir):
            return []
        return [f.replace(".json", "") for f in os.listdir(self.config_dir) if f.endswith(".json")]

    def load_config(self, config_name: str) -> Dict[str, Any]:
        """
        Load a specific configuration dictionary.

        Args:
            config_name: Name of the config file (without .json extension).
        """
        path = os.path.join(self.config_dir, f"{config_name}.json")
        try:
            with open(path, "r") as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"❌ Configuration '{config_name}' not found at {path}")
            return {}
        except json.JSONDecodeError:
            print(f"❌ Invalid JSON in configuration '{config_name}'")
            return {}

    def apply_config(self, config_name: str, robot_prim_path: str):
        """
        Apply a sensor configuration to a robot.

        Args:
            config_name: Name of the config to apply.
            robot_prim_path: Path to the robot prim in the stage.
        """
        config = self.load_config(config_name)
        if not config:
            return

        print(f"🔧 Applying sensor suite: {config.get('name', 'Unknown')}")
        sensors = config.get("sensors", [])

        for sensor_def in sensors:
            s_type = sensor_def.get("type")
            name = sensor_def.get("name")
            parent_rel = sensor_def.get("parent") # relative to robot root usually, or specific link
            
            # Construct full parent path
            # If parent_rel is just a link name, append it to robot path
            # This logic assumes the structure /World/Robot/LinkName
            parent_path = f"{robot_prim_path}/{parent_rel}"
            
            position = tuple(sensor_def.get("position", [0, 0, 0]))
            orientation = tuple(sensor_def.get("orientation", [1, 0, 0, 0]))
            
            print(f"  - Adding {s_type} '{name}' to {parent_path}")

            if s_type == "camera":
                res = tuple(sensor_def.get("config", {}).get("resolution", [1280, 720]))
                SensorFactory.create_camera(
                    prim_path=name,
                    parent_path=parent_path,
                    position=position,
                    orientation=orientation,
                    resolution=res
                )
            elif s_type == "lidar":
                SensorFactory.create_lidar(
                    prim_path=name,
                    parent_path=parent_path,
                    position=position,
                    orientation=orientation,
                    config=sensor_def.get("config", {}).get("type", "Rotary")
                )
            elif s_type == "imu":
                SensorFactory.create_imu(
                    prim_path=name,
                    parent_path=parent_path,
                    position=position
                )
            else:
                print(f"    ⚠️ Unknown sensor type: {s_type}")

# Example usage
if __name__ == "__main__":
    loader = SensorConfigLoader()
    print(f"Available configs: {loader.list_available_configs()}")
    # loader.apply_config("navigation_suite", "/World/Humanoid")
