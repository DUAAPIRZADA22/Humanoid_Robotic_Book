#!/usr/bin/env python3
"""
Sensor simulation interface for Isaac Sim.
This module provides wrappers to easily attach and configure sensors on robots.
"""

from typing import Optional, Tuple
import numpy as np

from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.sensor import Camera, LidarRtx, IMUSensor
import omni.kit.commands

class SensorFactory:
    """
    Factory class to create and configure sensors.
    """

    @staticmethod
    def create_camera(
        prim_path: str,
        parent_path: str,
        position: Tuple[float, float, float] = (0, 0, 0),
        orientation: Tuple[float, float, float, float] = (1, 0, 0, 0), # w, x, y, z
        resolution: Tuple[int, int] = (1280, 720)
    ) -> Optional[Camera]:
        """
        Create a camera sensor attached to a parent prim.

        Args:
            prim_path: Name of the camera prim (relative to parent).
            parent_path: Path of the parent prim (e.g., robot end-effector).
            position: Relative position (x, y, z).
            orientation: Relative orientation (w, x, y, z).
            resolution: Image resolution (width, height).

        Returns:
            Camera: The camera object.
        """
        full_path = f"{parent_path}/{prim_path}"
        
        try:
            # Create the camera prim
            camera = Camera(
                prim_path=full_path,
                position=np.array(position),
                orientation=np.array(orientation),
                resolution=resolution
            )
            
            # Initialize
            camera.initialize()
            print(f"✅ Created Camera at {full_path}")
            return camera
        except Exception as e:
            print(f"❌ Failed to create Camera: {e}")
            return None

    @staticmethod
    def create_lidar(
        prim_path: str,
        parent_path: str,
        config: str = "Rotary", # Example config
        position: Tuple[float, float, float] = (0, 0, 0),
        orientation: Tuple[float, float, float, float] = (1, 0, 0, 0)
    ) -> Optional[LidarRtx]:
        """
        Create a LiDAR sensor.

        Args:
            prim_path: Name of the lidar prim.
            parent_path: Path of the parent prim.
            config: Lidar configuration name.
            position: Relative position.
            orientation: Relative orientation.

        Returns:
            LidarRtx: The lidar object.
        """
        full_path = f"{parent_path}/{prim_path}"
        
        try:
            # Create the lidar prim
            # Note: LidarRtx usually requires specific flow to create via commands or wrapping existing prim
            # Here we use the command pattern for safety in recent Isaac Sim versions
            
            result = omni.kit.commands.execute(
                "RangeSensorCreate",
                path=full_path,
                parent=parent_path,
                min_range=0.4,
                max_range=100.0,
                draw_points=True,
                draw_lines=False,
                horizontal_fov=360.0,
                vertical_fov=30.0,
                horizontal_resolution=0.4,
                vertical_resolution=4.0,
                rotation_rate=0.0,
                high_lod=True,
                yaw_offset=0.0,
                enable_semantics=True
            )

            if result:
                lidar = LidarRtx(prim_path=full_path)
                lidar.initialize()
                # Set pose
                lidar.set_local_pose(translation=np.array(position), orientation=np.array(orientation))
                print(f"✅ Created LiDAR at {full_path}")
                return lidar
            else:
                print(f"❌ Failed to execute RangeSensorCreate command for {full_path}")
                return None

        except Exception as e:
            print(f"❌ Failed to create LiDAR: {e}")
            return None

    @staticmethod
    def create_imu(
        prim_path: str,
        parent_path: str,
        position: Tuple[float, float, float] = (0, 0, 0)
    ) -> Optional[IMUSensor]:
        """
        Create an IMU sensor.

        Args:
            prim_path: Name of the IMU prim.
            parent_path: Path of the parent prim.
            position: Relative position.

        Returns:
            IMUSensor: The IMU object.
        """
        full_path = f"{parent_path}/{prim_path}"
        
        try:
            imu = IMUSensor(
                prim_path=full_path,
                position=np.array(position),
                frequency=100, # 100 Hz
                sensor_period=-1.0 # driven by physics
            )
            
            imu.initialize()
            print(f"✅ Created IMU at {full_path}")
            return imu
        except Exception as e:
            print(f"❌ Failed to create IMU: {e}")
            return None

class DataCollector:
    """
    Helper to collect data from multiple sensors.
    """
    def __init__(self):
        self.sensors = {}

    def register_sensor(self, name: str, sensor_obj):
        self.sensors[name] = sensor_obj

    def get_data(self) -> dict:
        data = {}
        for name, sensor_obj in self.sensors.items():
            if isinstance(sensor_obj, Camera):
                data[name] = {
                    "rgb": sensor_obj.get_rgba(),
                    "depth": sensor_obj.get_depth()
                }
            elif isinstance(sensor_obj, IMUSensor):
                data[name] = {
                    "lin_acc": sensor_obj.get_linear_acceleration_data(),
                    "ang_vel": sensor_obj.get_angular_velocity_data()
                }
            # Add other sensor types as needed
        return data
