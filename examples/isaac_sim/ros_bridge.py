#!/usr/bin/env python3
"""
ROS 2 Bridge implementation for Isaac Sim.
This module handles the publication of sensor data and subscription to control commands.
"""

from typing import Optional, List
import omni.kit.commands
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.extensions import enable_extension

class Ros2Bridge:
    """
    Manages ROS 2 communication for Isaac Sim.
    """

    def __init__(self):
        """Initialize the ROS 2 bridge and ensure necessary extensions are enabled."""
        # Ensure ROS 2 bridge extension is enabled
        enable_extension("omni.isaac.ros2_bridge")
        print("✅ ROS 2 Bridge extension enabled")

    def setup_clock(self, prim_path: str = "/World/ROS_Clock"):
        """
        Setup the ROS 2 Clock publisher.
        
        Args:
            prim_path: Path for the clock prim.
        """
        omni.kit.commands.execute(
            "ROS2BridgeCreateClock",
            path=prim_path,
            topic_name="/clock"
        )
        print(f"✅ ROS 2 Clock setup at {prim_path}")

    def setup_camera_telemetry(self, camera_prim_path: str, topic_prefix: str = "/camera"):
        """
        Setup ROS 2 image and camera info publishers.

        Args:
            camera_prim_path: Path to the camera prim in Isaac Sim.
            topic_prefix: Prefix for ROS topics.
        """
        if not is_prim_path_valid(camera_prim_path):
            print(f"❌ Camera prim path invalid: {camera_prim_path}")
            return

        # Publish RGB
        omni.kit.commands.execute(
            "ROS2BridgeCreateCamera",
            path=f"{camera_prim_path}/ROS_RGB",
            camera_prim_path=camera_prim_path,
            topic_name=f"{topic_prefix}/image_raw",
            type="rgb"
        )

        # Publish Camera Info
        omni.kit.commands.execute(
            "ROS2BridgeCreateCameraInfo",
            path=f"{camera_prim_path}/ROS_Info",
            camera_prim_path=camera_prim_path,
            topic_name=f"{topic_prefix}/camera_info"
        )
        print(f"✅ ROS 2 Telemetry setup for {camera_prim_path}")

    def setup_lidar_telemetry(self, lidar_prim_path: str, topic: str = "/scan"):
        """
        Setup ROS 2 LaserScan publisher for a LiDAR.

        Args:
            lidar_prim_path: Path to the LiDAR prim.
            topic: ROS topic name.
        """
        omni.kit.commands.execute(
            "ROS2BridgeCreateLaserScan",
            path=f"{lidar_prim_path}/ROS_LaserScan",
            lidar_prim_path=lidar_prim_path,
            topic_name=topic,
            frame_id="lidar_link"
        )
        print(f"✅ ROS 2 LaserScan setup for {lidar_prim_path}")

    def setup_robot_control(self, robot_prim_path: str, topic: str = "/cmd_vel"):
        """
        Setup ROS 2 subscriber for robot control (e.g., Twist commands).

        Args:
            robot_prim_path: Path to the robot prim.
            topic: ROS topic to subscribe to.
        """
        # This typically involves a DifferentialController or ArticulationController node
        # mapping ROS Twist messages to joint velocities or wheel speeds.
        omni.kit.commands.execute(
            "ROS2BridgeCreateTwistSubscriber",
            path=f"{robot_prim_path}/ROS_TwistSub",
            topic_name=topic,
            robot_prim_path=robot_prim_path
        )
        print(f"✅ ROS 2 Twist Subscriber setup for {robot_prim_path}")

    def setup_tf_publisher(self, prim_path: str = "/World/ROS_TF", frames: Optional[List[str]] = None):
        """
        Setup ROS 2 TF publisher for transforms.

        Args:
            prim_path: Path for the TF prim.
            frames: List of prim paths to include in TF.
        """
        omni.kit.commands.execute(
            "ROS2BridgeCreateTFPublisher",
            path=prim_path,
            topic_name="/tf"
        )
        print(f"✅ ROS 2 TF Publisher setup")

# Example usage pattern
if __name__ == "__main__":
    bridge = Ros2Bridge()
    bridge.setup_clock()
