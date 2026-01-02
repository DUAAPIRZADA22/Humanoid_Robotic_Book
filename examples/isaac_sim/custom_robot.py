#!/usr/bin/env python3
"""
Custom Robot Example for Isaac Sim.
Demonstrates how to wrap a USD asset into a specialized Robot class with high-level control methods.
"""

from typing import Optional, List, Tuple
import numpy as np
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from examples.isaac_sim.asset_loader import AssetLoader

class CustomHumanoid(Robot):
    """
    A custom humanoid robot class that wraps a standard USD asset
    and provides high-level control interfaces.
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "custom_humanoid",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize the custom robot.

        Args:
            prim_path: The prim path where the robot will be spawned.
            name: Unique name for the robot instance.
            usd_path: Path to the USD asset. If None, uses default loader.
            position: Initial position.
            orientation: Initial orientation.
        """
        self.loader = AssetLoader()
        
        # Load the asset if it doesn't exist yet
        if usd_path:
            self.loader.load_usd(usd_path, prim_path, position=position, orientation=orientation)
        else:
            # Use the default humanoid from our loader if no custom USD provided
            # Note: We pass position here, but Robot class also takes it. 
            # Ideally asset loading happens before Robot wrapper init if we want to ensure it exists.
            self.loader.load_humanoid_robot(prim_path, position=position if position is not None else (0, 0, 1.0))

        super().__init__(
            prim_path=prim_path,
            name=name,
            position=position,
            orientation=orientation,
            articulation_controller=None,
        )

        self._dof_names = []
        self._default_joint_pos = None

    def initialize(self, physics_sim_view=None) -> None:
        """
        Initialize the robot after it has been added to the world.
        """
        super().initialize(physics_sim_view)
        
        self._dof_names = self.dof_names
        self._default_joint_pos = np.zeros(self.num_dof)
        
        print(f"🤖 CustomHumanoid '{self.name}' initialized with {self.num_dof} DOFs")
        print(f"   Joints: {self._dof_names}")

    def set_joint_positions(self, positions: np.ndarray, joint_indices: Optional[List[int]] = None) -> None:
        """
        Set target positions for specific joints.

        Args:
            positions: Target positions (radians/meters).
            joint_indices: Indices of joints to control. If None, assumes all joints.
        """
        action = ArticulationAction(joint_positions=positions, joint_indices=joint_indices)
        self.apply_action(action)

    def set_joint_velocities(self, velocities: np.ndarray, joint_indices: Optional[List[int]] = None) -> None:
        """
        Set target velocities for specific joints.

        Args:
            velocities: Target velocities.
            joint_indices: Indices of joints.
        """
        action = ArticulationAction(joint_velocities=velocities, joint_indices=joint_indices)
        self.apply_action(action)

    def wave_hand(self, side: str = "right"):
        """
        Example high-level behavior: Wave a hand.
        Note: This is a kinematic stub. Real implementation requires specific joint names.
        """
        print(f"👋 {self.name} is waving {side} hand!")
        
        # Example: Find shoulder and elbow joints for the specified side
        # This relies on standard naming conventions (e.g., 'right_shoulder_pitch')
        prefix = "r" if side == "right" else "l"
        
        # Hypothetical joint targets for a wave pose
        # In a real scenario, you'd map these to the actual DOF indices of your asset
        pass

    def walk_forward(self, speed: float = 1.0):
        """
        Example high-level behavior: Walk forward.
        """
        print(f"🚶 {self.name} walking forward at speed {speed}")
        # Implementation would interface with a gait controller or RL policy
        pass

    def get_status(self) -> dict:
        """
        Get current robot status telemetry.
        """
        return {
            "position": self.get_world_pose()[0].tolist(),
            "orientation": self.get_world_pose()[1].tolist(),
            "joint_positions": self.get_joint_positions().tolist(),
            "joint_velocities": self.get_joint_velocities().tolist(),
        }

# Example usage within the script
if __name__ == "__main__":
    # This block is for testing the class definition syntax, 
    # actual execution requires running in Isaac Sim context.
    print("CustomHumanoid class defined successfully.")
