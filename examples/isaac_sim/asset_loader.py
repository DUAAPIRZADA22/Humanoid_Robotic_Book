#!/usr/bin/env python3
"""
Asset loader utility for Isaac Sim humanoid robotics examples.
This module handles loading of USD and URDF assets into the stage.
"""

from typing import Optional, Tuple, Dict, Any
import os

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.robots import Robot

class AssetLoader:
    """
    Helper class to manage loading of robot and environment assets.
    """

    def __init__(self, assets_root_path: Optional[str] = None):
        """
        Initialize the asset loader.

        Args:
            assets_root_path: Custom path to assets. If None, uses default NVIDIA nucleus.
        """
        self._assets_root_path = assets_root_path or get_assets_root_path()
        if not self._assets_root_path:
            print("⚠️ Warning: Could not find NVIDIA Nucleus assets. Default assets may not load.")
        else:
            print(f"✅ Asset loader initialized with root: {self._assets_root_path}")

    def load_usd(
        self, 
        usd_path: str, 
        prim_path: str, 
        position: Tuple[float, float, float] = (0, 0, 0),
        orientation: Tuple[float, float, float, float] = (1, 0, 0, 0),
        scale: Tuple[float, float, float] = (1, 1, 1)
    ) -> Optional[XFormPrim]:
        """
        Load a USD asset into the stage.

        Args:
            usd_path: Path to the .usd file.
            prim_path: The prim path where the asset will be loaded.
            position: Initial position (x, y, z).
            orientation: Initial orientation quaternion (w, x, y, z).
            scale: Scale factor (x, y, z).

        Returns:
            XFormPrim: The loaded prim wrapper or None if failed.
        """
        if is_prim_path_valid(prim_path):
            print(f"⚠️ Prim already exists at {prim_path}. Skipping load.")
            return XFormPrim(prim_path)

        # Handle relative paths vs absolute paths
        if not usd_path.startswith("http") and not os.path.isabs(usd_path) and not usd_path.startswith("omniverse:"):
             # If it's a relative path, might be local or relative to assets root
             pass

        try:
            success = add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            if not success:
                print(f"❌ Failed to add reference: {usd_path}")
                return None

            prim = XFormPrim(
                prim_path=prim_path,
                name=prim_path.split("/")[-1],
                position=position,
                orientation=orientation,
                scale=scale
            )
            print(f"✅ Loaded USD asset at {prim_path}")
            return prim

        except Exception as e:
            print(f"❌ Error loading USD asset: {e}")
            return None

    def load_humanoid_robot(
        self, 
        prim_path: str = "/World/Humanoid",
        position: Tuple[float, float, float] = (0, 0, 1.0)
    ) -> Optional[Robot]:
        """
        Load a standard humanoid robot for testing.
        Tries to load a sample robot from NVIDIA assets.

        Args:
            prim_path: Target prim path.
            position: Initial position.

        Returns:
            Robot: The robot instance.
        """
        # Attempt to find a standard biped in the assets
        # Note: Specific path depends on Isaac Sim version, using a generic one for the example
        robot_assets = [
            f"{self._assets_root_path}/Isaac/Robots/Franka/franka.usd", # Fallback if humanoid not found
            f"{self._assets_root_path}/Isaac/Robots/Humanoid/humanoid.usd" # Hypothetical path
        ]

        selected_asset = None
        # Logic to check existence could go here, but for now we trust the path or fail gracefully
        # defaulting to Franka as it is almost always present for testing
        selected_asset = f"{self._assets_root_path}/Isaac/Robots/Franka/franka.usd"

        print(f"ℹ️ Loading robot from: {selected_asset}")
        
        prim = self.load_usd(selected_asset, prim_path, position=position)
        if prim:
            return Robot(prim_path=prim_path, name="humanoid_robot")
        return None

    def load_environment(
        self,
        environment_name: str = "Simple_Room",
        prim_path: str = "/World/Environment"
    ) -> bool:
        """
        Load a pre-configured environment.

        Args:
            environment_name: Name of the environment (e.g., 'Simple_Room', 'Warehouse').
            prim_path: Target prim path.

        Returns:
            bool: True if successful.
        """
        env_paths = {
            "Simple_Room": f"{self._assets_root_path}/Isaac/Environments/Simple_Room/simple_room.usd",
            "Warehouse": f"{self._assets_root_path}/Isaac/Environments/Simple_Warehouse/warehouse.usd",
            "Grid": f"{self._assets_root_path}/Isaac/Environments/Grid/default_environment_tier3.usd"
        }

        if environment_name not in env_paths:
            print(f"⚠️ Unknown environment: {environment_name}. Available: {list(env_paths.keys())}")
            return False

        path = env_paths[environment_name]
        return self.load_usd(path, prim_path) is not None

# Example usage
if __name__ == "__main__":
    loader = AssetLoader()
    # verify asset root
    if loader._assets_root_path:
        print("Asset loader ready.")
