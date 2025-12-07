#!/usr/bin/env python3
"""Base scene setup for Isaac Sim humanoid robot simulation."""

import asyncio
from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.kit.commands


class IsaacSimBaseScene:
    """Base scene class for Isaac Sim humanoid robot simulations."""

    def __init__(self, physics_dt=1/60.0, rendering_dt=1/60.0):
        """
        Initialize the base scene.

        Args:
            physics_dt: Physics timestep in seconds
            rendering_dt: Rendering timestep in seconds
        """
        self.world = None
        self.physics_dt = physics_dt
        self.rendering_dt = rendering_dt
        self.robot_prim_path = None

    async def create_scene(self, load_robot=True):
        """
        Create a basic simulation scene.

        Args:
            load_robot: Whether to load a humanoid robot

        Returns:
            World: The created world instance
        """
        # Create the world
        self.world = World(
            physics_dt=self.physics_dt,
            rendering_dt=self.rendering_dt,
            stage_units_in_meters=1.0
        )

        # Add ground plane with realistic material
        ground_plane = GroundPlane(
            prim_path="/World/GroundPlane",
            size=(100.0, 100.0),
            physics_material_path="/World/PhysicsMaterials/Concrete"
        )

        # Set up physics material properties
        from omni.isaac.core.materials import PhysicsMaterial
        concrete_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/Concrete",
            static_friction=0.7,
            dynamic_friction=0.6,
            restitution=0.1
        )
        ground_plane.apply_physics_material(concrete_material)

        # Add basic lighting
        self._setup_lighting()

        # Load humanoid robot if requested
        if load_robot:
            await self._load_humanoid_robot()

        # Set up scene boundaries
        self._setup_boundaries()

        # Initialize simulation
        self.world.initialize_simulation_physics()

        return self.world

    def _setup_lighting(self):
        """Set up basic lighting for the scene."""
        from omni.isaac.core.utils.prims import define_prim

        # Dome light for ambient lighting
        dome_light = define_prim(
            "/World/DomeLight",
            "DomeLight"
        )
        dome_light.CreateAttribute("inputs:intensity", 1000.0)
        dome_light.CreateAttribute("inputs:texture:file", "")

        # Directional light for shadows
        directional_light = define_prim(
            "/World/DirectionalLight",
            "DistantLight"
        )
        directional_light.CreateAttribute("inputs:intensity", 5000.0)
        directional_light.CreateAttribute("inputs:color", (1.0, 1.0, 0.9))  # Warm white

    async def _load_humanoid_robot(self):
        """Load a humanoid robot into the scene."""
        assets_root = get_assets_root_path()

        # Try to load humanoid robot from assets
        robot_usd_path = f"{assets_root}/Isaac/Robots/Humanoid/humanoid.usd"

        try:
            await add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/HumanoidRobot")
            self.robot_prim_path = "/World/HumanoidRobot"

            # Configure robot physics
            self._configure_robot_physics()

            print(f"✅ Humanoid robot loaded from {robot_usd_path}")

        except Exception as e:
            print(f"⚠️ Could not load humanoid robot: {e}")
            print("Creating a simple box robot instead...")

            # Create a simple box robot as fallback
            from omni.isaac.core.objects import DynamicCuboid
            robot = DynamicCuboid(
                prim_path="/World/SimpleRobot",
                position=(0, 0, 1.0),
                size=(0.5, 0.3, 1.0),
                mass=50.0,
                color=(0.0, 0.5, 1.0)
            )
            self.robot_prim_path = "/World/SimpleRobot"

    def _configure_robot_physics(self):
        """Configure physics properties for the humanoid robot."""
        from omni.isaac.core.utils.prims import get_prim_at_path
        from pxr import UsdPhysics, PhysxSchema

        robot_prim = get_prim_at_path(self.robot_prim_path)
        if not robot_prim:
            return

        # Enable rigid body physics
        physics_api = UsdPhysics.RigidBodyAPI.Get(robot_prim)
        if not physics_api:
            physics_api = UsdPhysics.RigidBodyAPI.Apply(robot_prim)
            physics_api.CreateVelocityAttr().Set((0.0, 0.0, 0.0))

        # Configure physx-specific properties
        physx_api = PhysxSchema.PhysxRigidBodyAPI.Get(robot_prim)
        if not physx_api:
            physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(robot_prim)

        # Set solver settings for better stability
        physx_api.CreateSolverPositionIterationCountAttr().Set(8)
        physx_api.CreateSolverVelocityIterationCountAttr().Add(2)

    def _setup_boundaries(self):
        """Set up invisible boundaries to keep robot contained."""
        from omni.isaac.core.utils.prims import define_prim

        # Create invisible walls
        boundary_size = 50.0
        wall_height = 10.0
        wall_thickness = 0.5

        wall_configs = [
            # Front wall
            {
                "path": "/World/Boundaries/FrontWall",
                "position": (0, boundary_size/2, wall_height/2),
                "size": (boundary_size, wall_thickness, wall_height)
            },
            # Back wall
            {
                "path": "/World/Boundaries/BackWall",
                "position": (0, -boundary_size/2, wall_height/2),
                "size": (boundary_size, wall_thickness, wall_height)
            },
            # Left wall
            {
                "path": "/World/Boundaries/LeftWall",
                "position": (-boundary_size/2, 0, wall_height/2),
                "size": (wall_thickness, boundary_size, wall_height)
            },
            # Right wall
            {
                "path": "/World/Boundaries/RightWall",
                "position": (boundary_size/2, 0, wall_height/2),
                "size": (wall_thickness, boundary_size, wall_height)
            }
        ]

        for config in wall_configs:
            wall = define_prim(config["path"], "Cube")
            wall.GetAttribute("size").Set(config["size"])
            wall.GetAttribute("translate").Set(config["position"])

            # Make walls invisible but collision-enabled
            from pxr import UsdGeom
            mesh_api = UsdGeom.Mesh.Get(wall)
            if mesh_api:
                mesh_api.CreateVisibilityAttr().Set("invisible")

            # Add collision
            from pxr import UsdPhysics
            collision_api = UsdPhysics.CollisionAPI.Apply(wall)

    async def run_simulation(self, duration=10.0):
        """
        Run the simulation for a specified duration.

        Args:
            duration: Duration to run in seconds
        """
        if not self.world:
            print("❌ Scene not created. Call create_scene() first.")
            return

        print(f"🚀 Starting simulation for {duration} seconds...")
        self.world.play()

        # Run for specified duration
        await asyncio.sleep(duration)

        self.world.stop()
        print("✅ Simulation completed")

    def reset_simulation(self):
        """Reset the simulation to initial state."""
        if self.world:
            self.world.reset()
            print("🔄 Simulation reset")

    async def close(self):
        """Clean up resources."""
        if self.world:
            self.world.clear()
            self.world = None
        print("🧹 Scene cleaned up")


# Example usage
async def main():
    """Example of how to use the base scene class."""
    scene = IsaacSimBaseScene()

    # Create scene with robot
    world = await scene.create_scene(load_robot=True)

    # Run simulation
    await scene.run_simulation(duration=5.0)

    # Clean up
    await scene.close()


if __name__ == "__main__":
    # Run the example
    asyncio.run(main())