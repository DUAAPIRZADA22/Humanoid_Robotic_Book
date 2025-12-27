#!/usr/bin/env python3
"""
Simulation controller for Isaac Sim humanoid robotics.
Integrates scene, robot, sensors, and ROS 2 bridge.
"""

import asyncio
from omni.isaac.core import World
from examples.isaac_sim.base_scene import IsaacSimBaseScene
from examples.isaac_sim.asset_loader import AssetLoader
from examples.isaac_sim.sensors import SensorFactory, DataCollector
from examples.isaac_sim.ros_bridge import Ros2Bridge

class HumanoidSimController:
    """
    High-level controller to manage the simulation lifecycle.
    """

    def __init__(self):
        self.scene = IsaacSimBaseScene()
        self.loader = AssetLoader()
        self.bridge = Ros2Bridge()
        self.data_collector = DataCollector()
        self.world = None

    async def setup(self):
        """Perform full simulation setup."""
        # 1. Create World and Base Scene
        self.world = await self.scene.create_scene(load_robot=False)
        
        # 2. Load Environment
        self.loader.load_environment("Grid")
        
        # 3. Load Humanoid Robot
        robot = self.loader.load_humanoid_robot(prim_path="/World/Humanoid", position=(0, 0, 1.0))
        if not robot:
            print("❌ Failed to load robot. Simulation might be empty.")
            return

        # 4. Attach Sensors
        # Attach a camera to the head
        camera = SensorFactory.create_camera(
            prim_path="head_camera",
            parent_path="/World/Humanoid/head", # Path depends on the specific USD structure
            position=(0.1, 0, 0.1)
        )
        if camera:
            self.data_collector.register_sensor("head_rgb", camera)

        # 5. Setup ROS 2 Bridge
        self.bridge.setup_clock()
        self.bridge.setup_tf_publisher()
        if camera:
            self.bridge.setup_camera_telemetry("/World/Humanoid/head/head_camera")
        
        self.bridge.setup_robot_control("/World/Humanoid")

        print("🚀 Simulation setup complete")

    async def run(self, iterations: int = 1000):
        """Run the simulation loop."""
        if not self.world:
            await self.setup()

        self.world.play()
        print(f"🏃 Running simulation for {iterations} iterations...")

        for i in range(iterations):
            # Step the simulation
            self.world.step(render=True)
            
            # Example: Collect data every 10 steps
            if i % 10 == 0:
                data = self.data_collector.get_data()
                # Here you would typically pass data to an AI model or log it
                pass

            # Yield to other tasks
            await asyncio.sleep(0)

        self.world.stop()
        print("🏁 Simulation finished")

    async def cleanup(self):
        """Clean up resources."""
        await self.scene.close()

async def main():
    controller = HumanoidSimController()
    try:
        await controller.setup()
        await controller.run(iterations=500)
    finally:
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
