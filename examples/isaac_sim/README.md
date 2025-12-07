# Isaac Sim Examples

This directory contains example code for working with NVIDIA Isaac Sim and humanoid robots.

## Files

- **base_scene.py** - Basic scene setup with ground plane, lighting, and robot loading
- **asset_loader.py** - Utilities for loading and managing robot assets
- **sensors.py** - Camera, LiDAR, and IMU sensor implementations
- **ros_bridge.py** - ROS 2 integration for Isaac Sim
- **sim_controller.py** - High-level simulation control utilities
- **data_collector.py** - Synthetic data generation tools

## Getting Started

1. **Set up Isaac Sim environment**:
```bash
source isaac_sim_python.sh
```

2. **Run the base scene example**:
```bash
python base_scene.py
```

3. **Explore other examples**:
```bash
# Run with a specific robot model
python base_scene.py --robot-path path/to/robot.usd

# Run with custom physics settings
python base_scene.py --physics-dt 0.002 --gravity -9.8
```

## Requirements

- NVIDIA Isaac Sim 2024.1.0 or later
- Python 3.8+
- RTX-capable GPU with 8GB+ VRAM
- Ubuntu 22.04 LTS or Windows 10/11 with WSL2

## Dependencies

```bash
pip install numpy
pip install matplotlib  # For visualization
pip install scipy      # For advanced calculations
```

## Configuration

Most examples can be configured via command-line arguments:

- `--physics-dt`: Physics timestep (default: 1/60)
- `--rendering-dt`: Rendering timestep (default: 1/60)
- `--gravity`: Gravity magnitude (default: -9.81)
- `--robot-path`: Path to custom robot USD file
- `--no-robot`: Run scene without loading robot
- `--duration`: Simulation duration in seconds

## Notes

- These examples are designed for educational purposes
- Performance may vary based on hardware capabilities
- For production use, additional optimization may be required
- Check the logs for detailed error messages if issues occur