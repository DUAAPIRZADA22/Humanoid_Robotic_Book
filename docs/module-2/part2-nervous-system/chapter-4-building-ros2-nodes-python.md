---
slug: /module-2/part2-nervous-system/chapter-4-building-ros2-nodes-python
title: "Chapter 4: Building ROS 2 Nodes with Python"
description: "Practical guide to developing ROS 2 nodes using Python, covering data structures, message passing, service implementation, and real-world robotics applications."
tags:
  - ros2-python
  - node-development
  - robotics-programming
  - python
  - practical-examples
---

## Learning Objectives

- Master ROS 2 Python client library (rclpy) fundamentals
- Implement publishers, subscribers, services, and actions in Python
- Handle sensor data and control commands effectively
- Create robust, production-ready ROS 2 nodes
- Apply best practices for debugging and testing

## 4.1 ROS 2 Python Client Library (rclpy) Overview

The ROS 2 Python client library (rclpy) provides the Python bindings for the ROS 2 client library (RCL). It enables Python developers to create ROS 2 nodes that can publish, subscribe, and interact with the ROS 2 ecosystem.

### 4.1.1 Basic Node Template

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading

class RobotBaseNode(Node):
    """Base template for ROS 2 nodes with common functionality"""

    def __init__(self, node_name):
        super().__init__(node_name)

        # Node metadata
        self.node_name = node_name
        self.start_time = self.get_clock().now()

        # State tracking
        self.is_running = False
        self.error_count = 0
        self.max_errors = 10

        # Configuration
        self.declare_parameter('update_frequency', 30.0)
        self.declare_parameter('debug_mode', False)

        self.update_frequency = self.get_parameter('update_frequency').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Logging setup
        self.setup_logging()

        # Initialize node components
        self.init_components()

        # Setup timers
        self.setup_timers()

        self.get_logger().info(f'Node {node_name} initialized successfully')

    def setup_logging(self):
        """Configure logging based on parameters"""
        if self.debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Create loggers for different components
        self.comms_logger = self.get_logger().get_child('communications')
        self.control_logger = self.get_logger().get_child('control')
        self.safety_logger = self.get_logger().get_child('safety')

    def init_components(self):
        """Initialize node-specific components"""
        # Override in subclasses
        pass

    def setup_timers(self):
        """Setup periodic timers"""
        if self.update_frequency > 0:
            period = 1.0 / self.update_frequency
            self.update_timer = self.create_timer(
                period, self.update_callback
            )

        # Status timer (1 Hz)
        self.status_timer = self.create_timer(
            1.0, self.status_callback
        )

    def update_callback(self):
        """Main update loop"""
        if not self.is_running:
            return

        try:
            self.update()
        except Exception as e:
            self.handle_error(f'Update callback error: {e}')

    def update(self):
        """Node-specific update logic"""
        # Override in subclasses
        pass

    def status_callback(self):
        """Publish periodic status"""
        uptime = (self.get_clock().now() - self.start_time).to_msg().sec
        status = {
            'node_name': self.node_name,
            'uptime_seconds': uptime,
            'is_running': self.is_running,
            'error_count': self.error_count,
            'update_frequency': self.update_frequency
        }

        self.get_logger().info(f'Status: {status}', throttle_duration_sec=5.0)

    def handle_error(self, error_message):
        """Handle errors with logging and recovery"""
        self.error_count += 1
        self.get_logger().error(error_message)

        if self.error_count >= self.max_errors:
            self.get_logger().fatal(f'Too many errors ({self.error_count}), shutting down')
            self.shutdown()

    def start(self):
        """Start node operation"""
        self.is_running = True
        self.get_logger().info(f'Starting node {self.node_name}')

    def stop(self):
        """Stop node operation"""
        self.is_running = False
        self.get_logger().info(f'Stopping node {self.node_name}')

    def shutdown(self):
        """Clean shutdown"""
        self.stop()
        self.destroy_timer(self.update_timer)
        self.destroy_timer(self.status_timer)
        self.get_logger().info(f'Node {self.node_name} shutdown complete')
```

## 4.2 Sensor Node Implementation

### 4.2.1 Camera Driver Node

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class CameraDriverNode(RobotBaseNode):
    """ROS 2 node for camera sensor interface"""

    def __init__(self):
        super().__init__('camera_driver')

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.camera_id = self.get_parameter('camera_id').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value

        # Camera calibration (example values)
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.camera_frame
        self.camera_info.width = self.image_width
        self.camera_info.height = self.image_height

        # Camera intrinsics (example for 640x480 camera)
        self.camera_info.k = [
            525.0, 0.0, 320.0,
            0.0, 525.0, 240.0,
            0.0, 0.0, 1.0
        ]
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients
        self.camera_info.p = [
            525.0, 0.0, 320.0, 0.0,
            0.0, 525.0, 240.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Camera state
        self.camera = None
        self.is_connected = False

        # Initialize camera
        self.init_camera()

    def init_camera(self):
        """Initialize camera device"""
        try:
            self.camera = cv2.VideoCapture(self.camera_id)

            if not self.camera.isOpened():
                raise Exception(f"Failed to open camera {self.camera_id}")

            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.camera.set(cv2.CAP_PROP_FPS, self.publish_rate)

            # Create publishers
            self.image_publisher = self.create_publisher(
                Image, 'camera/image_raw', 10
            )

            self.info_publisher = self.create_publisher(
                CameraInfo, 'camera/camera_info', 10
            )

            self.is_connected = True
            self.get_logger().info(f'Camera {self.camera_id} initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            self.is_connected = False

    def init_components(self):
        """Initialize node components"""
        if self.is_connected:
            self.start()

    def update(self):
        """Capture and publish camera data"""
        if not self.is_connected or not self.camera:
            return

        try:
            # Capture frame
            ret, frame = self.camera.read()

            if not ret:
                self.get_logger().warn('Failed to capture frame')
                return

            # Convert to ROS message
            timestamp = self.get_clock().now()

            # Publish image
            self.publish_image(frame, timestamp)

            # Publish camera info
            self.publish_camera_info(timestamp)

            # Publish transform
            self.publish_camera_transform(timestamp)

        except Exception as e:
            self.handle_error(f'Camera update error: {e}')

    def publish_image(self, frame, timestamp):
        """Convert and publish image frame"""
        try:
            # Convert OpenCV image to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(
                frame, encoding='bgr8'
            )

            # Set header
            ros_image.header.stamp = timestamp.to_msg()
            ros_image.header.frame_id = self.camera_frame

            # Publish
            self.image_publisher.publish(ros_image)

        except Exception as e:
            self.comms_logger.error(f'Failed to publish image: {e}')

    def publish_camera_info(self, timestamp):
        """Publish camera calibration info"""
        self.camera_info.header.stamp = timestamp.to_msg()
        self.info_publisher.publish(self.camera_info)

    def publish_camera_transform(self, timestamp):
        """Publish static camera transform"""
        # This would normally be configured based on camera mounting
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = self.camera_frame

        # Example transform (camera mounted 0.2m above base)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.2

        # No rotation
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

    def shutdown(self):
        """Clean shutdown"""
        if self.camera:
            self.camera.release()
        super().shutdown()
```

### 4.2.2 LiDAR Driver Node

```python
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import math

class LidarDriverNode(RobotBaseNode):
    """ROS 2 node for 2D LiDAR sensor interface"""

    def __init__(self):
        super().__init__('lidar_driver')

        # LiDAR parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('scan_frequency', 10.0)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_frequency = self.get_parameter('scan_frequency').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value

        # Calculate scan parameters
        self.num_readings = 720  # Number of laser readings per scan
        self.angle_increment = (self.angle_max - self.angle_min) / self.num_readings

        # LiDAR state
        self.laser_data = np.zeros(self.num_readings)
        self.intensities = np.zeros(self.num_readings)

        # Publishers
        self.scan_publisher = self.create_publisher(
            LaserScan, 'scan', 10
        )

        # Simulated LiDAR data (replace with actual hardware interface)
        self.simulation_mode = True
        self.obstacles = [
            {'x': 2.0, 'y': 0.0, 'radius': 0.5},
            {'x': -1.5, 'y': 1.5, 'radius': 0.3},
            {'x': 0.0, 'y': -2.0, 'radius': 0.8}
        ]

        self.start()

    def update(self):
        """Generate and publish LiDAR scan"""
        if self.simulation_mode:
            self.simulate_laser_scan()

        self.publish_laser_scan()

    def simulate_laser_scan(self):
        """Simulate laser scan with obstacles"""
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment

            # Cast ray in this direction
            min_range = self.range_max

            for obstacle in self.obstacles:
                # Calculate distance to obstacle
                dist = self.ray_circle_intersection(
                    angle, obstacle['x'], obstacle['y'], obstacle['radius']
                )

                if dist < min_range:
                    min_range = dist

            self.laser_data[i] = min_range
            self.intensities[i] = 100.0 if min_range < self.range_max else 0.0

    def ray_circle_intersection(self, angle, cx, cy, radius):
        """Calculate intersection of ray with circular obstacle"""
        # Ray origin at (0,0), direction given by angle
        dx = math.cos(angle)
        dy = math.sin(angle)

        # Ray-circle intersection
        a = dx*dx + dy*dy
        b = 2*(dx*(-cx) + dy*(-cy))
        c = cx*cx + cy*cy - radius*radius

        discriminant = b*b - 4*a*c

        if discriminant < 0:
            return self.range_max

        t1 = (-b - math.sqrt(discriminant)) / (2*a)
        t2 = (-b + math.sqrt(discriminant)) / (2*a)

        # Return closest positive intersection
        if t1 > 0 and t1 < self.range_max:
            return t1
        elif t2 > 0 and t2 < self.range_max:
            return t2
        else:
            return self.range_max

    def publish_laser_scan(self):
        """Publish LaserScan message"""
        scan_msg = LaserScan()

        # Header
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id

        # Scan parameters
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 1.0 / (self.num_readings * self.scan_frequency)
        scan_msg.scan_time = 1.0 / self.scan_frequency

        # Range parameters
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # Scan data
        scan_msg.ranges = self.laser_data.tolist()
        scan_msg.intensities = self.intensities.tolist()

        # Publish
        self.scan_publisher.publish(scan_msg)
```

## 4.3 Control Node Implementation

### 4.3.1 Differential Drive Controller

```python
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion
import math

class DifferentialDriveController(RobotBaseNode):
    """ROS 2 node for differential drive robot control"""

    def __init__(self):
        super().__init__('differential_drive_controller')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 0.4)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Control commands
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

        # Publishers and subscribers
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel',
            self.cmd_vel_callback, 10
        )

        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10
        )

        self.odom_publisher = self.create_publisher(
            Odometry, 'odom', 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Control parameters
        self.declare_parameter('linear_acceleration_limit', 2.0)
        self.declare_parameter('angular_acceleration_limit', 4.0)

        self.linear_acceleration_limit = self.get_parameter('linear_acceleration_limit').value
        self.angular_acceleration_limit = self.get_parameter('angular_acceleration_limit').value

        self.start()

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        # Limit velocities
        self.target_linear_velocity = max(
            -self.max_linear_velocity,
            min(self.max_linear_velocity, msg.linear.x)
        )
        self.target_angular_velocity = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, msg.angular.z)
        )

        self.control_logger.debug(
            f'Received cmd_vel: linear={self.target_linear_velocity:.3f}, '
            f'angular={self.target_angular_velocity:.3f}'
        )

    def update(self):
        """Update robot kinematics and odometry"""
        dt = 1.0 / self.update_frequency

        # Apply acceleration limits
        linear_acc = (self.target_linear_velocity - self.x) / dt
        angular_acc = (self.target_angular_velocity - self.theta) / dt

        if abs(linear_acc) > self.linear_acceleration_limit:
            self.target_linear_velocity = self.x + \
                np.sign(linear_acc) * self.linear_acceleration_limit * dt

        if abs(angular_acc) > self.angular_acceleration_limit:
            self.target_angular_velocity = self.theta + \
                np.sign(angular_acc) * self.angular_acceleration_limit * dt

        # Differential drive kinematics
        v = self.target_linear_velocity
        omega = self.target_angular_velocity

        # Wheel velocities
        self.left_wheel_velocity = (v - omega * self.wheel_base / 2) / self.wheel_radius
        self.right_wheel_velocity = (v + omega * self.wheel_base / 2) / self.wheel_radius

        # Update odometry (simple integration)
        self.x += v * dt * math.cos(self.theta)
        self.y += v * dt * math.sin(self.theta)
        self.theta += omega * dt

        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish joint states
        self.publish_joint_states()

        # Publish odometry
        self.publish_odometry()

    def publish_joint_states(self):
        """Publish joint state messages"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [0.0, 0.0]  # Would integrate from velocity
        joint_state.velocity = [self.left_wheel_velocity, self.right_wheel_velocity]
        joint_state.effort = [0.0, 0.0]

        self.joint_state_publisher.publish(joint_state)

    def publish_odometry(self):
        """Publish odometry information"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = q

        # Twist
        odom_msg.twist.twist.linear.x = self.target_linear_velocity
        odom_msg.twist.twist.angular.z = self.target_angular_velocity

        # Covariance (example values)
        odom_msg.pose.covariance = [0.1]*36
        odom_msg.twist.covariance = [0.1]*36

        self.odom_publisher.publish(odom_msg)

        # Publish TF transform
        self.publish_odom_transform()

    def publish_odom_transform(self):
        """Publish odom -> base_link transform"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        q = self.euler_to_quaternion(0, 0, self.theta)
        transform.transform.rotation = q

        self.tf_broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q
```

## 4.4 Service Implementation

### 4.4.1 Robot Status Service

```python
from example_interfaces.srv import AddTwoInts
from custom_interfaces.srv import GetRobotStatus, SetParameter

class RobotStatusService(Node):
    """Node providing robot status services"""

    def __init__(self):
        super().__init__('robot_status_service')

        # Robot status
        self.robot_status = {
            'operational': True,
            'battery_level': 0.85,
            'error_count': 0,
            'uptime': 0.0,
            'last_maintenance': '2024-01-01'
        }

        # Services
        self.get_status_service = self.create_service(
            GetRobotStatus, 'get_robot_status',
            self.get_status_callback
        )

        self.set_param_service = self.create_service(
            SetParameter, 'set_parameter',
            self.set_parameter_callback
        )

        self.math_service = self.create_service(
            AddTwoInts, 'add_two_ints',
            self.add_two_ints_callback
        )

        # Status update timer
        self.status_timer = self.create_timer(
            1.0, self.update_status
        )

        self.start_time = self.get_clock().now()

        self.get_logger().info('Robot status service initialized')

    def update_status(self):
        """Update internal robot status"""
        self.robot_status['uptime'] = (
            self.get_clock().now() - self.start_time
        ).to_msg().sec

        # Simulate battery drain
        self.robot_status['battery_level'] *= 0.9999

    def get_status_callback(self, request, response):
        """Handle get robot status request"""
        self.get_logger().info('Received robot status request')

        response.operational = self.robot_status['operational']
        response.battery_level = self.robot_status['battery_level']
        response.error_count = self.robot_status['error_count']
        response.uptime = self.robot_status['uptime']
        response.last_maintenance = self.robot_status['last_maintenance']

        return response

    def set_parameter_callback(self, request, response):
        """Handle set parameter request"""
        self.get_logger().info(
            f'Setting parameter {request.parameter_name} = {request.value}'
        )

        try:
            # Validate parameter
            if request.parameter_name in self.robot_status:
                old_value = self.robot_status[request.parameter_name]
                self.robot_status[request.parameter_name] = request.value

                response.success = True
                response.message = f'Parameter {request.parameter_name} updated from {old_value} to {request.value}'

                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f'Unknown parameter: {request.parameter_name}'

        except Exception as e:
            response.success = False
            response.message = f'Error setting parameter: {str(e)}'

            self.get_logger().error(response.message)

        return response

    def add_two_ints_callback(self, request, response):
        """Simple math service example"""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'{request.a} + {request.b} = {response.sum}'
        )
        return response
```

## 4.5 Action Server Implementation

### 4.5.1 Navigation Action Server

```python
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse

class NavigationActionServer(Node):
    """Action server for robot navigation"""

    def __init__(self):
        super().__init__('navigation_action_server')

        # Navigation state
        self.current_goal = None
        self.goal_handle = None
        self.is_navigating = False

        # Robot position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Navigation parameters
        self.declare_parameter('linear_velocity', 0.5)
        self.declare_parameter('angular_velocity', 1.0)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.1)

        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # Create action server
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publishers for control
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 'cmd_vel', 10
        )

        # Update timer
        self.update_timer = self.create_timer(
            0.1, self.navigation_update
        )

        self.get_logger().info('Navigation action server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject new goal requests"""
        self.get_logger().info('Received new navigation goal')

        # Check if robot is already navigating
        if self.is_navigating:
            self.get_logger().warn('Already navigating, rejecting new goal')
            return GoalResponse.REJECT

        # Validate goal
        if not self.is_valid_goal(goal_request.pose):
            self.get_logger().warn('Invalid goal, rejecting')
            return GoalResponse.REJECT

        self.get_logger().info('Goal accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation requests"""
        self.get_logger().info('Received cancel request')

        if goal_handle == self.goal_handle:
            self.is_navigating = False
            self.stop_robot()
            return CancelResponse.ACCEPT

        return CancelResponse.REJECT

    def async def execute_callback(self, goal_handle):
        """Execute navigation goal"""
        self.get_logger().info('Executing navigation goal')

        self.current_goal = goal_handle.request.pose
        self.goal_handle = goal_handle
        self.is_navigating = True

        # Initialize feedback
        feedback_msg = NavigateToPose.Feedback()

        # Navigation loop
        while self.is_navigating:
            if not self.goal_handle.is_active:
                self.get_logger().info('Goal became inactive, aborting')
                break

            # Check if goal reached
            if self.is_goal_reached():
                self.get_logger().info('Goal reached!')
                break

            # Update feedback
            feedback_msg.current_pose = self.get_current_pose()
            feedback_msg.distance_remaining = self.get_distance_to_goal()

            goal_handle.publish_feedback(feedback_msg)

            # Control update
            self.update_navigation_control()

            # Rate limiting
            await asyncio.sleep(0.1)

        # Set final result
        result = NavigateToPose.Result()

        if self.is_goal_reached():
            goal_handle.succeed()
            result.result = NavigateToPose.Result().SUCCESS
        else:
            goal_handle.abort()
            result.result = NavigateToPose.Result().FAILURE

        self.is_navigating = False
        self.stop_robot()

        return result

    def navigation_update(self):
        """Periodic navigation update"""
        if not self.is_navigating:
            # Simulate position updates when not navigating
            self.current_x += random.uniform(-0.01, 0.01)
            self.current_y += random.uniform(-0.01, 0.01)
            self.current_theta += random.uniform(-0.01, 0.01)

    def update_navigation_control(self):
        """Update control commands for navigation"""
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y

        # Calculate error
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance_error = math.sqrt(dx**2 + dy**2)

        # Calculate desired angle
        desired_angle = math.atan2(dy, dx)
        angle_error = desired_angle - self.current_theta

        # Normalize angle error
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Create control command
        cmd = Twist()

        # Simple P controller
        if distance_error > self.goal_tolerance:
            # Proportional control
            cmd.linear.x = self.linear_velocity * min(1.0, distance_error / 1.0)
            cmd.angular.z = self.angular_velocity * angle_error
        else:
            # Orientation control only
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_velocity * angle_error

        # Publish command
        self.cmd_vel_publisher.publish(cmd)

    def is_goal_reached(self):
        """Check if navigation goal is reached"""
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance_error = math.sqrt(dx**2 + dy**2)

        return distance_error < self.goal_tolerance

    def get_distance_to_goal(self):
        """Calculate distance to goal"""
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y

        return math.sqrt(dx**2 + dy**2)

    def get_current_pose(self):
        """Get current robot pose"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.position.x = self.current_x
        pose.pose.position.y = self.current_y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = Quaternion()
        q.w = math.cos(self.current_theta / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.current_theta / 2)

        pose.pose.orientation = q

        return pose

    def is_valid_goal(self, goal_pose):
        """Validate navigation goal"""
        # Check for NaN or infinite values
        pos = goal_pose.pose.position
        if (math.isnan(pos.x) or math.isnan(pos.y) or
            math.isinf(pos.x) or math.isinf(pos.y)):
            return False

        # Check if goal is within reasonable bounds
        max_distance = 100.0
        distance = math.sqrt(pos.x**2 + pos.y**2)

        return distance <= max_distance

    def stop_robot(self):
        """Stop robot motion"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
```

## 4.6 Node Coordination and Launch

### 4.6.1 Multi-Node Launch File

```python
# launch/robot_system.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_1',
        description='Robot identifier'
    )

    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='true',
        description='Enable simulation mode'
    )

    # Get package directory
    package_dir = get_package_share_directory('my_robot_package')

    # Node definitions
    camera_driver_node = Node(
        package='my_robot_package',
        executable='camera_driver',
        name='camera_driver',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            os.path.join(package_dir, 'config', 'camera_params.yaml'),
            {
                'debug_mode': LaunchConfiguration('simulation_mode')
            }
        ],
        output='screen'
    )

    lidar_driver_node = Node(
        package='my_robot_package',
        executable='lidar_driver',
        name='lidar_driver',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {
                'simulation_mode': LaunchConfiguration('simulation_mode')
            }
        ],
        output='screen'
    )

    drive_controller_node = Node(
        package='my_robot_package',
        executable='drive_controller',
        name='drive_controller',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            os.path.join(package_dir, 'config', 'drive_params.yaml')
        ],
        output='screen'
    )

    navigation_server_node = Node(
        package='my_robot_package',
        executable='navigation_server',
        name='navigation_server',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('simulation_mode')
            }
        ],
        output='screen'
    )

    status_service_node = Node(
        package='my_robot_package',
        executable='status_service',
        name='status_service',
        namespace=LaunchConfiguration('robot_name'),
        output='screen'
    )

    # Optional nodes based on mode
    nodes_to_launch = [
        robot_name_arg,
        simulation_mode_arg,
        camera_driver_node,
        lidar_driver_node,
        drive_controller_node,
        navigation_server_node,
        status_service_node
    ]

    return LaunchDescription(nodes_to_launch)
```

## 4.7 Testing and Debugging

### 4.7.1 Node Testing Framework

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TestRobotNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Test data
        self.received_messages = []

    def tearDown(self):
        self.node.destroy_node()
        self.executor.shutdown()

    def test_camera_driver_node(self):
        """Test camera driver node functionality"""
        # Create subscriber to test camera output
        def image_callback(msg):
            self.received_messages.append(msg)

        subscription = self.node.create_subscription(
            Image, 'camera/image_raw',
            image_callback, 10
        )

        # Launch camera driver node
        camera_node = CameraDriverNode()
        self.executor.add_node(camera_node)

        # Run for a short time
        timeout = 5.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            self.executor.spin_once(timeout_sec=0.1)

            if len(self.received_messages) > 0:
                break

        # Verify camera images are published
        self.assertGreater(len(self.received_messages), 0)

        # Check image format
        if self.received_messages:
            image_msg = self.received_messages[0]
            self.assertIsInstance(image_msg, Image)
            self.assertGreater(image_msg.width, 0)
            self.assertGreater(image_msg.height, 0)

        # Cleanup
        camera_node.shutdown()
        self.executor.remove_node(camera_node)

    def test_drive_controller(self):
        """Test differential drive controller"""
        # Create publishers and subscribers
        cmd_vel_pub = self.node.create_publisher(
            Twist, 'cmd_vel', 10
        )

        def odom_callback(msg):
            self.received_messages.append(msg)

        odom_sub = self.node.create_subscription(
            Odometry, 'odom',
            odom_callback, 10
        )

        # Create controller
        controller = DifferentialDriveController()
        self.executor.add_node(controller)

        # Send command
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 0.5
        cmd_vel_pub.publish(cmd)

        # Run for some time
        for _ in range(50):
            self.executor.spin_once(timeout_sec=0.1)

        # Verify odometry updates
        self.assertGreater(len(self.received_messages), 0)

        if self.received_messages:
            odom = self.received_messages[-1]
            # Robot should have moved
            self.assertNotEqual(odom.pose.pose.position.x, 0.0)
            self.assertNotEqual(odom.pose.pose.position.y, 0.0)

        # Cleanup
        controller.shutdown()
        self.executor.remove_node(controller)
```

### 4.7.2 Debugging Tools and Techniques

```python
class DebugHelper:
    """Helper class for debugging ROS 2 nodes"""

    def __init__(self, node):
        self.node = node
        self.debug_topics = {}
        self.message_counts = {}

    def monitor_topic(self, topic_name, msg_type):
        """Monitor topic message flow"""
        self.debug_topics[topic_name] = {
            'count': 0,
            'last_received': None,
            'last_message': None
        }

        def callback(msg):
            self.debug_topics[topic_name]['count'] += 1
            self.debug_topics[topic_name]['last_received'] = time.time()
            self.debug_topics[topic_name]['last_message'] = msg

            # Log at reasonable rate
            if self.debug_topics[topic_name]['count'] % 10 == 0:
                self.node.get_logger().debug(
                    f'Topic {topic_name}: {self.debug_topics[topic_name]["count"]} messages'
                )

        subscription = self.node.create_subscription(
            msg_type, topic_name, callback, 10
        )

        return subscription

    def print_topic_stats(self):
        """Print debugging statistics for monitored topics"""
        self.node.get_logger().info('=== Topic Debug Statistics ===')

        for topic_name, stats in self.debug_topics.items():
            if stats['last_received']:
                time_since_last = time.time() - stats['last_received']
                self.node.get_logger().info(
                    f'{topic_name}: {stats["count"]} messages, '
                    f'last: {time_since_last:.2f}s ago'
                )
            else:
                self.node.get_logger().info(f'{topic_name}: 0 messages')

    def check_message_frequency(self, topic_name, expected_freq, tolerance=0.1):
        """Check if topic frequency is as expected"""
        if topic_name not in self.debug_topics:
            return False

        stats = self.debug_topics[topic_name]
        if not stats['last_received']:
            return False

        time_window = 5.0  # Check last 5 seconds
        messages_in_window = 0

        # This would need actual timestamps from messages
        # Simplified version

        actual_freq = messages_in_window / time_window

        return abs(actual_freq - expected_freq) <= tolerance
```

## 4.8 Performance Optimization

### 4.8.1 Efficient Message Processing

```python
class HighPerformanceNode(Node):
    """Node optimized for high-frequency message processing"""

    def __init__(self):
        super().__init__('high_performance_node')

        # Use single-threaded executor for better performance
        self.use_sim_time = False

        # Pre-allocate message objects
        self.output_msg = String()
        self.preallocated_buffer_size = 1000

        # Use custom QoS for performance
        high_perf_qos = rclpy.qos.QoSProfile(
            depth=100,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            deadline=rclpy.qos.Duration(seconds=0, nanoseconds=10000000)
        )

        self.publisher = self.create_publisher(
            String, 'high_freq_topic', high_perf_qos
        )

        self.subscription = self.create_subscription(
            String, 'input_topic',
            self.process_callback, high_perf_qos
        )

        # Profiling
        self.processing_times = []
        self.max_samples = 1000

    def process_callback(self, msg):
        """Optimized message processing callback"""
        start_time = time.perf_counter()

        # Process message efficiently
        self.process_message_fast(msg)

        # Profile if needed
        if len(self.processing_times) < self.max_samples:
            processing_time = time.perf_counter() - start_time
            self.processing_times.append(processing_time)

    def process_message_fast(self, msg):
        """Fast message processing without allocations"""
        # Reuse pre-allocated message
        self.output_msg.data = f'Processed: {msg.data}'

        # Publish
        self.publisher.publish(self.output_msg)

    def get_performance_stats(self):
        """Get processing performance statistics"""
        if not self.processing_times:
            return None

        avg_time = sum(self.processing_times) / len(self.processing_times)
        max_time = max(self.processing_times)
        min_time = min(self.processing_times)

        return {
            'avg_processing_time_ms': avg_time * 1000,
            'max_processing_time_ms': max_time * 1000,
            'min_processing_time_ms': min_time * 1000,
            'message_rate': len(self.processing_times) / 10.0  # Assuming 10s window
        }
```

## 4.9 Summary

This chapter provided a comprehensive guide to building ROS 2 nodes with Python:

1. **Node Templates**: Reusable base classes for common patterns
2. **Sensor Drivers**: Camera and LiDAR integration examples
3. **Control Systems**: Differential drive controller implementation
4. **Service Implementation**: Robot status and parameter services
5. **Action Servers**: Navigation with feedback and cancellation
6. **Launch Systems**: Multi-node coordination and configuration
7. **Testing**: Unit tests and debugging techniques
8. **Performance**: Optimization for high-frequency operations

Key takeaways for developing robust ROS 2 nodes:

- Use proper error handling and logging throughout
- Implement graceful shutdown and resource cleanup
- Choose appropriate QoS profiles for your use case
- Test thoroughly with both simulation and hardware
- Monitor performance and optimize critical paths
- Use parameter files for configuration management

The examples and patterns presented here provide a solid foundation for building production-ready ROS 2 applications for robotics systems.

## Knowledge Check

### Multiple Choice Questions

1. **What is the purpose of rclpy in ROS 2?**
   - a) To provide C++ bindings for ROS
   - b) To provide Python bindings for ROS 2 client library
   - c) To implement real-time control
   - d) To handle hardware abstraction

2. **When should you use an Action Server instead of a Service?**
   - a) For quick request-response operations
   - b) For long-running tasks with feedback
   - c) For high-frequency data streaming
   - d) For parameter configuration

3. **What is the purpose of QoS profiles in ROS 2?**
   - a) To secure communication channels
   - b) To configure data reliability and delivery characteristics
   - c) To control node execution order
   - d) To define message serialization format

### Short Answer Questions

1. Explain the difference between synchronous and asynchronous operations in ROS 2 Python.

2. Describe the advantages of using a base class template for ROS 2 nodes.

3. How can you optimize a ROS 2 node for high-frequency message processing?

### Practical Exercise

Create a ROS 2 node that:
1. Subscribes to camera and LiDAR data
2. Implements a simple object detection algorithm
3. Publishes detected objects as visualization markers
4. Provides a service to enable/disable detection

Write the complete node implementation with proper error handling and documentation.

---

*Next Chapter: Launch Systems & Parameter Management - We'll explore advanced launch systems, parameter handling, and configuration management for complex robotics applications.*