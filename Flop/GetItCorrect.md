System Architecture & Data Flow
Sensor Data (D435i → Radxa)

The Intel RealSense D435i streams raw stereo infrared images and high-rate IMU data to the Radxa single-board computer over USB 3.0.
Onboard Processing (Radxa)

Visual-Inertial Odometry / SLAM (RTAB-Map) processes the stereo images and IMU on the Radxa to calculate the drone's precise position ($x, y, z$) and orientation (roll, pitch, yaw).
Grid Mapping processes the 3D point cloud into 2D occupancy grids and wall boundary vectors for navigation and safety.
External Vision Stream (Radxa → Pixhawk)

The script px4_vision_bridge.py takes the calculated /odom pose from ROS 2, converts it into MAVLink VISION_POSITION_ESTIMATE packets, and sends it over serial (/dev/pixhawk) to the Pixhawk.
Pixhawk’s EKF2 estimator ingests these vision packets as its position source (replacing GPS), enabling stable indoor position holding and autonomous flight.