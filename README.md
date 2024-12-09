# IMU and GPS Sensor Fusion with ROS 2

## Overview

This project focuses on integrating and analyzing IMU and GPS data for precise navigation using sensor fusion techniques. It includes ROS 2 packages for real-time IMU and GPS data collection, custom message formats, and MATLAB-based post-processing for calibration and analysis.

The project addresses challenges like sensor drift, noise, and distortions, applying advanced techniques like magnetometer calibration, velocity estimation, and dead reckoning.

---

## Features

- **ROS 2 Integration**: 
  - Custom drivers for real-time IMU and GPS data acquisition.
  - ROS 2 messages (`imu_msgs` and `gps_msgs`) tailored for efficient communication.

- **Sensor Calibration**:
  - Magnetometer calibration for hard-iron and soft-iron distortions.
  - Bias correction for IMU accelerometers to reduce drift during velocity integration.

- **Sensor Fusion**:
  - Combines gyroscope and magnetometer data for robust yaw estimation using a complementary filter.
  - Integrates GPS and IMU-derived velocities for improved accuracy.

- **Analysis Tools**:
  - MATLAB scripts for detailed data analysis, including Allan Variance and trajectory comparisons.
  - Python utilities to convert ROS 2 bag data to MATLAB-readable formats.

---

## Repository Structure

- **`src/`**: Contains ROS 2 packages.
  - **`imu_msgs/`**: Custom message definitions for IMU data.
  - **`gps_msgs/`**: Custom message definitions for GPS data.
  - **`nav_driver/`**: A ROS 2 driver for simultaneous data collection from IMU and GPS sensors.

- **`data/`**: Includes raw datasets and processed data files.
  - `data_driving.mat`: Processed IMU and GPS data from a driving test.
  - `data_going_in_circles.mat`: Processed IMU and GPS data from a circular driving test.

- **`scripts/`**: Python and MATLAB utilities.
  - `bag_matlab.py`: Converts ROS 2 bag files to MATLAB `.mat` files.
  - `magnetometer_calibration.mlx`: MATLAB script for magnetometer calibration.

- **`analysis/`**: MATLAB code for post-processing and data visualization.
  - Allan Variance computation.
  - Velocity and trajectory comparisons between IMU and GPS.

---

## Key Techniques

### Magnetometer Calibration
1. **Hard-Iron Distortion**:
   - Offsets are calculated using:
     ```math
     offset_x = (max_x + min_x) / 2
     offset_y = (max_y + min_y) / 2
     ```
   - These offsets are subtracted from raw magnetometer readings.

2. **Soft-Iron Distortion**:
   - Scaling factors are applied to correct axis-wise distortions.

3. **Yaw Calculation**:
   - Using corrected magnetometer readings:
     ```math
     yaw = atan2(magy_calibrated, magx_calibrated)
     ```
![Comparison of before and after Hard & Soft Iron Corrections] (/analysis/images/hisi_corrections.png)

### Velocity Estimation
- IMU-derived velocity is obtained by integrating forward acceleration:
  ```math
  v(t) = ∫ a(t) dt
  ```
- GPS velocity is computed by calculating the Euclidean distance between successive GPS points

### Dead Reckoning
- Position estimates are derived by integrating IMU-derived velocities over time and compared with GPS-based displacements

### Sensor Fusion
- A complementary filter combines low-pass filtered magnetometer yaw and high-pass filtered gyroscope yaw to improve robustness
![Complementary filter] (/analysis/images/complementary_filter.png)

4. **Analyze Data in MATLAB**
- Velocity estimate from the GPS with Velocity estimate from accelerometer after bias adjustment
![Bias Adjusted] (/analysis/images/vel_gps_acc.png)
- Compare trajectories with gps_imu_analysis.m
![GPU and IMU Trajectories] (/analysis/images/gps_imu_trajectory.png)


