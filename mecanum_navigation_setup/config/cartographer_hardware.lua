-- Copyright 2024 Mecanum Hardware
-- Cartographer configuration for hardware robot with IMU fusion
-- Optimized for mecanum drive with NEMA23 steppers + IMU + LiDAR

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = true,   -- Publish odom->base_link (with IMU fusion)
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,          -- Use wheel odometry from mecanum controller
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,  -- Longer timeout for hardware
  submap_publish_period_sec = 0.5,     -- Publish submaps less frequently
  pose_publish_period_sec = 10e-3,     -- 100Hz pose updates
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,             -- Use all IMU data
  landmarks_sampling_ratio = 1.,
}

-- ========== TRAJECTORY BUILDER 2D ==========
-- Hardware-optimized SLAM configuration

-- IMU Configuration (CRITICAL for mecanum drift correction)
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 5.0  -- Trust IMU for gravity estimation

-- LiDAR Range Configuration (RPLidar A1)
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0

-- Scan Matching (hardware needs stronger matching for noisy steppers)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres Scan Matcher (fine-tune pose)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 2

-- Motion Filter (hardware robots move more erratically)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(3.)

-- Submaps (smaller for faster processing on RPi5)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Voxel Filter (reduce computation on RPi5)
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- Adaptive Voxel Filter
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150

-- ========== MAP BUILDER ==========
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2  -- RPi5 has 4 cores, leave some for other processes

-- ========== POSE GRAPH OPTIMIZATION ==========
-- Optimize for hardware with potential wheel slip and IMU drift

POSE_GRAPH.optimization_problem.huber_scale = 5e1
POSE_GRAPH.optimize_every_n_nodes = 60
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

-- Loop Closure (important for long-term drift correction)
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Optimization Weights (trust IMU and wheel odometry)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3  -- Higher trust in wheel odometry
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3     -- Higher trust in wheel odometry
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5

-- Ceres Solver
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2

return options
