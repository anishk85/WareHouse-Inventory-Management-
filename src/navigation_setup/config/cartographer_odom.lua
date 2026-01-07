include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- ODOMETRY MODE SETTINGS
  map_frame = "odom",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  -- SENSOR CONFIGURATION 
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,     
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  lookup_transform_timeout_sec = 0.2,  -- Increased for stability
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ========================================
-- TRAJECTORY BUILDER 2D - TUNED FOR YDLIDAR X2
-- ========================================

-- IMU Integration (trust IMU more for rotation)
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- YDLIDAR X2 Range Settings (X2 has 8m practical range)
TRAJECTORY_BUILDER_2D.min_range = 0.12  -- X2 min: 0.12m
TRAJECTORY_BUILDER_2D.max_range = 8.0   -- X2 max: 8m (practical)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0

-- YDLIDAR X2 has ~2000 points/scan (vs RPLidar's 8000+)
-- Need to reduce noise sensitivity and increase robustness
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025  -- Reduced filtering (was implicit)

-- Online Scan Matching - RELAXED for noisy scans
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- Reduced from 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)  -- Reduced from 20°
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0  -- Much lower (was 10)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1

-- Ceres Scan Matcher - TRUST ODOMETRY MORE
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 5.0  -- Reduced from 10 (less trust in scan)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 15.0    -- Increased from 10 (trust odom more)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30.0       -- Increased from 1 (trust IMU/odom more)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 15
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- Motion Filter - Less aggressive (allow more updates)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2  -- More frequent updates
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05  -- Smaller threshold
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(3.)  -- Smaller threshold

-- Submap Configuration - Larger submaps for stability
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120  -- Increased from 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Adaptive Voxel Filter (handles varying point density)
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 8.0

-- ========================================
-- MAP BUILDER CONFIGURATION
-- ========================================
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- ========================================
-- POSE GRAPH OPTIMIZATION - DISABLED (Odometry mode)
-- ========================================

-- Loop Closure Detection - Less aggressive
POSE_GRAPH.optimize_every_n_nodes = 30  -- Increased from 20
POSE_GRAPH.constraint_builder.min_score = 0.70  -- Stricter (was 0.65)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65  -- Stricter
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.  -- Reduced from 15
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- Reduced from 0.5

-- Fast Correlative Scan Matcher
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Optimization Weights - HEAVILY TRUST ODOMETRY
POSE_GRAPH.optimization_problem.huber_scale = 5e0  -- Reduced (less outlier rejection)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2  -- Increased 10x
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e1     -- Increased 10x
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e4  -- Reduced (was 1e5)
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e4     -- Reduced (was 1e5)

-- Solver Settings
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

return options