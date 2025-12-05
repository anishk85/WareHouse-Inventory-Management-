-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- ...

-- This is a 2D configuration for a holonomic Mecanum robot
-- using IMU, wheel odometry, and a 2D Lidar.


include "map_builder.lua"
include "trajectory_builder.lua"


options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  

  -- Frames
  map_frame = "map",
  tracking_frame = "base_link",      -- frame of IMU and odometry
  published_frame = "base_link",     -- frame for Cartographer outputs
  odom_frame = "odom",                -- frame that odometry will be published in
  provide_odom_frame = true,          -- Cartographer publishes odom -> base_link TF
  publish_frame_projected_to_2d = true,
  

  -- Sensors
  use_odometry = true,
  use_imu = true,
  use_laser_scan = true,
  use_nav_sat = false,
  use_landmarks = false,


  -- Laser / Lidar settings
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- Transform lookup timeout
  lookup_transform_timeout_sec = 0.2,
}




-- 2D Local SLAM Settings
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- Global SLAM (Loop Closure) Settings
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

return options
