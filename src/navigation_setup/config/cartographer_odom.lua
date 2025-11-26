include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- ODOMETRY MODE SETTINGS (Sensor Fusion Only)
  -- In navigation mode, AMCL publishes map->odom
  -- Cartographer publishes odom->base_link with IMU+wheel fusion
  map_frame = "odom",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,  -- Publish odom->base_link (NOT map frame!)
  publish_frame_projected_to_2d = true,
  use_odometry = true,  -- Fuse with wheel odometry
  use_nav_sat = false,
  use_landmarks = false,

  -- SENSOR CONFIGURATION 
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,     
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- TIMINGS
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- TRAJECTORY BUILDER TUNING
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 15.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 16.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- MAP BUILDER TUNING
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2

return options