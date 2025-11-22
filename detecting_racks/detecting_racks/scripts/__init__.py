def __init__(self, map_config, blob_config, rect_config):
    super().__init__('table_detector')

    # ... other init code ...

    # --- NEW ---
    # Declare a parameter for the map file path.
    # Default value is 'map.pgm' (will be treated as relative to CWD)
    self.declare_parameter('map_file_path', 'map.pgm')
    # --- END NEW ---

    self.get_logger().info("ROS 2 Table Detector node initialized.")