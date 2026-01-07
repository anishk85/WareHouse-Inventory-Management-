// lib/launch-configs.ts
export interface LaunchConfig {
  id: string;
  name: string;
  description: string;
  launchFile: string;
  category: 'mapping' | 'navigation' | 'inventory' | 'testing' | 'system';
  icon: string;
  params: {
    name: string;
    label: string;
    type: 'string' | 'number' | 'boolean' | 'select';
    default: any;
    options?: string[];
    description: string;
  }[];
  status?: 'idle' | 'running' | 'error';
}

export const launchConfigs: LaunchConfig[] = [
  // SYSTEM LAUNCHES
  {
    id: 'hardware',
    name: 'Hardware Layer',
    description: 'Start robot hardware (motors, controllers, sensors)',
    launchFile: 'mecanum_hardware hardware.launch.py',
    category: 'system',
    icon: '⚙️',
    params: [],
  },
  {
    id: 'ros2_bridge',
    name: 'ROS2 Web Bridge',
    description: 'WebSocket bridge for GUI communication',
    launchFile: 'mecanum_hardware ros2_web_bridge.py',
    category: 'system',
    icon: '🌐',
    params: [],
  },

  // MAPPING
  {
    id: 'mapping',
    name: 'Create Map',
    description: 'SLAM mapping with Cartographer',
    launchFile: 'mecanum_hardware complete_mapping_navigation.launch.py',
    category: 'mapping',
    icon: '🗺️',
    params: [
      {
        name: 'mode',
        label: 'Mode',
        type: 'select',
        default: 'mapping',
        options: ['mapping'],
        description: 'Operation mode'
      },
    ],
  },

  // NAVIGATION
  {
    id: 'navigation',
    name: 'Navigate with Map',
    description: 'Autonomous navigation with Nav2',
    launchFile: 'mecanum_hardware complete_mapping_navigation.launch.py',
    category: 'navigation',
    icon: '🧭',
    params: [
      {
        name: 'mode',
        label: 'Mode',
        type: 'select',
        default: 'navigation',
        options: ['navigation'],
        description: 'Operation mode'
      },
      {
        name: 'map',
        label: 'Map File',
        type: 'string',
        default: '/tmp/warehouse_map.yaml',
        description: 'Path to map YAML file'
      },
    ],
  },
  {
    id: 'waypoint_creation',
    name: 'Create Waypoints',
    description: 'Drive manually and save waypoint positions',
    launchFile: 'mecanum_hardware waypoint_creation.launch.py',
    category: 'navigation',
    icon: '📍',
    params: [
      {
        name: 'map',
        label: 'Map File',
        type: 'string',
        default: '/tmp/warehouse_map.yaml',
        description: 'Map to use for localization'
      },
      {
        name: 'waypoints_output',
        label: 'Output File',
        type: 'string',
        default: '/tmp/waypoints.yaml',
        description: 'Where to save waypoints'
      },
    ],
  },

  // INVENTORY SYSTEM
  {
    id: 'inventory_mission',
    name: 'Inventory Mission',
    description: 'Autonomous waypoint following with QR scanning',
    launchFile: 'mecanum_hardware inventory_mission.launch.py',
    category: 'inventory',
    icon: '📦',
    params: [
      {
        name: 'map',
        label: 'Map File',
        type: 'string',
        default: '/tmp/warehouse_map.yaml',
        description: 'Warehouse map'
      },
      {
        name: 'waypoints',
        label: 'Waypoints File',
        type: 'string',
        default: '/tmp/waypoints.yaml',
        description: 'Waypoints to visit'
      },
      {
        name: 'scan_duration',
        label: 'Scan Duration (s)',
        type: 'number',
        default: 50.0,
        description: 'How long to scan at each rack'
      },
      {
        name: 'use_nav2_follower',
        label: 'Use Nav2 Follower',
        type: 'boolean',
        default: true,
        description: 'Use Nav2 waypoint follower (recommended)'
      },
    ],
  },
  {
    id: 'qr_detection',
    name: 'QR Detection Only',
    description: 'Enhanced neon QR detection with database updates',
    launchFile: 'warehouse_rover_image_processing enhanced_neon_qr_detection.launch.py',
    category: 'inventory',
    icon: '📷',
    params: [
      {
        name: 'camera_index',
        label: 'Camera Index',
        type: 'number',
        default: 0,
        description: 'USB camera device index'
      },
    ],
  },

  // TESTING
  {
    id: 'actuator_test',
    name: 'Test Actuator',
    description: 'Test scissor lift actuator controls',
    launchFile: 'mecanum_hardware actuator_control_node.py',
    category: 'testing',
    icon: '🔧',
    params: [],
  },
  {
    id: 'laser_sensor',
    name: 'Laser Distance Sensor',
    description: 'Pro-Range laser distance sensor',
    launchFile: 'mecanum_hardware laser_sensor.py',
    category: 'testing',
    icon: '📏',
    params: [],
  },
  {
    id: 'teleop',
    name: 'Keyboard Teleop',
    description: 'Control robot with keyboard',
    launchFile: 'teleop_twist_keyboard teleop_twist_keyboard',
    category: 'testing',
    icon: '⌨️',
    params: [],
  },
];

export function getLaunchConfig(id: string): LaunchConfig | undefined {
  return launchConfigs.find(config => config.id === id);
}

export function getLaunchsByCategory(category: LaunchConfig['category']): LaunchConfig[] {
  return launchConfigs.filter(config => config.category === category);
}
