// hooks/useROS2.ts
import { useEffect, useRef, useState, useCallback } from 'react';

export interface ROS2Message {
  type: string;
  topic?: string;
  data: any;
  timestamp: string;
}

export interface SystemStatus {
  active_nodes: number;
  active_topics: number;
  launch_processes: string[];
  bridge_uptime: number;
}

export interface QRDetection {
  qr_data: string;
  rack_id: string;
  shelf_id: string;
  item_code: string;
  confidence: number;
  is_valid: boolean;
  error_message: string;
}

export interface RobotPose {
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
}

export function useROS2(url: string = 'ws://localhost:9090') {
  const ws = useRef<WebSocket | null>(null);
  const [connected, setConnected] = useState(false);
  const [systemStatus, setSystemStatus] = useState<SystemStatus | null>(null);
  const [missionStatus, setMissionStatus] = useState<string>('');
  const [waypointStatus, setWaypointStatus] = useState<string>('');
  const [actuatorStatus, setActuatorStatus] = useState<string>('');
  const [qrDetections, setQrDetections] = useState<QRDetection[]>([]);
  const [robotPose, setRobotPose] = useState<RobotPose | null>(null);
  const [lidarData, setLidarData] = useState<any>(null);
  const [laserDistance, setLaserDistance] = useState<number>(0);

  // Connect to WebSocket
  useEffect(() => {
    const connect = () => {
      try {
        ws.current = new WebSocket(url);

        ws.current.onopen = () => {
          console.log('Connected to ROS2 WebSocket Bridge');
          setConnected(true);
        };

        ws.current.onclose = () => {
          console.log('Disconnected from ROS2 WebSocket Bridge');
          setConnected(false);
          // Reconnect after 3 seconds
          setTimeout(connect, 3000);
        };

        ws.current.onerror = (error) => {
          console.error('WebSocket error:', error);
        };

        ws.current.onmessage = (event) => {
          try {
            const message: ROS2Message = JSON.parse(event.data);
            handleMessage(message);
          } catch (error) {
            console.error('Error parsing message:', error);
          }
        };
      } catch (error) {
        console.error('Error connecting to WebSocket:', error);
        setTimeout(connect, 3000);
      }
    };

    connect();

    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, [url]);

  // Handle incoming messages
  const handleMessage = (message: ROS2Message) => {
    switch (message.type) {
      case 'system_status':
        setSystemStatus(message.data);
        break;
      
      case 'topic':
        handleTopicMessage(message);
        break;
      
      case 'connection':
        console.log('Connection status:', message);
        break;
      
      default:
        console.log('Unknown message type:', message.type);
    }
  };

  // Handle topic messages
  const handleTopicMessage = (message: ROS2Message) => {
    switch (message.topic) {
      case 'mission_status':
        setMissionStatus(message.data.data);
        break;
      
      case 'waypoint_status':
        setWaypointStatus(message.data.data);
        break;
      
      case 'actuator_status':
        setActuatorStatus(message.data.data);
        break;
      
      case 'qr_detections':
        setQrDetections(message.data.detections);
        break;
      
      case 'odometry':
        setRobotPose({
          position: message.data.position,
          orientation: message.data.orientation
        });
        break;
      
      case 'lidar':
        setLidarData(message.data);
        break;
      
      case 'laser_distance':
        setLaserDistance(message.data.data);
        break;
      
      default:
        break;
    }
  };

  // Send command to ROS2
  const sendCommand = useCallback((action: string, data: any = {}) => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify({ action, ...data }));
    } else {
      console.error('WebSocket not connected');
    }
  }, []);

  // Control robot velocity
  const publishCmdVel = useCallback((linear_x: number, linear_y: number, angular_z: number) => {
    sendCommand('publish_cmd_vel', { linear_x, linear_y, angular_z });
  }, [sendCommand]);

  // Control actuator
  const controlActuator = useCallback((command: 'up' | 'down' | 'stop') => {
    sendCommand('actuator_command', { command });
  }, [sendCommand]);

  // Enable/disable QR detection
  const enableQRDetection = useCallback((enable: boolean) => {
    sendCommand('qr_enable', { enable });
  }, [sendCommand]);

  // Launch ROS2 launch file
  const launch = useCallback((launchName: string, launchFile: string, params: any = {}) => {
    sendCommand('launch', { launch_name: launchName, launch_file: launchFile, params });
  }, [sendCommand]);

  // Stop launch
  const stopLaunch = useCallback((launchName: string) => {
    sendCommand('stop_launch', { launch_name: launchName });
  }, [sendCommand]);

  // Send navigation goal
  const sendNavGoal = useCallback((x: number, y: number, theta: number) => {
    sendCommand('nav_goal', { x, y, theta });
  }, [sendCommand]);

  // Cancel navigation
  const cancelNavigation = useCallback(() => {
    sendCommand('cancel_nav');
  }, [sendCommand]);

  // Get waypoints
  const getWaypoints = useCallback(() => {
    sendCommand('get_waypoints');
  }, [sendCommand]);

  return {
    connected,
    systemStatus,
    missionStatus,
    waypointStatus,
    actuatorStatus,
    qrDetections,
    robotPose,
    lidarData,
    laserDistance,
    publishCmdVel,
    controlActuator,
    enableQRDetection,
    sendNavGoal,
    cancelNavigation,
    launch,
    stopLaunch,
    getWaypoints,
  };
}
