'use client';

import { useState } from 'react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Switch } from '@/components/ui/switch';
import { useROS2 } from '@/hooks/useROS2';
import { Play, Square, RotateCw, Wifi, WifiOff } from 'lucide-react';

interface LaunchConfig {
  id: string;
  name: string;
  description: string;
  command: string;
  package?: string;
  launchFile?: string;
  parameters?: {
    name: string;
    label: string;
    type: 'string' | 'boolean' | 'number';
    default: any;
  }[];
}

const launchConfigs: LaunchConfig[] = [
  {
    id: 'hardware_bringup',
    name: 'Hardware Layer',
    description: 'Motors, controllers, sensors',
    command: 'ros2 launch',
    package: 'mecanum_hardware',
    launchFile: 'bringup.launch.py',
    parameters: [
      { name: 'use_sim', label: 'use_sim', type: 'boolean', default: false },
      { name: 'serial_port', label: 'serial_port', type: 'string', default: '/dev/ttyUSB0' },
      { name: 'lidar_port', label: 'lidar_port', type: 'string', default: '/dev/ttyUSB2' },
    ],
  },
  {
    id: 'cartographer_slam',
    name: 'Mapping Mode',
    description: 'Cartographer SLAM',
    command: 'ros2 launch',
    package: 'navigation_setup',
    launchFile: 'cartographer.launch.py',
    parameters: [],
  },
  {
    id: 'nav2_navigation',
    name: 'Navigation Mode',
    description: 'Nav2 path planning',
    command: 'ros2 launch',
    package: 'navigation_setup',
    launchFile: 'nav2_with_map.launch.py',
    parameters: [
      { name: 'map_file', label: 'map_file', type: 'string', default: '/root/ros2_ws/maps/warehouse.yaml' },
    ],
  },
  {
    id: 'qr_detection',
    name: 'QR Detection',
    description: 'Camera and QR code detection',
    command: 'ros2 launch',
    package: 'warehouse_rover_qr_detection',
    launchFile: 'qr_detection.launch.py',
    parameters: [
      { name: 'camera_index', label: 'camera_index', type: 'number', default: 0 },
    ],
  },
  {
    id: 'waypoint_follower',
    name: 'Waypoint Navigation',
    description: 'Autonomous waypoint following',
    command: 'ros2 run',
    package: 'mecanum_hardware',
    launchFile: 'nav2_waypoint_follower.py',
    parameters: [],
  },
];

export default function SimpleLaunchControl() {
  const { 
    connected, 
    launch, 
    stopLaunch, 
    controlActuator,
    enableQRDetection,
    qrDetections,
    publishCmdVel,
    sendNavGoal,
  } = useROS2();
  const [launchStates, setLaunchStates] = useState<Record<string, 'stopped' | 'running'>>({});
  const [parameters, setParameters] = useState<Record<string, Record<string, any>>>({});

  // Initialize parameters with defaults
  const getParameters = (configId: string) => {
    if (!parameters[configId]) {
      const config = launchConfigs.find(c => c.id === configId);
      if (config?.parameters) {
        const defaultParams: Record<string, any> = {};
        config.parameters.forEach(p => {
          defaultParams[p.name] = p.default;
        });
        return defaultParams;
      }
    }
    return parameters[configId] || {};
  };

  const updateParameter = (configId: string, paramName: string, value: any) => {
    setParameters(prev => ({
      ...prev,
      [configId]: {
        ...prev[configId],
        [paramName]: value,
      },
    }));
  };

  const handleLaunch = async (config: LaunchConfig) => {
    const params = getParameters(config.id);
    
    // Build launch file path (package + launch file)
    const launchFilePath = `${config.package} ${config.launchFile}`;
    
    // Build parameters object
    const launchParams: Record<string, any> = {};
    if (config.parameters) {
      config.parameters.forEach(p => {
        const value = params[p.name] ?? p.default;
        launchParams[p.name] = value;
      });
    }

    try {
      await launch(config.id, launchFilePath, launchParams);
      setLaunchStates(prev => ({ ...prev, [config.id]: 'running' }));
    } catch (error) {
      console.error('Launch failed:', error);
    }
  };

  const handleStop = async (config: LaunchConfig) => {
    try {
      await stopLaunch(config.id);
      setLaunchStates(prev => ({ ...prev, [config.id]: 'stopped' }));
    } catch (error) {
      console.error('Stop failed:', error);
    }
  };

  const handleRestart = async (config: LaunchConfig) => {
    await handleStop(config);
    setTimeout(() => handleLaunch(config), 1000);
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-6">
      {/* Header */}
      <div className="mb-8">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold text-white mb-2">WAREHOUSE ROVER</h1>
            <p className="text-slate-400">Control System v1.0</p>
          </div>
          <div className="flex items-center gap-3">
            <div className="flex items-center gap-2 px-4 py-2 rounded-lg bg-slate-800 border border-slate-700">
              {connected ? (
                <>
                  <div className="w-3 h-3 rounded-full bg-green-500 animate-pulse" />
                  <span className="text-green-400 font-medium">System Online</span>
                </>
              ) : (
                <>
                  <div className="w-3 h-3 rounded-full bg-red-500" />
                  <span className="text-red-400 font-medium">System Offline</span>
                </>
              )}
            </div>
            <div className="text-sm text-slate-400 px-4 py-2 rounded-lg bg-slate-800 border border-slate-700">
              Connection: <span className="text-cyan-400 font-mono">Stable</span>
            </div>
          </div>
        </div>
      </div>

      {/* ROS Bridge Status */}
      <Card className="mb-6 bg-slate-800/50 border-slate-700">
        <CardHeader>
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              {connected ? <Wifi className="w-5 h-5 text-green-400" /> : <WifiOff className="w-5 h-5 text-red-400" />}
              <CardTitle className="text-white">ROS Bridge Status</CardTitle>
            </div>
            <span className={`px-3 py-1 rounded-full text-sm ${connected ? 'bg-green-500/20 text-green-400' : 'bg-red-500/20 text-red-400'}`}>
              {connected ? 'Disconnected' : 'Disconnected'}
            </span>
          </div>
          <CardDescription className="text-slate-400">Real-time connection to ROS bridge server</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-2 gap-4">
            <div>
              <p className="text-sm text-slate-400">Bridge URL:</p>
              <p className="text-slate-200 font-mono text-sm">ws://localhost:9090</p>
            </div>
            <div>
              <p className="text-sm text-slate-400">Last Heartbeat:</p>
              <p className="text-slate-200 font-mono text-sm">Never</p>
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Master Controls */}
      <Card className="bg-slate-800/50 border-slate-700 mb-6">
        <CardHeader>
          <CardTitle className="text-white flex items-center gap-2">
            ⚡ Master Controls
          </CardTitle>
          <CardDescription className="text-slate-400">Control all ROS processes simultaneously</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-3 gap-3">
            <Button
              onClick={() => {
                // Launch all systems
                launchConfigs.forEach(config => handleLaunch(config));
              }}
              disabled={!connected}
              className="bg-green-600 hover:bg-green-700 text-white"
            >
              ▶️ Start All Systems
            </Button>
            <Button
              onClick={() => {
                // Stop all systems
                launchConfigs.forEach(config => handleStop(config));
              }}
              disabled={!connected}
              className="bg-red-600 hover:bg-red-700 text-white"
            >
              ⏹️ Stop All Systems
            </Button>
            <Button
              onClick={() => {
                // Emergency stop
                launchConfigs.forEach(config => handleStop(config));
                publishCmdVel(0, 0, 0);
                controlActuator('stop');
              }}
              disabled={!connected}
              className="bg-orange-600 hover:bg-orange-700 text-white"
            >
              🛑 Emergency Stop
            </Button>
          </div>
        </CardContent>
      </Card>

      {/* Quick Control Sections */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
        {/* Actuator Control */}
        <Card className="bg-slate-800/50 border-slate-700">
          <CardHeader>
            <CardTitle className="text-white flex items-center gap-2">
              🤖 Actuator Control
            </CardTitle>
            <CardDescription className="text-slate-400">Control the lift actuator</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-2 gap-3">
              <Button
                onClick={() => controlActuator('up')}
                disabled={!connected}
                className="bg-green-600 hover:bg-green-700 text-white"
              >
                ⬆️ Lift Up
              </Button>
              <Button
                onClick={() => controlActuator('down')}
                disabled={!connected}
                className="bg-blue-600 hover:bg-blue-700 text-white"
              >
                ⬇️ Lift Down
              </Button>
              <Button
                onClick={() => controlActuator('stop')}
                disabled={!connected}
                className="bg-yellow-600 hover:bg-yellow-700 text-white col-span-2"
              >
                ⏹️ Stop
              </Button>
            </div>
          </CardContent>
        </Card>

        {/* QR Detection Control */}
        <Card className="bg-slate-800/50 border-slate-700">
          <CardHeader>
            <CardTitle className="text-white flex items-center gap-2">
              📷 QR Detection
            </CardTitle>
            <CardDescription className="text-slate-400">Enable/disable QR code scanning</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-2 gap-3">
              <Button
                onClick={() => enableQRDetection(true)}
                disabled={!connected}
                className="bg-green-600 hover:bg-green-700 text-white"
              >
                ▶️ Enable
              </Button>
              <Button
                onClick={() => enableQRDetection(false)}
                disabled={!connected}
                className="bg-red-600 hover:bg-red-700 text-white"
              >
                ⏹️ Disable
              </Button>
            </div>
            {qrDetections && qrDetections.length > 0 && (
              <div className="mt-4 p-3 bg-slate-900 rounded-lg border border-slate-700">
                <p className="text-xs text-slate-400 mb-2">Latest Detection:</p>
                <p className="text-sm text-cyan-400 font-mono">{qrDetections[0].qr_data}</p>
                <p className="text-xs text-slate-500 mt-1">Confidence: {(qrDetections[0].confidence * 100).toFixed(1)}%</p>
              </div>
            )}
          </CardContent>
        </Card>
      </div>

      {/* Robot Position Control */}
      <Card className="bg-slate-800/50 border-slate-700 mb-6">
        <CardHeader>
          <CardTitle className="text-white flex items-center gap-2">
            🎯 Position Control
          </CardTitle>
          <CardDescription className="text-slate-400">Move robot to specific coordinates</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-4">
            <div>
              <label className="text-sm text-slate-300 mb-2 block">X Position</label>
              <Input
                type="number"
                step="0.1"
                defaultValue="0.0"
                id="pos-x"
                className="bg-slate-900/50 border-slate-700 text-slate-200"
              />
            </div>
            <div>
              <label className="text-sm text-slate-300 mb-2 block">Y Position</label>
              <Input
                type="number"
                step="0.1"
                defaultValue="0.0"
                id="pos-y"
                className="bg-slate-900/50 border-slate-700 text-slate-200"
              />
            </div>
            <div>
              <label className="text-sm text-slate-300 mb-2 block">Theta (rad)</label>
              <Input
                type="number"
                step="0.1"
                defaultValue="0.0"
                id="pos-theta"
                className="bg-slate-900/50 border-slate-700 text-slate-200"
              />
            </div>
          </div>
          <Button
            onClick={() => {
              const x = parseFloat((document.getElementById('pos-x') as HTMLInputElement)?.value || '0');
              const y = parseFloat((document.getElementById('pos-y') as HTMLInputElement)?.value || '0');
              const theta = parseFloat((document.getElementById('pos-theta') as HTMLInputElement)?.value || '0');
              sendNavGoal(x, y, theta);
            }}
            disabled={!connected}
            className="w-full bg-cyan-600 hover:bg-cyan-700 text-white"
          >
            🎯 Move to Position
          </Button>
        </CardContent>
      </Card>

      {/* Velocity Control */}
      <Card className="bg-slate-800/50 border-slate-700 mb-6">
        <CardHeader>
          <CardTitle className="text-white flex items-center gap-2">
            🕹️ Manual Control
          </CardTitle>
          <CardDescription className="text-slate-400">Control robot velocity with keyboard or buttons</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-3 gap-3 max-w-md mx-auto mb-4">
            <div></div>
            <Button
              onMouseDown={() => publishCmdVel(0.3, 0, 0)}
              onMouseUp={() => publishCmdVel(0, 0, 0)}
              disabled={!connected}
              className="bg-slate-700 hover:bg-slate-600 text-white h-16"
            >
              ⬆️
            </Button>
            <div></div>
            <Button
              onMouseDown={() => publishCmdVel(0, 0, 0.5)}
              onMouseUp={() => publishCmdVel(0, 0, 0)}
              disabled={!connected}
              className="bg-slate-700 hover:bg-slate-600 text-white h-16"
            >
              ↺
            </Button>
            <Button
              onMouseDown={() => publishCmdVel(-0.3, 0, 0)}
              onMouseUp={() => publishCmdVel(0, 0, 0)}
              disabled={!connected}
              className="bg-slate-700 hover:bg-slate-600 text-white h-16"
            >
              ⬇️
            </Button>
            <Button
              onMouseDown={() => publishCmdVel(0, 0, -0.5)}
              onMouseUp={() => publishCmdVel(0, 0, 0)}
              disabled={!connected}
              className="bg-slate-700 hover:bg-slate-600 text-white h-16"
            >
              ↻
            </Button>
          </div>
          <div className="text-center text-sm text-slate-400">
            Hold button to move, release to stop
          </div>
        </CardContent>
      </Card>

      {/* Launch Controls */}
      <div className="grid gap-6">
        {launchConfigs.map((config) => {
          const isRunning = launchStates[config.id] === 'running';
          const params = getParameters(config.id);

          return (
            <Card key={config.id} className="bg-slate-800/50 border-slate-700">
              <CardHeader>
                <div className="flex items-center justify-between">
                  <div>
                    <CardTitle className="text-white text-xl">{config.name}</CardTitle>
                    <CardDescription className="text-slate-400">{config.description}</CardDescription>
                  </div>
                  <div className={`px-4 py-2 rounded-lg text-sm font-medium ${
                    isRunning 
                      ? 'bg-green-500/20 text-green-400' 
                      : 'bg-slate-700 text-slate-400'
                  }`}>
                    {isRunning ? 'Running' : 'Not Running'}
                  </div>
                </div>
              </CardHeader>
              <CardContent>
                {/* Parameters */}
                {config.parameters && config.parameters.length > 0 && (
                  <div className="mb-6">
                    <h4 className="text-cyan-400 font-semibold mb-4">Parameters</h4>
                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                      {config.parameters.map((param) => (
                        <div key={param.name} className="space-y-2">
                          <label className="text-sm text-slate-300">{param.label}</label>
                          {param.type === 'boolean' ? (
                            <div className="flex items-center gap-3 p-3 bg-slate-900/50 rounded-lg border border-slate-700">
                              <Switch
                                checked={params[param.name] ?? param.default}
                                onCheckedChange={(checked) => updateParameter(config.id, param.name, checked)}
                                disabled={isRunning}
                              />
                              <span className="text-slate-300">
                                {params[param.name] ?? param.default ? 'Enabled' : 'Disabled'}
                              </span>
                            </div>
                          ) : (
                            <Input
                              type={param.type === 'number' ? 'number' : 'text'}
                              value={params[param.name] ?? param.default}
                              onChange={(e) => updateParameter(config.id, param.name, e.target.value)}
                              disabled={isRunning}
                              className="bg-slate-900/50 border-slate-700 text-slate-200"
                            />
                          )}
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {/* Command Display */}
                <div className="mb-4 p-3 bg-slate-900 rounded-lg border border-slate-700">
                  <p className="text-xs text-slate-500 mb-1">Command:</p>
                  <code className="text-sm text-cyan-400 font-mono">
                    {config.command} {config.package} {config.launchFile}
                  </code>
                </div>

                {/* Action Buttons */}
                <div className="flex gap-3">
                  <Button
                    onClick={() => handleLaunch(config)}
                    disabled={isRunning || !connected}
                    className="flex-1 bg-green-600 hover:bg-green-700 text-white"
                  >
                    <Play className="w-4 h-4 mr-2" />
                    Launch
                  </Button>
                  <Button
                    onClick={() => handleStop(config)}
                    disabled={!isRunning || !connected}
                    className="flex-1 bg-red-600 hover:bg-red-700 text-white"
                  >
                    <Square className="w-4 h-4 mr-2" />
                    Stop
                  </Button>
                  <Button
                    onClick={() => handleRestart(config)}
                    disabled={!isRunning || !connected}
                    className="bg-cyan-600 hover:bg-cyan-700 text-white"
                  >
                    <RotateCw className="w-4 h-4" />
                  </Button>
                </div>
              </CardContent>
            </Card>
          );
        })}
      </div>
    </div>
  );
}
