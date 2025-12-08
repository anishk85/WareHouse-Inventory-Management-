'use client';

import { useState } from 'react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Switch } from '@/components/ui/switch';
import { useROS2 } from '@/hooks/useROS2';
import { Play, Square, RotateCw, ChevronDown, ChevronUp } from 'lucide-react';

interface LaunchConfig {
  id: string;
  name: string;
  description: string;
  icon: string;
  package: string;
  launchFile: string;
  parameters?: {
    name: string;
    label: string;
    type: 'string' | 'boolean';
    default: any;
  }[];
}

const launchConfigs: LaunchConfig[] = [
  {
    id: 'hardware',
    name: 'Hardware Layer',
    description: 'Motors, controllers, sensors',
    icon: '⚙️',
    package: 'mecanum_hardware',
    launchFile: 'bringup.launch.py',
    parameters: [
      { name: 'use_sim', label: 'use_sim', type: 'boolean', default: false },
      { name: 'serial_port', label: 'serial_port', type: 'string', default: '/dev/ttyUSB0' },
      { name: 'lidar_port', label: 'lidar_port', type: 'string', default: '/dev/ttyUSB2' },
    ],
  },
  {
    id: 'mapping',
    name: 'Mapping Mode',
    description: 'Cartographer SLAM',
    icon: '🗺️',
    package: 'navigation_setup',
    launchFile: 'cartographer.launch.py',
  },
  {
    id: 'navigation',
    name: 'Navigation Mode',
    description: 'Nav2 path planning',
    icon: '🧭',
    package: 'navigation_setup',
    launchFile: 'nav2_with_map.launch.py',
  },
  {
    id: 'qr_detection',
    name: 'QR Detection',
    description: 'Camera and QR scanning',
    icon: '📷',
    package: 'warehouse_rover_qr_detection',
    launchFile: 'qr_detection.launch.py',
  },
  {
    id: 'waypoint',
    name: 'Waypoint Navigation',
    description: 'Autonomous waypoint following',
    icon: '📍',
    package: 'mecanum_hardware',
    launchFile: 'nav2_waypoint_follower.py',
  },
];

export default function SimpleUI() {
  const { connected, launch, stopLaunch } = useROS2();
  const [launchStates, setLaunchStates] = useState<Record<string, 'stopped' | 'running'>>({});
  const [expandedCards, setExpandedCards] = useState<Record<string, boolean>>({ hardware: true });
  const [parameters, setParameters] = useState<Record<string, Record<string, any>>>({});

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
    const launchFilePath = `${config.package} ${config.launchFile}`;
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

  const toggleCard = (id: string) => {
    setExpandedCards(prev => ({ ...prev, [id]: !prev[id] }));
  };

  return (
    <div className="min-h-screen bg-[#0a0e27] p-6">
      {/* Header */}
      <div className="flex items-center justify-between mb-8">
        <div className="flex items-center gap-4">
          <div className="w-12 h-12 bg-cyan-500 rounded-lg flex items-center justify-center text-white font-bold text-xl">
            WR
          </div>
          <div>
            <h1 className="text-2xl font-bold text-white">WAREHOUSE ROVER</h1>
            <p className="text-slate-400 text-sm">Control System v1.0</p>
          </div>
        </div>
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2">
            <div className={`w-2 h-2 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'}`} />
            <span className={`text-sm font-medium ${connected ? 'text-green-400' : 'text-red-400'}`}>
              {connected ? 'System Online' : 'System Offline'}
            </span>
          </div>
          <div className="text-sm text-slate-400">
            Connection: <span className="text-cyan-400">Stable</span>
          </div>
        </div>
      </div>

      {/* Navigation Sidebar (fake for now) */}
      <div className="flex gap-6">
        <div className="w-56 space-y-2">
          <div className="px-4 py-3 rounded-lg text-slate-400 hover:bg-slate-800/50 cursor-pointer flex items-center gap-3">
            <span>⚡</span> Dashboard
          </div>
          <div className="px-4 py-3 rounded-lg bg-green-500 text-white cursor-pointer flex items-center gap-3">
            <span>🚀</span> Launch Control
          </div>
          <div className="px-4 py-3 rounded-lg text-slate-400 hover:bg-slate-800/50 cursor-pointer flex items-center gap-3">
            <span>🧭</span> Navigation
          </div>
          <div className="px-4 py-3 rounded-lg text-slate-400 hover:bg-slate-800/50 cursor-pointer flex items-center gap-3">
            <span>📍</span> Waypoints
          </div>
          <div className="px-4 py-3 rounded-lg text-slate-400 hover:bg-slate-800/50 cursor-pointer flex items-center gap-3">
            <span>📦</span> Inventory
          </div>
          <div className="px-4 py-3 rounded-lg text-slate-400 hover:bg-slate-800/50 cursor-pointer flex items-center gap-3">
            <span>📊</span> System Monitor
          </div>
        </div>

        {/* Main Content */}
        <div className="flex-1">
          <h2 className="text-3xl font-bold text-white mb-2">Launch Control</h2>
          <p className="text-slate-400 mb-6">Start and manage ROS2 subsystems</p>

          <div className="space-y-4">
            {launchConfigs.map((config) => {
              const isRunning = launchStates[config.id] === 'running';
              const isExpanded = expandedCards[config.id];
              const params = getParameters(config.id);

              return (
                <div key={config.id} className="bg-[#1a1f3a] rounded-lg border border-slate-700/50">
                  <div className="p-6">
                    <div className="flex items-center justify-between mb-4">
                      <div className="flex items-center gap-4">
                        <div className="text-4xl">{config.icon}</div>
                        <div>
                          <h3 className="text-xl font-semibold text-green-400">{config.name}</h3>
                          <p className="text-slate-400 text-sm">{config.description}</p>
                        </div>
                      </div>
                      <div className="flex items-center gap-3">
                        <div className={`px-4 py-2 rounded-lg text-sm ${
                          isRunning ? 'text-green-400' : 'text-slate-400'
                        }`}>
                          <span className={`inline-block w-2 h-2 rounded-full mr-2 ${
                            isRunning ? 'bg-green-400' : 'bg-slate-600'
                          }`} />
                          {isRunning ? 'Running' : 'Not Running'}
                        </div>
                        {config.parameters && (
                          <button
                            onClick={() => toggleCard(config.id)}
                            className="p-2 hover:bg-slate-700/50 rounded-lg"
                          >
                            {isExpanded ? (
                              <ChevronUp className="w-5 h-5 text-cyan-400" />
                            ) : (
                              <ChevronDown className="w-5 h-5 text-cyan-400" />
                            )}
                          </button>
                        )}
                      </div>
                    </div>

                    {/* Parameters Section */}
                    {config.parameters && isExpanded && (
                      <div className="mb-6 pb-6 border-b border-slate-700/50">
                        <h4 className="text-cyan-400 font-semibold mb-4">Parameters</h4>
                        <div className="space-y-4">
                          {config.parameters.map((param) => (
                            <div key={param.name} className="flex items-center justify-between">
                              <label className="text-slate-300">{param.label}</label>
                              {param.type === 'boolean' ? (
                                <Switch
                                  checked={params[param.name] ?? param.default}
                                  onCheckedChange={(checked) => updateParameter(config.id, param.name, checked)}
                                  disabled={isRunning}
                                />
                              ) : (
                                <Input
                                  type="text"
                                  value={params[param.name] ?? param.default}
                                  onChange={(e) => updateParameter(config.id, param.name, e.target.value)}
                                  disabled={isRunning}
                                  className="w-64 bg-[#0a0e27] border-slate-700 text-slate-200"
                                />
                              )}
                            </div>
                          ))}
                        </div>
                      </div>
                    )}

                    {/* Action Buttons */}
                    <div className="flex gap-3">
                      <Button
                        onClick={() => handleLaunch(config)}
                        disabled={isRunning || !connected}
                        className="flex-1 bg-green-500 hover:bg-green-600 text-white h-12 text-base font-semibold"
                      >
                        Launch
                      </Button>
                      <Button
                        onClick={() => handleStop(config)}
                        disabled={!isRunning || !connected}
                        className="flex-1 bg-red-500 hover:bg-red-600 text-white h-12 text-base font-semibold"
                      >
                        Stop
                      </Button>
                      <Button
                        onClick={() => handleRestart(config)}
                        disabled={!isRunning || !connected}
                        className="bg-cyan-500 hover:bg-cyan-600 text-white h-12 px-6"
                      >
                        Restart
                      </Button>
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      </div>
    </div>
  );
}
