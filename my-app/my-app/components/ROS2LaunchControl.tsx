'use client';

import { useState, useEffect, useRef } from 'react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Switch } from '@/components/ui/switch';
import { ScrollArea } from '@/components/ui/scroll-area';
import { Play, Square, RotateCw, ChevronDown, ChevronUp, Terminal, Trash2, X } from 'lucide-react';

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

interface LogEntry {
  timestamp: string;
  launch_name: string;
  log: string;
  type: 'info' | 'error' | 'warn' | 'success';
}

const launchConfigs: LaunchConfig[] = [
  {
    id: 'hardware',
    name: 'Hardware Layer',
    description: 'Motors, controllers, sensors',
    icon: '⚙️',
    package: 'mecanum_hardware',
    launchFile: 'hardware.launch.py',
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
    package: 'mecanum_hardware',
    launchFile: 'hardware_mapping.launch.py',
  },
  {
    id: 'navigation',
    name: 'Navigation Mode',
    description: 'Nav2 path planning',
    icon: '🧭',
    package: 'mecanum_hardware',
    launchFile: 'hardware_navigation.launch.py',
    parameters: [
      { name: 'map', label: 'map_file', type: 'string', default: '/root/ros2_ws/maps/warehouse.yaml' },
    ],
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
    launchFile: 'waypoint_follower.launch.py',
  },
];

export default function ROS2LaunchControl() {
  const [connected, setConnected] = useState(false);
  const [launchStates, setLaunchStates] = useState<Record<string, 'stopped' | 'running' | 'error'>>({});
  const [expandedCards, setExpandedCards] = useState<Record<string, boolean>>({ hardware: true });
  const [parameters, setParameters] = useState<Record<string, Record<string, any>>>({});
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [showLogs, setShowLogs] = useState(true);
  const [selectedLogFilter, setSelectedLogFilter] = useState<string | null>(null);
  
  const wsRef = useRef<WebSocket | null>(null);
  const logsEndRef = useRef<HTMLDivElement>(null);

  // WebSocket connection
  useEffect(() => {
    const wsUrl = process.env.NEXT_PUBLIC_WS_URL || 'ws://localhost:9090';
    
    const connect = () => {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setConnected(true);
        addLog('system', '✅ Connected to ROS2 Bridge', 'success');
      };

      ws.onclose = () => {
        setConnected(false);
        addLog('system', '❌ Disconnected from ROS2 Bridge', 'error');
        // Reconnect after 3 seconds
        setTimeout(connect, 3000);
      };

      ws.onerror = () => {
        addLog('system', '⚠️ WebSocket error', 'error');
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          handleMessage(data);
        } catch (e) {
          console.error('Failed to parse message:', e);
        }
      };
    };

    connect();

    return () => {
      wsRef.current?.close();
    };
  }, []);

  // Auto-scroll logs
  useEffect(() => {
    logsEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [logs]);

  const handleMessage = (data: any) => {
    switch (data.type) {
      case 'connection':
        addLog('system', `🔌 ${data.status}`, 'info');
        break;
      
      case 'launch_log':
        addLog(data.launch_name, data.log, detectLogType(data.log));
        break;
      
      case 'launch_ended':
        const exitType = data.exit_code === 0 ? 'success' : 'error';
        addLog(data.launch_name, `Process ended with exit code: ${data.exit_code}`, exitType);
        setLaunchStates(prev => ({ 
          ...prev, 
          [data.launch_name]: data.exit_code === 0 ? 'stopped' : 'error' 
        }));
        break;
      
      default:
        // Handle other message types
        break;
    }
  };

  const detectLogType = (log: string): 'info' | 'error' | 'warn' | 'success' => {
    const lowerLog = log.toLowerCase();
    if (lowerLog.includes('error') || lowerLog.includes('failed') || lowerLog.includes('exception')) {
      return 'error';
    }
    if (lowerLog.includes('warn') || lowerLog.includes('warning')) {
      return 'warn';
    }
    if (lowerLog.includes('success') || lowerLog.includes('started') || lowerLog.includes('ready')) {
      return 'success';
    }
    return 'info';
  };

  const addLog = (launch_name: string, log: string, type: LogEntry['type']) => {
    setLogs(prev => [...prev.slice(-500), { // Keep last 500 logs
      timestamp: new Date().toISOString(),
      launch_name,
      log,
      type,
    }]);
  };

  const sendCommand = (action: string, data: any = {}) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ action, ...data }));
    }
  };

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
      [configId]: { ...prev[configId], [paramName]: value },
    }));
  };

  const handleLaunch = async (config: LaunchConfig) => {
    const params = getParameters(config.id);
    const launchFilePath = `${config.package} ${config.launchFile}`;
    
    addLog(config.id, `🚀 Launching: ros2 launch ${launchFilePath}`, 'info');
    
    sendCommand('launch', {
      launch_name: config.id,
      launch_file: launchFilePath,
      params,
    });
    
    setLaunchStates(prev => ({ ...prev, [config.id]: 'running' }));
  };

  const handleStop = (config: LaunchConfig) => {
    addLog(config.id, `⏹️ Stopping ${config.name}...`, 'info');
    sendCommand('stop_launch', { launch_name: config.id });
    setLaunchStates(prev => ({ ...prev, [config.id]: 'stopped' }));
  };

  const handleRestart = async (config: LaunchConfig) => {
    handleStop(config);
    setTimeout(() => handleLaunch(config), 2000);
  };

  const toggleCard = (id: string) => {
    setExpandedCards(prev => ({ ...prev, [id]: !prev[id] }));
  };

  const clearLogs = () => setLogs([]);

  const filteredLogs = selectedLogFilter 
    ? logs.filter(l => l.launch_name === selectedLogFilter)
    : logs;

  const getLogColor = (type: LogEntry['type']) => {
    switch (type) {
      case 'error': return 'text-red-400';
      case 'warn': return 'text-yellow-400';
      case 'success': return 'text-green-400';
      default: return 'text-slate-300';
    }
  };

  return (
    <div className="min-h-screen bg-[#0a0e27] flex">
      {/* Sidebar */}
      <div className="w-56 bg-[#0d1230] border-r border-slate-700/50 p-4">
        <div className="flex items-center gap-3 mb-8">
          <div className="w-10 h-10 bg-cyan-500 rounded-lg flex items-center justify-center text-white font-bold">
            WR
          </div>
          <div>
            <h1 className="text-lg font-bold text-white">WAREHOUSE</h1>
            <p className="text-xs text-slate-400">Control System v1.0</p>
          </div>
        </div>
        
        <nav className="space-y-1">
          {['Dashboard', 'Launch Control', 'Navigation', 'Waypoints', 'Inventory', 'System Monitor'].map((item, i) => (
            <div
              key={item}
              className={`px-4 py-3 rounded-lg cursor-pointer flex items-center gap-3 ${
                i === 1 ? 'bg-green-500 text-white' : 'text-slate-400 hover:bg-slate-800/50'
              }`}
            >
              {['⚡', '🚀', '🧭', '📍', '📦', '📊'][i]} {item}
            </div>
          ))}
        </nav>
      </div>

      {/* Main Content */}
      <div className="flex-1 flex flex-col">
        {/* Header */}
        <div className="h-16 border-b border-slate-700/50 px-6 flex items-center justify-between">
          <h2 className="text-xl font-bold text-white">Launch Control</h2>
          <div className="flex items-center gap-4">
            <div className="flex items-center gap-2">
              <div className={`w-2 h-2 rounded-full ${connected ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`} />
              <span className={connected ? 'text-green-400' : 'text-red-400'}>
                {connected ? 'Connected' : 'Disconnected'}
              </span>
            </div>
            <Button
              variant="outline"
              size="sm"
              onClick={() => setShowLogs(!showLogs)}
              className="border-slate-600 text-slate-300"
            >
              <Terminal className="w-4 h-4 mr-2" />
              {showLogs ? 'Hide Logs' : 'Show Logs'}
            </Button>
          </div>
        </div>

        <div className="flex-1 flex">
          {/* Launch Cards */}
          <div className="flex-1 p-6 overflow-auto">
            <p className="text-slate-400 mb-6">Start and manage ROS2 subsystems</p>
            
            <div className="space-y-4">
              {launchConfigs.map((config) => {
                const state = launchStates[config.id] || 'stopped';
                const isRunning = state === 'running';
                const isExpanded = expandedCards[config.id];
                const params = getParameters(config.id);

                return (
                  <div key={config.id} className="bg-[#1a1f3a] rounded-lg border border-slate-700/50">
                    <div className="p-5">
                      <div className="flex items-center justify-between mb-4">
                        <div className="flex items-center gap-4">
                          <span className="text-3xl">{config.icon}</span>
                          <div>
                            <h3 className="text-lg font-semibold text-green-400">{config.name}</h3>
                            <p className="text-slate-400 text-sm">{config.description}</p>
                          </div>
                        </div>
                        <div className="flex items-center gap-3">
                          <div className={`px-3 py-1 rounded text-sm flex items-center gap-2 ${
                            state === 'running' ? 'text-green-400' : 
                            state === 'error' ? 'text-red-400' : 'text-slate-400'
                          }`}>
                            <span className={`w-2 h-2 rounded-full ${
                              state === 'running' ? 'bg-green-400 animate-pulse' : 
                              state === 'error' ? 'bg-red-400' : 'bg-slate-600'
                            }`} />
                            {state === 'running' ? 'Running' : state === 'error' ? 'Error' : 'Stopped'}
                          </div>
                          {config.parameters && (
                            <button onClick={() => toggleCard(config.id)} className="p-2 hover:bg-slate-700/50 rounded">
                              {isExpanded ? <ChevronUp className="w-5 h-5 text-cyan-400" /> : <ChevronDown className="w-5 h-5 text-cyan-400" />}
                            </button>
                          )}
                        </div>
                      </div>

                      {/* Command Preview */}
                      <div className="mb-4 p-2 bg-[#0a0e27] rounded text-xs font-mono text-slate-400 overflow-x-auto">
                        $ ros2 launch {config.package} {config.launchFile}
                        {config.parameters?.map(p => ` ${p.name}:=${params[p.name] ?? p.default}`).join('')}
                      </div>

                      {/* Parameters */}
                      {config.parameters && isExpanded && (
                        <div className="mb-4 pb-4 border-b border-slate-700/50">
                          <h4 className="text-cyan-400 text-sm font-semibold mb-3">Parameters</h4>
                          <div className="space-y-3">
                            {config.parameters.map((param) => (
                              <div key={param.name} className="flex items-center justify-between">
                                <label className="text-slate-300 text-sm">{param.label}</label>
                                {param.type === 'boolean' ? (
                                  <Switch
                                    checked={params[param.name] ?? param.default}
                                    onCheckedChange={(checked) => updateParameter(config.id, param.name, checked)}
                                    disabled={isRunning}
                                  />
                                ) : (
                                  <Input
                                    value={params[param.name] ?? param.default}
                                    onChange={(e) => updateParameter(config.id, param.name, e.target.value)}
                                    disabled={isRunning}
                                    className="w-48 h-8 bg-[#0a0e27] border-slate-700 text-slate-200 text-sm"
                                  />
                                )}
                              </div>
                            ))}
                          </div>
                        </div>
                      )}

                      {/* Buttons */}
                      <div className="flex gap-2">
                        <Button
                          onClick={() => handleLaunch(config)}
                          disabled={isRunning || !connected}
                          className="flex-1 bg-green-500 hover:bg-green-600 text-white h-10"
                        >
                          <Play className="w-4 h-4 mr-2" /> Launch
                        </Button>
                        <Button
                          onClick={() => handleStop(config)}
                          disabled={!isRunning || !connected}
                          className="flex-1 bg-red-500 hover:bg-red-600 text-white h-10"
                        >
                          <Square className="w-4 h-4 mr-2" /> Stop
                        </Button>
                        <Button
                          onClick={() => handleRestart(config)}
                          disabled={!isRunning || !connected}
                          className="bg-cyan-500 hover:bg-cyan-600 text-white h-10 px-4"
                        >
                          <RotateCw className="w-4 h-4" />
                        </Button>
                      </div>
                    </div>
                  </div>
                );
              })}
            </div>
          </div>

          {/* Logs Panel */}
          {showLogs && (
            <div className="w-[500px] border-l border-slate-700/50 flex flex-col bg-[#0d1230]">
              <div className="h-12 border-b border-slate-700/50 px-4 flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Terminal className="w-4 h-4 text-cyan-400" />
                  <span className="text-white font-medium">Logs</span>
                  <span className="text-xs text-slate-400">({filteredLogs.length})</span>
                </div>
                <div className="flex items-center gap-2">
                  <select
                    value={selectedLogFilter || ''}
                    onChange={(e) => setSelectedLogFilter(e.target.value || null)}
                    className="bg-[#0a0e27] border border-slate-700 rounded px-2 py-1 text-xs text-slate-300"
                  >
                    <option value="">All Logs</option>
                    <option value="system">System</option>
                    {launchConfigs.map(c => (
                      <option key={c.id} value={c.id}>{c.name}</option>
                    ))}
                  </select>
                  <button onClick={clearLogs} className="p-1 hover:bg-slate-700/50 rounded">
                    <Trash2 className="w-4 h-4 text-slate-400" />
                  </button>
                  <button onClick={() => setShowLogs(false)} className="p-1 hover:bg-slate-700/50 rounded">
                    <X className="w-4 h-4 text-slate-400" />
                  </button>
                </div>
              </div>
              
              <ScrollArea className="flex-1 p-4">
                <div className="space-y-1 font-mono text-xs">
                  {filteredLogs.map((log, i) => (
                    <div key={i} className="flex gap-2 py-1">
                      <span className="text-slate-500 shrink-0">
                        {new Date(log.timestamp).toLocaleTimeString()}
                      </span>
                      <span className="text-cyan-400 shrink-0">[{log.launch_name}]</span>
                      <span className={getLogColor(log.type)}>{log.log}</span>
                    </div>
                  ))}
                  <div ref={logsEndRef} />
                </div>
              </ScrollArea>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
