'use client';

import { useROS2 } from '@/hooks/useROS2';
import { launchConfigs, LaunchConfig } from '@/lib/launch-configs';
import { useState } from 'react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs';
import { ScrollArea } from '@/components/ui/scroll-area';
import { Alert, AlertDescription } from '@/components/ui/alert';
import { Progress } from '@/components/ui/progress';

export default function WarehouseRoverDashboard() {
  const {
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
    launch,
    stopLaunch,
  } = useROS2(process.env.NEXT_PUBLIC_ROS2_WS_URL || 'ws://localhost:9090');

  const [activeLaunches, setActiveLaunches] = useState<Set<string>>(new Set());
  const [selectedCategory, setSelectedCategory] = useState<string>('system');

  const handleLaunch = (config: LaunchConfig) => {
    // Build params object
    const params: any = {};
    config.params.forEach(param => {
      params[param.name] = param.default;
    });

    launch(config.id, config.launchFile, params);
    setActiveLaunches(prev => new Set([...prev, config.id]));
  };

  const handleStopLaunch = (configId: string) => {
    stopLaunch(configId);
    setActiveLaunches(prev => {
      const newSet = new Set(prev);
      newSet.delete(configId);
      return newSet;
    });
  };

  const getFilteredLaunches = () => {
    return launchConfigs.filter(config => config.category === selectedCategory);
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="mb-8">
          <div className="flex items-center justify-between mb-4">
            <div>
              <h1 className="text-4xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-cyan-400 to-purple-400">
                🤖 Warehouse Rover Control Center
              </h1>
              <p className="text-slate-400 mt-2">Autonomous Inventory Management System</p>
            </div>
            <div className="flex items-center gap-4">
              <Badge variant={connected ? 'default' : 'destructive'} className="text-lg px-4 py-2">
                {connected ? '🟢 Connected' : '🔴 Disconnected'}
              </Badge>
            </div>
          </div>
        </div>

        {/* Connection Warning */}
        {!connected && (
          <Alert className="mb-6 border-yellow-500 bg-yellow-500/10">
            <AlertDescription>
              Not connected to ROS2 bridge. Start the bridge with: 
              <code className="ml-2 bg-black/30 px-2 py-1 rounded">
                ros2 run mecanum_hardware ros2_web_bridge.py
              </code>
            </AlertDescription>
          </Alert>
        )}

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Left Column: System Status */}
          <div className="lg:col-span-1 space-y-6">
            {/* System Status */}
            <Card className="bg-slate-800/50 border-slate-700">
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  ⚡ System Status
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                {systemStatus && (
                  <>
                    <div className="flex justify-between items-center">
                      <span className="text-sm text-slate-400">Active Nodes</span>
                      <Badge>{systemStatus.active_nodes}</Badge>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-sm text-slate-400">Active Topics</span>
                      <Badge>{systemStatus.active_topics}</Badge>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-sm text-slate-400">Bridge Uptime</span>
                      <Badge>{Math.floor(systemStatus.bridge_uptime)}s</Badge>
                    </div>
                  </>
                )}
              </CardContent>
            </Card>

            {/* Mission Status */}
            <Card className="bg-slate-800/50 border-slate-700">
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  🎯 Mission Status
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <div>
                  <p className="text-xs text-slate-400 mb-1">Mission</p>
                  <p className="text-sm font-mono">{missionStatus || 'Idle'}</p>
                </div>
                <div>
                  <p className="text-xs text-slate-400 mb-1">Waypoint</p>
                  <p className="text-sm font-mono">{waypointStatus || 'No data'}</p>
                </div>
                <div>
                  <p className="text-xs text-slate-400 mb-1">Actuator</p>
                  <p className="text-sm font-mono">{actuatorStatus || 'No data'}</p>
                </div>
              </CardContent>
            </Card>

            {/* Sensor Data */}
            <Card className="bg-slate-800/50 border-slate-700">
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  📡 Sensors
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <div>
                  <p className="text-xs text-slate-400 mb-1">Laser Distance</p>
                  <p className="text-2xl font-bold">{laserDistance.toFixed(2)}m</p>
                </div>
                {lidarData && (
                  <div>
                    <p className="text-xs text-slate-400 mb-1">Closest Obstacle</p>
                    <p className="text-2xl font-bold">{lidarData.closest_obstacle.toFixed(2)}m</p>
                  </div>
                )}
                {robotPose && (
                  <div>
                    <p className="text-xs text-slate-400 mb-1">Robot Position</p>
                    <p className="text-sm font-mono">
                      X: {robotPose.position.x.toFixed(2)}m<br/>
                      Y: {robotPose.position.y.toFixed(2)}m
                    </p>
                  </div>
                )}
              </CardContent>
            </Card>

            {/* Quick Controls */}
            <Card className="bg-slate-800/50 border-slate-700">
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  🎮 Quick Controls
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <div>
                  <p className="text-sm text-slate-400 mb-2">Actuator</p>
                  <div className="flex gap-2">
                    <Button size="sm" onClick={() => controlActuator('up')} className="flex-1">
                      ⬆️ Up
                    </Button>
                    <Button size="sm" onClick={() => controlActuator('down')} className="flex-1">
                      ⬇️ Down
                    </Button>
                    <Button size="sm" onClick={() => controlActuator('stop')} variant="destructive" className="flex-1">
                      🛑 Stop
                    </Button>
                  </div>
                </div>
                <div>
                  <p className="text-sm text-slate-400 mb-2">QR Detection</p>
                  <div className="flex gap-2">
                    <Button size="sm" onClick={() => enableQRDetection(true)} className="flex-1">
                      ▶️ Start
                    </Button>
                    <Button size="sm" onClick={() => enableQRDetection(false)} variant="outline" className="flex-1">
                      ⏸️ Stop
                    </Button>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>

          {/* Middle Column: Launch Controls */}
          <div className="lg:col-span-2">
            <Card className="bg-slate-800/50 border-slate-700 h-full">
              <CardHeader>
                <CardTitle>🚀 Launch Controls</CardTitle>
                <CardDescription>Start and stop ROS2 launch files</CardDescription>
              </CardHeader>
              <CardContent>
                <Tabs value={selectedCategory} onValueChange={setSelectedCategory}>
                  <TabsList className="grid w-full grid-cols-5 mb-4">
                    <TabsTrigger value="system">⚙️ System</TabsTrigger>
                    <TabsTrigger value="mapping">🗺️ Mapping</TabsTrigger>
                    <TabsTrigger value="navigation">🧭 Navigation</TabsTrigger>
                    <TabsTrigger value="inventory">📦 Inventory</TabsTrigger>
                    <TabsTrigger value="testing">🔧 Testing</TabsTrigger>
                  </TabsList>

                  <ScrollArea className="h-[600px]">
                    <div className="space-y-4 pr-4">
                      {getFilteredLaunches().map(config => {
                        const isActive = activeLaunches.has(config.id);
                        return (
                          <Card key={config.id} className={`border ${isActive ? 'border-green-500 bg-green-500/10' : 'border-slate-600'}`}>
                            <CardHeader>
                              <div className="flex items-start justify-between">
                                <div className="flex-1">
                                  <CardTitle className="text-lg flex items-center gap-2">
                                    <span>{config.icon}</span>
                                    <span>{config.name}</span>
                                    {isActive && <Badge variant="default" className="ml-2">Running</Badge>}
                                  </CardTitle>
                                  <CardDescription className="mt-1">{config.description}</CardDescription>
                                </div>
                                <div className="flex gap-2">
                                  {!isActive ? (
                                    <Button
                                      size="sm"
                                      onClick={() => handleLaunch(config)}
                                      disabled={!connected}
                                    >
                                      ▶️ Launch
                                    </Button>
                                  ) : (
                                    <Button
                                      size="sm"
                                      variant="destructive"
                                      onClick={() => handleStopLaunch(config.id)}
                                    >
                                      ⏹️ Stop
                                    </Button>
                                  )}
                                </div>
                              </div>
                            </CardHeader>
                            {config.params.length > 0 && (
                              <CardContent>
                                <div className="text-xs text-slate-400 space-y-1">
                                  <p className="font-semibold mb-2">Parameters:</p>
                                  {config.params.map(param => (
                                    <div key={param.name} className="flex justify-between">
                                      <span>{param.label}:</span>
                                      <span className="font-mono text-slate-300">{String(param.default)}</span>
                                    </div>
                                  ))}
                                </div>
                              </CardContent>
                            )}
                          </Card>
                        );
                      })}
                    </div>
                  </ScrollArea>
                </Tabs>
              </CardContent>
            </Card>
          </div>
        </div>

        {/* QR Detections (Bottom) */}
        {qrDetections.length > 0 && (
          <Card className="mt-6 bg-slate-800/50 border-slate-700">
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                📷 Recent QR Detections ({qrDetections.length})
              </CardTitle>
            </CardHeader>
            <CardContent>
              <ScrollArea className="h-[200px]">
                <div className="space-y-2">
                  {qrDetections.map((det, idx) => (
                    <div key={idx} className="flex items-center justify-between p-3 bg-slate-700/50 rounded border border-slate-600">
                      <div className="flex gap-4">
                        <Badge variant={det.is_valid ? 'default' : 'destructive'}>
                          {det.is_valid ? '✓' : '✗'}
                        </Badge>
                        <span className="font-mono text-sm">{det.qr_data}</span>
                        <span className="text-slate-400 text-sm">
                          Rack: {det.rack_id} | Shelf: {det.shelf_id} | Item: {det.item_code}
                        </span>
                      </div>
                      <Badge>{(det.confidence * 100).toFixed(0)}%</Badge>
                    </div>
                  ))}
                </div>
              </ScrollArea>
            </CardContent>
          </Card>
        )}
      </div>
    </div>
  );
}
