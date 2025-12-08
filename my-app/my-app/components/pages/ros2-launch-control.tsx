"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Switch } from "@/components/ui/switch"
import { useRealTime } from "@/components/real-time-provider"
import { Play, Square, RotateCw, ChevronDown, ChevronUp, Settings, Map, Navigation, Camera, MapPin } from "lucide-react"

interface LaunchConfig {
  id: string
  name: string
  description: string
  icon: React.ReactNode
  package: string
  launchFile: string
  command: string
  parameters?: {
    name: string
    label: string
    type: 'string' | 'boolean'
    default: any
  }[]
}

const launchConfigs: LaunchConfig[] = [
  {
    id: 'hardware',
    name: 'Hardware Layer',
    description: 'Motors, controllers, sensors',
    icon: <Settings className="w-8 h-8 text-cyan-400" />,
    package: 'mecanum_hardware',
    launchFile: 'hardware.launch.py',
    command: 'ros2 launch',
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
    icon: <Map className="w-8 h-8 text-green-400" />,
    package: 'mecanum_hardware',
    launchFile: 'hardware_mapping.launch.py',
    command: 'ros2 launch',
  },
  {
    id: 'navigation',
    name: 'Navigation Mode',
    description: 'Nav2 path planning',
    icon: <Navigation className="w-8 h-8 text-blue-400" />,
    package: 'mecanum_hardware',
    launchFile: 'hardware_navigation.launch.py',
    command: 'ros2 launch',
    parameters: [
      { name: 'map', label: 'map_file', type: 'string', default: '/root/ros2_ws/maps/warehouse.yaml' },
    ],
  },
  {
    id: 'qr_detection',
    name: 'QR Detection',
    description: 'Camera and QR scanning',
    icon: <Camera className="w-8 h-8 text-purple-400" />,
    package: 'warehouse_rover_qr_detection',
    launchFile: 'qr_detection.launch.py',
    command: 'ros2 launch',
  },
  {
    id: 'waypoint',
    name: 'Waypoint Navigation',
    description: 'Autonomous waypoint following',
    icon: <MapPin className="w-8 h-8 text-yellow-400" />,
    package: 'mecanum_hardware',
    launchFile: 'waypoint_follower.launch.py',
    command: 'ros2 launch',
  },
]

export default function ROS2LaunchControl() {
  const { isConnected, send, subscribe } = useRealTime()
  const [launchStates, setLaunchStates] = useState<Record<string, 'stopped' | 'running'>>({})
  const [expandedCards, setExpandedCards] = useState<Record<string, boolean>>({ hardware: true })
  const [parameters, setParameters] = useState<Record<string, Record<string, any>>>({})
  const [topicData, setTopicData] = useState<Record<string, any>>({})

  // Subscribe to ROS2 topics
  useEffect(() => {
    const unsubscribes: (() => void)[] = []

    // Subscribe to topic updates from backend
    unsubscribes.push(subscribe('topic', (data: any) => {
      setTopicData(prev => ({
        ...prev,
        [data.topic]: data.data
      }))
    }))

    // Subscribe to system status
    unsubscribes.push(subscribe('system_status', (data: any) => {
      setTopicData(prev => ({
        ...prev,
        system_status: data.data
      }))
    }))

    // Subscribe to launch status updates
    unsubscribes.push(subscribe('launch_status', (data: any) => {
      if (data.launch_name && data.status) {
        setLaunchStates(prev => ({
          ...prev,
          [data.launch_name]: data.status
        }))
      }
    }))

    return () => {
      unsubscribes.forEach(unsub => unsub())
    }
  }, [subscribe])

  const getParameters = (configId: string) => {
    if (!parameters[configId]) {
      const config = launchConfigs.find(c => c.id === configId)
      if (config?.parameters) {
        const defaultParams: Record<string, any> = {}
        config.parameters.forEach(p => {
          defaultParams[p.name] = p.default
        })
        return defaultParams
      }
    }
    return parameters[configId] || {}
  }

  const updateParameter = (configId: string, paramName: string, value: any) => {
    setParameters(prev => ({
      ...prev,
      [configId]: {
        ...prev[configId],
        [paramName]: value,
      },
    }))
  }

  const handleLaunch = (config: LaunchConfig) => {
    const params = getParameters(config.id)
    const launchParams: Record<string, any> = {}
    
    if (config.parameters) {
      config.parameters.forEach(p => {
        const value = params[p.name] ?? p.default
        launchParams[p.name] = value
      })
    }

    // Send launch command to backend
    send('launch', {
      launch_name: config.id,
      launch_file: `${config.package} ${config.launchFile}`,
      params: launchParams
    })

    setLaunchStates(prev => ({ ...prev, [config.id]: 'running' }))
  }

  const handleStop = (config: LaunchConfig) => {
    send('stop_launch', { launch_name: config.id })
    setLaunchStates(prev => ({ ...prev, [config.id]: 'stopped' }))
  }

  const handleRestart = (config: LaunchConfig) => {
    handleStop(config)
    setTimeout(() => handleLaunch(config), 1000)
  }

  const toggleCard = (id: string) => {
    setExpandedCards(prev => ({ ...prev, [id]: !prev[id] }))
  }

  return (
    <div className="p-6 space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-white">Launch Control</h1>
          <p className="text-muted-foreground">Start and manage ROS2 subsystems</p>
        </div>
        <div className="flex items-center gap-3">
          <div className={`px-4 py-2 rounded-lg ${isConnected ? 'bg-green-500/20 text-green-400' : 'bg-red-500/20 text-red-400'}`}>
            <span className={`inline-block w-2 h-2 rounded-full mr-2 ${isConnected ? 'bg-green-400' : 'bg-red-400'}`} />
            {isConnected ? 'Connected' : 'Disconnected'}
          </div>
        </div>
      </div>

      {/* System Status */}
      {topicData.system_status && (
        <Card className="bg-card/50 border-border">
          <CardHeader className="pb-3">
            <CardTitle className="text-lg">System Status</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-3 gap-4 text-sm">
              <div>
                <span className="text-muted-foreground">Active Nodes:</span>
                <span className="ml-2 text-white font-mono">{topicData.system_status.active_nodes || 0}</span>
              </div>
              <div>
                <span className="text-muted-foreground">Active Topics:</span>
                <span className="ml-2 text-white font-mono">{topicData.system_status.active_topics || 0}</span>
              </div>
              <div>
                <span className="text-muted-foreground">Bridge Uptime:</span>
                <span className="ml-2 text-white font-mono">{Math.floor(topicData.system_status.bridge_uptime || 0)}s</span>
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Launch Controls */}
      <div className="space-y-4">
        {launchConfigs.map((config) => {
          const isRunning = launchStates[config.id] === 'running'
          const isExpanded = expandedCards[config.id]
          const params = getParameters(config.id)

          return (
            <Card key={config.id} className="bg-card/50 border-border hover:border-primary/50 transition-colors">
              <CardHeader className="pb-3">
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-4">
                    {config.icon}
                    <div>
                      <CardTitle className="text-xl text-primary">{config.name}</CardTitle>
                      <p className="text-sm text-muted-foreground">{config.description}</p>
                    </div>
                  </div>
                  <div className="flex items-center gap-3">
                    <div className={`px-4 py-2 rounded-lg text-sm ${
                      isRunning ? 'bg-green-500/20 text-green-400' : 'bg-muted text-muted-foreground'
                    }`}>
                      <span className={`inline-block w-2 h-2 rounded-full mr-2 ${
                        isRunning ? 'bg-green-400' : 'bg-muted-foreground'
                      }`} />
                      {isRunning ? 'Running' : 'Not Running'}
                    </div>
                    {config.parameters && (
                      <Button
                        variant="ghost"
                        size="icon"
                        onClick={() => toggleCard(config.id)}
                      >
                        {isExpanded ? (
                          <ChevronUp className="w-5 h-5" />
                        ) : (
                          <ChevronDown className="w-5 h-5" />
                        )}
                      </Button>
                    )}
                  </div>
                </div>
              </CardHeader>

              <CardContent className="space-y-4">
                {/* Command Preview */}
                <div className="p-3 bg-background rounded-lg border border-border">
                  <p className="text-xs text-muted-foreground mb-1">Command:</p>
                  <code className="text-sm text-cyan-400 font-mono">
                    {config.command} {config.package} {config.launchFile}
                  </code>
                </div>

                {/* Parameters Section */}
                {config.parameters && isExpanded && (
                  <div className="space-y-4 pt-4 border-t border-border">
                    <h4 className="text-sm font-semibold text-primary">Parameters</h4>
                    <div className="space-y-3">
                      {config.parameters.map((param) => (
                        <div key={param.name} className="flex items-center justify-between">
                          <label className="text-sm text-muted-foreground">{param.label}</label>
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
                              className="w-64"
                            />
                          )}
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {/* Action Buttons */}
                <div className="flex gap-3 pt-2">
                  <Button
                    onClick={() => handleLaunch(config)}
                    disabled={isRunning || !isConnected}
                    className="flex-1 bg-green-600 hover:bg-green-700"
                  >
                    <Play className="w-4 h-4 mr-2" />
                    Launch
                  </Button>
                  <Button
                    onClick={() => handleStop(config)}
                    disabled={!isRunning || !isConnected}
                    variant="destructive"
                    className="flex-1"
                  >
                    <Square className="w-4 h-4 mr-2" />
                    Stop
                  </Button>
                  <Button
                    onClick={() => handleRestart(config)}
                    disabled={!isRunning || !isConnected}
                    variant="outline"
                  >
                    <RotateCw className="w-4 h-4" />
                  </Button>
                </div>
              </CardContent>
            </Card>
          )
        })}
      </div>

      {/* Topic Data Display */}
      {Object.keys(topicData).length > 0 && (
        <Card className="bg-card/50 border-border">
          <CardHeader>
            <CardTitle className="text-lg">Live Topic Data</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4 text-sm">
              {topicData.odometry && (
                <div className="p-3 bg-background rounded-lg border border-border">
                  <h4 className="font-semibold text-cyan-400 mb-2">Odometry</h4>
                  <div className="font-mono text-xs space-y-1">
                    <p>X: {topicData.odometry.position?.x?.toFixed(3) || '0.000'}</p>
                    <p>Y: {topicData.odometry.position?.y?.toFixed(3) || '0.000'}</p>
                  </div>
                </div>
              )}
              {topicData.lidar && (
                <div className="p-3 bg-background rounded-lg border border-border">
                  <h4 className="font-semibold text-green-400 mb-2">LiDAR</h4>
                  <div className="font-mono text-xs">
                    <p>Closest: {topicData.lidar.closest_obstacle?.toFixed(2) || '0.00'}m</p>
                    <p>Readings: {topicData.lidar.num_readings || 0}</p>
                  </div>
                </div>
              )}
              {topicData.qr_detections && (
                <div className="p-3 bg-background rounded-lg border border-border">
                  <h4 className="font-semibold text-purple-400 mb-2">QR Detections</h4>
                  <div className="font-mono text-xs">
                    <p>Count: {topicData.qr_detections.detection_count || 0}</p>
                    {topicData.qr_detections.detections?.[0] && (
                      <p>Latest: {topicData.qr_detections.detections[0].qr_data}</p>
                    )}
                  </div>
                </div>
              )}
              {topicData.actuator_status && (
                <div className="p-3 bg-background rounded-lg border border-border">
                  <h4 className="font-semibold text-yellow-400 mb-2">Actuator</h4>
                  <div className="font-mono text-xs">
                    <p>Status: {topicData.actuator_status.data || 'unknown'}</p>
                  </div>
                </div>
              )}
            </div>
          </CardContent>
        </Card>
      )}
    </div>
  )
}
