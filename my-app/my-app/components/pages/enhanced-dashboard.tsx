"use client"

import { useState, useEffect, useCallback, useRef } from "react"
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Switch } from "@/components/ui/switch"
import { Slider } from "@/components/ui/slider"
import { Badge } from "@/components/ui/badge"
import { ScrollArea } from "@/components/ui/scroll-area"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { useRealTime } from "@/components/real-time-provider"
import { 
  Play, Square, Settings, Map, Navigation, Camera, MapPin, 
  ArrowUp, ArrowDown, ArrowLeft, ArrowRight, RotateCcw, RotateCw,
  Gauge, Activity, Wifi, WifiOff, ChevronDown, ChevronUp,
  Gamepad2, Sliders, Rocket, Box, Database, Eye
} from "lucide-react"

// =================== VIRTUAL JOYSTICK COMPONENT ===================
interface JoystickProps {
  onMove: (x: number, y: number) => void
  onRotate?: (z: number) => void
  size?: number
  label?: string
}

function VirtualJoystick({ onMove, size = 150, label = "Movement" }: JoystickProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [isDragging, setIsDragging] = useState(false)
  const [position, setPosition] = useState({ x: 0, y: 0 })
  const centerRef = useRef({ x: size / 2, y: size / 2 })
  const maxRadius = size / 2 - 20

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Clear canvas
    ctx.clearRect(0, 0, size, size)

    // Draw outer circle (base)
    ctx.beginPath()
    ctx.arc(size / 2, size / 2, maxRadius + 10, 0, Math.PI * 2)
    ctx.strokeStyle = '#22d3ee'
    ctx.lineWidth = 2
    ctx.stroke()
    ctx.fillStyle = 'rgba(34, 211, 238, 0.1)'
    ctx.fill()

    // Draw crosshairs
    ctx.beginPath()
    ctx.strokeStyle = 'rgba(34, 211, 238, 0.3)'
    ctx.lineWidth = 1
    ctx.moveTo(size / 2, 10)
    ctx.lineTo(size / 2, size - 10)
    ctx.moveTo(10, size / 2)
    ctx.lineTo(size - 10, size / 2)
    ctx.stroke()

    // Draw inner circle (knob)
    const knobX = size / 2 + position.x * maxRadius
    const knobY = size / 2 - position.y * maxRadius
    
    ctx.beginPath()
    ctx.arc(knobX, knobY, 20, 0, Math.PI * 2)
    ctx.fillStyle = isDragging ? '#22d3ee' : 'rgba(34, 211, 238, 0.7)'
    ctx.fill()
    ctx.strokeStyle = '#06b6d4'
    ctx.lineWidth = 3
    ctx.stroke()

  }, [position, isDragging, size, maxRadius])

  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true)
    updatePosition(e)
  }

  const handleMouseMove = (e: React.MouseEvent) => {
    if (isDragging) {
      updatePosition(e)
    }
  }

  const handleMouseUp = () => {
    setIsDragging(false)
    setPosition({ x: 0, y: 0 })
    onMove(0, 0)
  }

  const handleTouchStart = (e: React.TouchEvent) => {
    e.preventDefault()
    setIsDragging(true)
    updateTouchPosition(e)
  }

  const handleTouchMove = (e: React.TouchEvent) => {
    e.preventDefault()
    if (isDragging) {
      updateTouchPosition(e)
    }
  }

  const handleTouchEnd = () => {
    setIsDragging(false)
    setPosition({ x: 0, y: 0 })
    onMove(0, 0)
  }

  const updatePosition = (e: React.MouseEvent) => {
    const canvas = canvasRef.current
    if (!canvas) return

    const rect = canvas.getBoundingClientRect()
    const x = e.clientX - rect.left - size / 2
    const y = -(e.clientY - rect.top - size / 2)

    const distance = Math.sqrt(x * x + y * y)
    const angle = Math.atan2(y, x)

    const clampedDistance = Math.min(distance, maxRadius)
    const normalizedX = (clampedDistance * Math.cos(angle)) / maxRadius
    const normalizedY = (clampedDistance * Math.sin(angle)) / maxRadius

    setPosition({ x: normalizedX, y: normalizedY })
    onMove(normalizedX, normalizedY)
  }

  const updateTouchPosition = (e: React.TouchEvent) => {
    const canvas = canvasRef.current
    if (!canvas) return

    const rect = canvas.getBoundingClientRect()
    const touch = e.touches[0]
    const x = touch.clientX - rect.left - size / 2
    const y = -(touch.clientY - rect.top - size / 2)

    const distance = Math.sqrt(x * x + y * y)
    const angle = Math.atan2(y, x)

    const clampedDistance = Math.min(distance, maxRadius)
    const normalizedX = (clampedDistance * Math.cos(angle)) / maxRadius
    const normalizedY = (clampedDistance * Math.sin(angle)) / maxRadius

    setPosition({ x: normalizedX, y: normalizedY })
    onMove(normalizedX, normalizedY)
  }

  return (
    <div className="flex flex-col items-center gap-2">
      <span className="text-sm text-cyan-400 font-semibold">{label}</span>
      <canvas
        ref={canvasRef}
        width={size}
        height={size}
        className="cursor-pointer touch-none"
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onTouchStart={handleTouchStart}
        onTouchMove={handleTouchMove}
        onTouchEnd={handleTouchEnd}
      />
      <div className="text-xs text-slate-400">
        X: {position.x.toFixed(2)} | Y: {position.y.toFixed(2)}
      </div>
    </div>
  )
}

// =================== ROTATION SLIDER ===================
interface RotationSliderProps {
  value: number
  onChange: (value: number) => void
}

function RotationSlider({ value, onChange }: RotationSliderProps) {
  const [isDragging, setIsDragging] = useState(false)

  return (
    <div className="flex flex-col items-center gap-2 w-full">
      <span className="text-sm text-cyan-400 font-semibold">Rotation</span>
      <div className="flex items-center gap-4 w-full">
        <RotateCcw className="w-5 h-5 text-cyan-400" />
        <Slider
          value={[value]}
          onValueChange={(v) => {
            setIsDragging(true)
            onChange(v[0])
          }}
          onValueCommit={() => {
            setIsDragging(false)
            onChange(0)
          }}
          min={-1}
          max={1}
          step={0.05}
          className="flex-1"
        />
        <RotateCw className="w-5 h-5 text-cyan-400" />
      </div>
      <span className="text-xs text-slate-400">
        Angular: {value.toFixed(2)} rad/s
      </span>
    </div>
  )
}

// =================== LIFT HEIGHT VISUALIZER ===================
interface LiftVisualizerProps {
  currentHeight: number
  maxHeight: number
  minHeight: number
  status: string
}

function LiftVisualizer({ currentHeight, maxHeight, minHeight, status }: LiftVisualizerProps) {
  const heightPercent = ((currentHeight - minHeight) / (maxHeight - minHeight)) * 100
  const clampedPercent = Math.max(0, Math.min(100, heightPercent))

  return (
    <div className="flex items-end gap-4 h-40">
      {/* Lift visualization */}
      <div className="relative w-20 h-full bg-slate-800 rounded-lg border border-cyan-500/30 overflow-hidden">
        {/* Height markers */}
        <div className="absolute inset-0 flex flex-col justify-between p-1 text-[8px] text-slate-500">
          <span>{maxHeight.toFixed(1)}m</span>
          <span>{((maxHeight + minHeight) / 2).toFixed(1)}m</span>
          <span>{minHeight.toFixed(1)}m</span>
        </div>
        
        {/* Current height bar */}
        <div 
          className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-cyan-500 to-cyan-400 transition-all duration-300"
          style={{ height: `${clampedPercent}%` }}
        />
        
        {/* Lift platform */}
        <div 
          className="absolute left-1 right-1 h-3 bg-yellow-400 rounded-sm transition-all duration-300 flex items-center justify-center"
          style={{ bottom: `${clampedPercent}%` }}
        >
          <Box className="w-2 h-2 text-yellow-800" />
        </div>
      </div>

      {/* Stats */}
      <div className="flex flex-col gap-2">
        <div className="text-2xl font-bold text-cyan-400">
          {currentHeight.toFixed(2)}m
        </div>
        <Badge 
          variant={status === 'moving_up' ? 'default' : status === 'moving_down' ? 'secondary' : 'outline'}
          className={status === 'moving_up' ? 'bg-green-500' : status === 'moving_down' ? 'bg-orange-500' : ''}
        >
          {status || 'idle'}
        </Badge>
      </div>
    </div>
  )
}

// =================== PARAMETER EDITOR ===================
interface Parameter {
  name: string
  label: string
  type: 'string' | 'number' | 'boolean'
  value: any
  min?: number
  max?: number
  step?: number
}

interface ParameterEditorProps {
  parameters: Parameter[]
  onChange: (name: string, value: any) => void
}

function ParameterEditor({ parameters, onChange }: ParameterEditorProps) {
  return (
    <div className="space-y-4">
      {parameters.map((param) => (
        <div key={param.name} className="flex items-center justify-between gap-4">
          <label className="text-sm text-slate-300 min-w-[120px]">{param.label}</label>
          
          {param.type === 'boolean' ? (
            <Switch
              checked={param.value}
              onCheckedChange={(v) => onChange(param.name, v)}
            />
          ) : param.type === 'number' ? (
            <div className="flex items-center gap-2">
              <Slider
                value={[param.value]}
                onValueChange={(v) => onChange(param.name, v[0])}
                min={param.min || 0}
                max={param.max || 100}
                step={param.step || 0.1}
                className="w-24"
              />
              <Input
                type="number"
                value={param.value}
                onChange={(e) => onChange(param.name, parseFloat(e.target.value))}
                className="w-20 bg-slate-800 border-slate-600 text-cyan-400"
              />
            </div>
          ) : (
            <Input
              value={param.value}
              onChange={(e) => onChange(param.name, e.target.value)}
              className="w-48 bg-slate-800 border-slate-600"
            />
          )}
        </div>
      ))}
    </div>
  )
}

// =================== LAUNCH CARD ===================
interface LaunchConfig {
  id: string
  name: string
  description: string
  icon: React.ReactNode
  package: string
  launchFile: string
  parameters?: Parameter[]
}

interface LaunchCardProps {
  config: LaunchConfig
  isRunning: boolean
  onLaunch: (params: Record<string, any>) => void
  onStop: () => void
}

function LaunchCard({ config, isRunning, onLaunch, onStop }: LaunchCardProps) {
  const [expanded, setExpanded] = useState(false)
  const [params, setParams] = useState<Record<string, any>>(() => {
    const initial: Record<string, any> = {}
    config.parameters?.forEach(p => {
      initial[p.name] = p.value
    })
    return initial
  })

  return (
    <Card className={`bg-slate-900/80 border-slate-700 transition-all ${isRunning ? 'border-green-500/50 shadow-green-500/20 shadow-lg' : ''}`}>
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            {config.icon}
            <div>
              <CardTitle className="text-lg text-white">{config.name}</CardTitle>
              <CardDescription className="text-slate-400">{config.description}</CardDescription>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <Badge variant={isRunning ? "default" : "secondary"} className={isRunning ? "bg-green-500" : ""}>
              {isRunning ? "Running" : "Stopped"}
            </Badge>
            {config.parameters && config.parameters.length > 0 && (
              <Button
                variant="ghost"
                size="sm"
                onClick={() => setExpanded(!expanded)}
              >
                {expanded ? <ChevronUp className="w-4 h-4" /> : <ChevronDown className="w-4 h-4" />}
              </Button>
            )}
          </div>
        </div>
      </CardHeader>
      
      {expanded && config.parameters && (
        <CardContent className="pt-2 pb-4 border-t border-slate-700">
          <ParameterEditor
            parameters={config.parameters.map(p => ({ ...p, value: params[p.name] ?? p.value }))}
            onChange={(name, value) => setParams(prev => ({ ...prev, [name]: value }))}
          />
        </CardContent>
      )}
      
      <CardContent className="pt-2">
        <div className="flex gap-2">
          {!isRunning ? (
            <Button 
              className="flex-1 bg-green-600 hover:bg-green-700"
              onClick={() => onLaunch(params)}
            >
              <Play className="w-4 h-4 mr-2" /> Launch
            </Button>
          ) : (
            <Button 
              className="flex-1 bg-red-600 hover:bg-red-700"
              onClick={onStop}
            >
              <Square className="w-4 h-4 mr-2" /> Stop
            </Button>
          )}
        </div>
      </CardContent>
    </Card>
  )
}

// =================== MAIN DASHBOARD ===================
export default function EnhancedDashboard() {
  const { isConnected, send, subscribe } = useRealTime()
  
  // State
  const [activeTab, setActiveTab] = useState("teleop")
  const [launchStates, setLaunchStates] = useState<Record<string, boolean>>({})
  const [robotData, setRobotData] = useState({
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 },
    linearVel: { x: 0, y: 0, z: 0 },
    angularVel: { x: 0, y: 0, z: 0 }
  })
  const [sensorData, setSensorData] = useState({
    laserHeight: 0.0,
    closestObstacle: 10.0,
    imuOrientation: { roll: 0, pitch: 0, yaw: 0 }
  })
  const [actuatorStatus, setActuatorStatus] = useState("idle")
  const [qrDetections, setQrDetections] = useState<any[]>([])
  const [logs, setLogs] = useState<string[]>([])
  
  const [liftCommand, setLiftCommand] = useState<'up' | 'down' | 'stop'>('stop')
  
  // Velocity state for teleop
  const [velocity, setVelocity] = useState({ linear_x: 0, linear_y: 0, angular_z: 0 })
  const [maxSpeed, setMaxSpeed] = useState(0.5)
  const [maxAngular, setMaxAngular] = useState(1.0)
  
  // Launch configurations
  const launchConfigs: LaunchConfig[] = [
    {
      id: 'hardware',
      name: 'Hardware Layer',
      description: 'Motors, sensors, controllers',
      icon: <Settings className="w-8 h-8 text-cyan-400" />,
      package: 'mecanum_hardware',
      launchFile: 'hardware.launch.py',
      parameters: [
        { name: 'serial_port', label: 'Serial Port', type: 'string', value: '/dev/ttyUSB0' },
        { name: 'lidar_port', label: 'LiDAR Port', type: 'string', value: '/dev/ttyUSB2' },
      ]
    },
    {
      id: 'mapping',
      name: 'Mapping Mode',
      description: 'SLAM with Cartographer',
      icon: <Map className="w-8 h-8 text-green-400" />,
      package: 'mecanum_hardware',
      launchFile: 'hardware_mapping.launch.py',
    },
    {
      id: 'navigation',
      name: 'Navigation',
      description: 'Nav2 autonomous navigation',
      icon: <Navigation className="w-8 h-8 text-blue-400" />,
      package: 'mecanum_hardware',
      launchFile: 'hardware_navigation.launch.py',
      parameters: [
        { name: 'map_file', label: 'Map File', type: 'string', value: '/root/ros2_ws/maps/warehouse.yaml' },
      ]
    },
    {
      id: 'inventory',
      name: 'Inventory Mission',
      description: 'Waypoint + QR scanning',
      icon: <Box className="w-8 h-8 text-yellow-400" />,
      package: 'mecanum_hardware',
      launchFile: 'waypoint_inventory_mission.launch.py',
      parameters: [
        { name: 'waypoint_delay', label: 'Waypoint Delay (s)', type: 'number', value: 60, min: 10, max: 120 },
        { name: 'max_lift_height', label: 'Max Lift Height (m)', type: 'number', value: 1.5, min: 0.5, max: 2.0, step: 0.1 },
        { name: 'use_manual_waypoints', label: 'Manual Waypoints', type: 'boolean', value: false },
      ]
    },
    {
      id: 'qr_detection',
      name: 'QR Detection',
      description: 'Camera + QR scanning',
      icon: <Camera className="w-8 h-8 text-purple-400" />,
      package: 'warehouse_rover_qr_detection',
      launchFile: 'qr_detector.launch.py',
    },
    {
      id: 'actuator',
      name: 'Actuator System',
      description: 'Lift control with laser feedback',
      icon: <Gauge className="w-8 h-8 text-orange-400" />,
      package: 'mecanum_hardware',
      launchFile: 'actuator_complete.launch.py',
      parameters: [
        { name: 'min_height', label: 'Min Height (m)', type: 'number', value: 0.05, min: 0, max: 0.5, step: 0.01 },
        { name: 'max_height', label: 'Max Height (m)', type: 'number', value: 1.5, min: 1.0, max: 2.5, step: 0.1 },
        { name: 'pid_kp', label: 'PID Kp', type: 'number', value: 50.0, min: 0, max: 100, step: 1 },
        { name: 'pid_ki', label: 'PID Ki', type: 'number', value: 0.1, min: 0, max: 10, step: 0.1 },
        { name: 'pid_kd', label: 'PID Kd', type: 'number', value: 5.0, min: 0, max: 50, step: 0.5 },
      ]
    },
  ]

  // Subscribe to ROS2 data
  useEffect(() => {
    const unsubscribes: (() => void)[] = []

    unsubscribes.push(subscribe('topic', (data: any) => {
      if (data.topic === 'odometry') {
        setRobotData({
          position: data.data.position,
          orientation: data.data.orientation,
          linearVel: data.data.linear_velocity,
          angularVel: data.data.angular_velocity
        })
      } else if (data.topic === 'laser_distance') {
        setSensorData(prev => ({ ...prev, laserHeight: data.data.data }))
      } else if (data.topic === 'lidar') {
        setSensorData(prev => ({ ...prev, closestObstacle: data.data.closest_obstacle }))
      } else if (data.topic === 'actuator_status') {
        setActuatorStatus(data.data.data)
      } else if (data.topic === 'qr_detections') {
        setQrDetections(data.data.detections || [])
      }
    }))

    unsubscribes.push(subscribe('launch_status', (data: any) => {
      if (data.launch_name) {
        setLaunchStates(prev => ({ ...prev, [data.launch_name]: data.status === 'running' }))
      }
    }))

    unsubscribes.push(subscribe('launch_log', (data: any) => {
      if (data.line) {
        setLogs(prev => [...prev.slice(-100), `[${data.launch_name}] ${data.line}`])
      }
    }))

    return () => unsubscribes.forEach(u => u())
  }, [subscribe])

  // Send velocity commands
  useEffect(() => {
    const interval = setInterval(() => {
      const linear_x = Number(velocity.linear_x) * Number(maxSpeed) || 0.0
      const linear_y = Number(velocity.linear_y) * Number(maxSpeed) || 0.0
      const angular_z = Number(velocity.angular_z) * Number(maxAngular) || 0.0
      
      send('publish_cmd_vel', {
        linear_x: linear_x,
        linear_y: linear_y,
        angular_z: angular_z
      })
    }, 100)

    return () => clearInterval(interval)
  }, [velocity, maxSpeed, maxAngular, send])

  // Lift height simulation effect (internal, no UI indicator)
  useEffect(() => {
    const interval = setInterval(() => {
      setSensorData(prev => {
        const MIN_HEIGHT = 0.05
        const MAX_HEIGHT = 1.5
        const SPEED = 0.01 // meters per update (100ms interval = 0.1m/s)
        
        let newHeight = prev.laserHeight

        if (liftCommand === 'up') {
          newHeight = Math.min(prev.laserHeight + SPEED, MAX_HEIGHT)
        } else if (liftCommand === 'down') {
          newHeight = Math.max(prev.laserHeight - SPEED, MIN_HEIGHT)
        }
        // If 'stop', keep current height

        return { ...prev, laserHeight: newHeight }
      })
    }, 100) // Update every 100ms

    return () => clearInterval(interval)
  }, [liftCommand])

  const handleJoystickMove = useCallback((x: number, y: number) => {
    setVelocity(prev => ({ ...prev, linear_x: Number(y) || 0, linear_y: Number(-x) || 0 }))
  }, [])

  const handleRotationChange = useCallback((z: number) => {
    setVelocity(prev => ({ ...prev, angular_z: Number(z) || 0 }))
  }, [])

  const handleLaunch = (configId: string, params: Record<string, any>) => {
    const config = launchConfigs.find(c => c.id === configId)
    if (config) {
      send('launch', {
        launch_name: configId,
        launch_file: `${config.package} ${config.launchFile}`,
        params
      })
    }
  }

  const handleStopLaunch = (configId: string) => {
    send('stop_launch', { launch_name: configId })
  }

  const handleActuatorControl = (command: 'up' | 'down' | 'stop') => {
    console.log('[Actuator] Sending command:', command)
    send('actuator_command', { command })
    
    // Update simulation state
    setLiftCommand(command)
    
    // Also add visual feedback
    setActuatorStatus(command === 'up' ? 'moving_up' : command === 'down' ? 'moving_down' : 'stopped')
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-950 via-slate-900 to-slate-950 dark:from-slate-950 dark:via-slate-900 dark:to-slate-950 bg-white dark:bg-slate-950 p-4 md:p-6 lg:p-8">
      {/* Header */}
      <div className="flex flex-col md:flex-row items-start md:items-center justify-between mb-6 gap-4">
        <div className="flex flex-col sm:flex-row items-start sm:items-center gap-3 md:gap-4">
          <h1 className="text-2xl md:text-3xl lg:text-4xl font-bold bg-gradient-to-r from-cyan-400 to-blue-500 bg-clip-text text-transparent">
            🤖 Warehouse Rover Control
          </h1>
          <Badge variant={isConnected ? "default" : "destructive"} className={isConnected ? "bg-green-500" : ""}>
            {isConnected ? <><Wifi className="w-3 h-3 mr-1" /> Connected</> : <><WifiOff className="w-3 h-3 mr-1" /> Disconnected</>}
          </Badge>
        </div>
        
        {/* Quick Stats */}
        <div className="flex gap-3 md:gap-4 flex-wrap">
          <div className="text-center">
            <div className="text-xs text-slate-400 dark:text-slate-500">Position</div>
            <div className="text-sm text-cyan-600 dark:text-cyan-400">
              ({robotData.position.x.toFixed(2)}, {robotData.position.y.toFixed(2)})
            </div>
          </div>
          <div className="text-center">
            <div className="text-xs text-slate-400 dark:text-slate-500">Speed</div>
            <div className="text-sm text-green-600 dark:text-green-400">
              {Math.sqrt(robotData.linearVel.x ** 2 + robotData.linearVel.y ** 2).toFixed(2)} m/s
            </div>
          </div>
          <div className="text-center">
            <div className="text-xs text-slate-400 dark:text-slate-500">Obstacle</div>
            <div className={`text-sm ${sensorData.closestObstacle < 0.5 ? 'text-red-600 dark:text-red-400' : 'text-green-600 dark:text-green-400'}`}>
              {sensorData.closestObstacle.toFixed(2)}m
            </div>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <Tabs value={activeTab} onValueChange={setActiveTab} className="space-y-4">
        <TabsList className="bg-slate-800/50 border border-slate-700">
          <TabsTrigger value="teleop" className="data-[state=active]:bg-cyan-600">
            <Gamepad2 className="w-4 h-4 mr-2" /> Teleop
          </TabsTrigger>
          <TabsTrigger value="launch" className="data-[state=active]:bg-cyan-600">
            <Rocket className="w-4 h-4 mr-2" /> Launch Control
          </TabsTrigger>
          <TabsTrigger value="actuator" className="data-[state=active]:bg-cyan-600">
            <Sliders className="w-4 h-4 mr-2" /> Actuator
          </TabsTrigger>
          <TabsTrigger value="sensors" className="data-[state=active]:bg-cyan-600">
            <Activity className="w-4 h-4 mr-2" /> Sensors
          </TabsTrigger>
          <TabsTrigger value="qr" className="data-[state=active]:bg-cyan-600">
            <Eye className="w-4 h-4 mr-2" /> QR Detection
          </TabsTrigger>
        </TabsList>

        {/* TELEOP TAB */}
        <TabsContent value="teleop" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
            {/* Joystick Control */}
            <Card className="bg-slate-900/80 border-slate-700 lg:col-span-2">
              <CardHeader>
                <CardTitle className="text-cyan-400 flex items-center gap-2">
                  <Gamepad2 className="w-5 h-5" /> Manual Control
                </CardTitle>
                <CardDescription>Use the joystick to control the robot manually</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="flex flex-col lg:flex-row items-center justify-center gap-8">
                  {/* Movement Joystick */}
                  <VirtualJoystick 
                    onMove={handleJoystickMove} 
                    size={180}
                    label="Movement (X/Y)"
                  />
                  
                  {/* Rotation Control */}
                  <div className="w-64 space-y-6">
                    <RotationSlider 
                      value={velocity.angular_z}
                      onChange={handleRotationChange}
                    />
                    
                    {/* Speed Settings */}
                    <div className="space-y-4 p-4 bg-slate-800/50 rounded-lg">
                      <div className="flex items-center justify-between">
                        <span className="text-sm text-slate-300">Max Speed</span>
                        <span className="text-sm text-cyan-400">{maxSpeed.toFixed(1)} m/s</span>
                      </div>
                      <Slider
                        value={[maxSpeed]}
                        onValueChange={(v) => setMaxSpeed(v[0])}
                        min={0.1}
                        max={1.0}
                        step={0.1}
                      />
                      
                      <div className="flex items-center justify-between">
                        <span className="text-sm text-slate-300">Max Angular</span>
                        <span className="text-sm text-cyan-400">{maxAngular.toFixed(1)} rad/s</span>
                      </div>
                      <Slider
                        value={[maxAngular]}
                        onValueChange={(v) => setMaxAngular(v[0])}
                        min={0.5}
                        max={2.0}
                        step={0.1}
                      />
                    </div>
                  </div>

                  {/* Quick Buttons */}
                  <div className="grid grid-cols-3 gap-2">
                    <div></div>
                    <Button 
                      variant="outline" 
                      className="border-cyan-500 text-cyan-400 hover:bg-cyan-500/20"
                      onMouseDown={() => setVelocity(prev => ({ ...prev, linear_x: 1 }))}
                      onMouseUp={() => setVelocity(prev => ({ ...prev, linear_x: 0 }))}
                      onMouseLeave={() => setVelocity(prev => ({ ...prev, linear_x: 0 }))}
                    >
                      <ArrowUp className="w-5 h-5" />
                    </Button>
                    <div></div>
                    
                    <Button 
                      variant="outline"
                      className="border-cyan-500 text-cyan-400 hover:bg-cyan-500/20"
                      onMouseDown={() => setVelocity(prev => ({ ...prev, linear_y: 1 }))}
                      onMouseUp={() => setVelocity(prev => ({ ...prev, linear_y: 0 }))}
                      onMouseLeave={() => setVelocity(prev => ({ ...prev, linear_y: 0 }))}
                    >
                      <ArrowLeft className="w-5 h-5" />
                    </Button>
                    <Button 
                      variant="destructive"
                      onClick={() => setVelocity({ linear_x: 0, linear_y: 0, angular_z: 0 })}
                    >
                      STOP
                    </Button>
                    <Button 
                      variant="outline"
                      className="border-cyan-500 text-cyan-400 hover:bg-cyan-500/20"
                      onMouseDown={() => setVelocity(prev => ({ ...prev, linear_y: -1 }))}
                      onMouseUp={() => setVelocity(prev => ({ ...prev, linear_y: 0 }))}
                      onMouseLeave={() => setVelocity(prev => ({ ...prev, linear_y: 0 }))}
                    >
                      <ArrowRight className="w-5 h-5" />
                    </Button>
                    
                    <Button 
                      variant="outline"
                      className="border-purple-500 text-purple-400 hover:bg-purple-500/20"
                      onMouseDown={() => setVelocity(prev => ({ ...prev, angular_z: 1 }))}
                      onMouseUp={() => setVelocity(prev => ({ ...prev, angular_z: 0 }))}
                      onMouseLeave={() => setVelocity(prev => ({ ...prev, angular_z: 0 }))}
                    >
                      <RotateCcw className="w-5 h-5" />
                    </Button>
                    <Button 
                      variant="outline"
                      className="border-cyan-500 text-cyan-400 hover:bg-cyan-500/20"
                      onMouseDown={() => setVelocity(prev => ({ ...prev, linear_x: -1 }))}
                      onMouseUp={() => setVelocity(prev => ({ ...prev, linear_x: 0 }))}
                      onMouseLeave={() => setVelocity(prev => ({ ...prev, linear_x: 0 }))}
                    >
                      <ArrowDown className="w-5 h-5" />
                    </Button>
                    <Button 
                      variant="outline"
                      className="border-purple-500 text-purple-400 hover:bg-purple-500/20"
                      onMouseDown={() => setVelocity(prev => ({ ...prev, angular_z: -1 }))}
                      onMouseUp={() => setVelocity(prev => ({ ...prev, angular_z: 0 }))}
                      onMouseLeave={() => setVelocity(prev => ({ ...prev, angular_z: 0 }))}
                    >
                      <RotateCw className="w-5 h-5" />
                    </Button>
                  </div>
                </div>
              </CardContent>
            </Card>

            {/* Quick Actuator Control */}
            <Card className="bg-slate-900/80 border-slate-700">
              <CardHeader>
                <CardTitle className="text-orange-400 flex items-center gap-2">
                  <Sliders className="w-5 h-5" /> Lift Control
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                <LiftVisualizer
                  currentHeight={sensorData.laserHeight}
                  maxHeight={1.5}
                  minHeight={0.05}
                  status={actuatorStatus}
                />
                
                <div className="grid grid-cols-3 gap-2">
                  <Button 
                    className="bg-green-600 hover:bg-green-700"
                    onClick={() => handleActuatorControl('up')}
                  >
                    <ArrowUp className="w-4 h-4 mr-1" /> Up
                  </Button>
                  <Button 
                    variant="destructive"
                    onClick={() => handleActuatorControl('stop')}
                  >
                    Stop
                  </Button>
                  <Button 
                    className="bg-orange-600 hover:bg-orange-700"
                    onClick={() => handleActuatorControl('down')}
                  >
                    <ArrowDown className="w-4 h-4 mr-1" /> Down
                  </Button>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

        {/* LAUNCH CONTROL TAB */}
        <TabsContent value="launch" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
            {launchConfigs.map(config => (
              <LaunchCard
                key={config.id}
                config={config}
                isRunning={launchStates[config.id] || false}
                onLaunch={(params) => handleLaunch(config.id, params)}
                onStop={() => handleStopLaunch(config.id)}
              />
            ))}
          </div>

          {/* Logs */}
          <Card className="bg-slate-900/80 border-slate-700">
            <CardHeader>
              <CardTitle className="text-slate-300">Launch Logs</CardTitle>
            </CardHeader>
            <CardContent>
              <ScrollArea className="h-48 bg-black/50 rounded-lg p-4 font-mono text-xs">
                {logs.length === 0 ? (
                  <span className="text-slate-500">No logs yet...</span>
                ) : (
                  logs.map((log, i) => (
                    <div key={i} className="text-green-400">{log}</div>
                  ))
                )}
              </ScrollArea>
            </CardContent>
          </Card>
        </TabsContent>

        {/* ACTUATOR TAB */}
        <TabsContent value="actuator" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
            <Card className="bg-slate-900/80 border-slate-700">
              <CardHeader>
                <CardTitle className="text-orange-400">Scissor Lift Control</CardTitle>
                <CardDescription>Control the lift with laser height feedback</CardDescription>
              </CardHeader>
              <CardContent className="space-y-6">
                <div className="flex justify-center">
                  <LiftVisualizer
                    currentHeight={sensorData.laserHeight}
                    maxHeight={1.5}
                    minHeight={0.05}
                    status={actuatorStatus}
                  />
                </div>

                <div className="grid grid-cols-3 gap-4">
                  <Button 
                    size="lg"
                    className="h-16 bg-green-600 hover:bg-green-700"
                    onClick={() => handleActuatorControl('up')}
                  >
                    <ArrowUp className="w-6 h-6 mr-2" /> Lift Up
                  </Button>
                  <Button 
                    size="lg"
                    variant="destructive"
                    className="h-16"
                    onClick={() => handleActuatorControl('stop')}
                  >
                    <Square className="w-6 h-6 mr-2" /> STOP
                  </Button>
                  <Button 
                    size="lg"
                    className="h-16 bg-orange-600 hover:bg-orange-700"
                    onClick={() => handleActuatorControl('down')}
                  >
                    <ArrowDown className="w-6 h-6 mr-2" /> Lower
                  </Button>
                </div>
              </CardContent>
            </Card>

            {/* Target Height Control */}
            <Card className="bg-slate-900/80 border-slate-700">
              <CardHeader>
                <CardTitle className="text-cyan-400">Height Target</CardTitle>
                <CardDescription>Set a target height for automatic positioning</CardDescription>
              </CardHeader>
              <CardContent className="space-y-6">
                <div className="space-y-4">
                  <div className="flex items-center justify-between">
                    <span className="text-slate-300">Target Height</span>
                    <span className="text-2xl text-cyan-400 font-bold">1.00m</span>
                  </div>
                  <Slider
                    defaultValue={[1.0]}
                    min={0.1}
                    max={1.5}
                    step={0.05}
                  />
                </div>
                
                <Button className="w-full bg-cyan-600 hover:bg-cyan-700">
                  <MapPin className="w-4 h-4 mr-2" /> Go to Target Height
                </Button>
                
                <div className="grid grid-cols-3 gap-2">
                  <Button variant="outline" className="border-slate-600">
                    Low (0.3m)
                  </Button>
                  <Button variant="outline" className="border-slate-600">
                    Mid (0.8m)
                  </Button>
                  <Button variant="outline" className="border-slate-600">
                    High (1.3m)
                  </Button>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

        {/* SENSORS TAB */}
        <TabsContent value="sensors" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
            <Card className="bg-slate-900/80 border-slate-700">
              <CardHeader className="pb-2">
                <CardTitle className="text-sm text-slate-400">Position (X, Y)</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-cyan-400">
                  ({robotData.position.x.toFixed(3)}, {robotData.position.y.toFixed(3)})
                </div>
              </CardContent>
            </Card>

            <Card className="bg-slate-900/80 border-slate-700">
              <CardHeader className="pb-2">
                <CardTitle className="text-sm text-slate-400">Linear Velocity</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-green-400">
                  {Math.sqrt(robotData.linearVel.x ** 2 + robotData.linearVel.y ** 2).toFixed(3)} m/s
                </div>
              </CardContent>
            </Card>

            <Card className="bg-slate-900/80 border-slate-700">
              <CardHeader className="pb-2">
                <CardTitle className="text-sm text-slate-400">Closest Obstacle</CardTitle>
              </CardHeader>
              <CardContent>
                <div className={`text-2xl font-bold ${sensorData.closestObstacle < 0.5 ? 'text-red-400' : 'text-green-400'}`}>
                  {sensorData.closestObstacle.toFixed(2)} m
                </div>
              </CardContent>
            </Card>

            <Card className="bg-slate-900/80 border-slate-700">
              <CardHeader className="pb-2">
                <CardTitle className="text-sm text-slate-400">Lift Height</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-orange-400">
                  {sensorData.laserHeight.toFixed(3)} m
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

        {/* QR DETECTION TAB */}
        <TabsContent value="qr" className="space-y-4">
          <Card className="bg-slate-900/80 border-slate-700">
            <CardHeader>
              <CardTitle className="text-purple-400 flex items-center gap-2">
                <Camera className="w-5 h-5" /> QR Code Detections
              </CardTitle>
            </CardHeader>
            <CardContent>
              {qrDetections.length === 0 ? (
                <div className="text-center py-8 text-slate-400">
                  <Camera className="w-12 h-12 mx-auto mb-2 opacity-50" />
                  <p>No QR codes detected</p>
                  <p className="text-sm">Start the QR detection launch to begin scanning</p>
                </div>
              ) : (
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                  {qrDetections.map((det, i) => (
                    <Card key={i} className={`border ${det.is_valid ? 'border-green-500 bg-green-500/10' : 'border-red-500 bg-red-500/10'}`}>
                      <CardContent className="p-4">
                        <div className="flex items-center gap-2 mb-2">
                          <Badge variant={det.is_valid ? "default" : "destructive"}>
                            {det.is_valid ? "Valid" : "Invalid"}
                          </Badge>
                          <span className="text-xs text-slate-400">{(det.confidence * 100).toFixed(0)}%</span>
                        </div>
                        <div className="text-sm">
                          <p><span className="text-slate-400">Rack:</span> {det.rack_id}</p>
                          <p><span className="text-slate-400">Shelf:</span> {det.shelf_id}</p>
                          <p><span className="text-slate-400">Item:</span> {det.item_code}</p>
                        </div>
                      </CardContent>
                    </Card>
                  ))}
                </div>
              )}
            </CardContent>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  )
}
