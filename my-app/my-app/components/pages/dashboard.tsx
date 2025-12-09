"use client"

import type React from "react"
import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Activity, AlertCircle, MapPin, Battery, Wifi, Server } from "lucide-react"
import TelemetryChart from "@/components/telemetry-chart"
import { useRealTime } from "@/components/real-time-provider"
import { RoverVisualization } from "@/components/rover-visualization"
import { GSAPStatusIndicator } from "@/components/gsap-status-indicator"

export default function Dashboard() {
  const { isConnected, subscribe } = useRealTime()
  const [telemetry, setTelemetry] = useState({
    batteryLevel: 87,
    speed: 2.4,
    temperature: 45,
    gpsAccuracy: 1.2,
    signalStrength: 95,
    systemLoad: 42,
  })
  const [robotStatus, setRobotStatus] = useState<"idle" | "active" | "error" | "warning">("active")
  const [robotPosition, setRobotPosition] = useState({ x: 0, y: 0, theta: 0 })
  const [robotVelocity, setRobotVelocity] = useState({ linear: 0, angular: 0 })
  const [liftHeight, setLiftHeight] = useState(0.3)
  const [isScanning, setIsScanning] = useState(false)

  useEffect(() => {
    // Subscribe to telemetry updates from WebSocket
    const unsubscribeTelemetry = subscribe("telemetry", (payload: unknown) => {
      if (typeof payload === "object" && payload !== null) {
        setTelemetry((prev) => ({ ...prev, ...payload }))
      }
    })

    return () => unsubscribeTelemetry()
  }, [subscribe])

  // Fallback to local simulation if not connected
  useEffect(() => {
    if (isConnected) return

    const interval = setInterval(() => {
      setTelemetry((prev) => ({
        ...prev,
        batteryLevel: Math.max(20, prev.batteryLevel + (Math.random() - 0.5) * 2),
        speed: Math.max(0, Math.min(5, prev.speed + (Math.random() - 0.5) * 0.5)),
        temperature: prev.temperature + (Math.random() - 0.5) * 2,
        gpsAccuracy: Math.max(0.5, prev.gpsAccuracy + (Math.random() - 0.5) * 0.3),
        signalStrength: Math.max(50, Math.min(100, prev.signalStrength + (Math.random() - 0.5) * 5)),
        systemLoad: Math.max(10, Math.min(90, prev.systemLoad + (Math.random() - 0.5) * 10)),
      }))
    }, 2000)

    return () => clearInterval(interval)
  }, [isConnected])

  useEffect(() => {
    // Simulate scanning and movement events
    const scanInterval = setInterval(() => {
      setIsScanning(true)
      setTimeout(() => setIsScanning(false), 3000)
    }, 8000)

    // Simulate position changes
    const posInterval = setInterval(() => {
      setRobotPosition(prev => ({
        x: prev.x + (Math.random() - 0.5) * 0.1,
        y: prev.y + (Math.random() - 0.5) * 0.1,
        theta: prev.theta + (Math.random() - 0.5) * 0.1
      }))
      setRobotVelocity({
        linear: Math.random() * 0.5,
        angular: (Math.random() - 0.5) * 0.3
      })
    }, 1000)

    return () => {
      clearInterval(scanInterval)
      clearInterval(posInterval)
    }
  }, [])

  return (
    <div className="p-4 md:p-6 space-y-6" style={{ backgroundColor: "#0a0e0f" }}>
      {/* Rover Visualization */}
      <div
        className="rounded-lg p-4 md:p-6 flex flex-col items-center gap-4 relative"
        style={{ backgroundColor: "#0f1419", borderColor: "#22d3ee", borderWidth: "1px" }}
      >
        <h2 className="text-lg md:text-xl font-bold text-cyan-400">Rover Status</h2>
        <RoverVisualization 
          position={robotPosition}
          velocity={robotVelocity}
          liftHeight={liftHeight}
          maxLiftHeight={1.5}
          isMoving={robotVelocity.linear > 0.1}
          scanActive={isScanning}
        />
      </div>

      {/* Status Indicators */}
      <div className="flex gap-4 flex-wrap">
        <GSAPStatusIndicator status={robotStatus} label="Robot Status" />
        <GSAPStatusIndicator status={telemetry.batteryLevel > 50 ? "active" : "warning"} label="Battery" />
        <GSAPStatusIndicator status={telemetry.temperature < 60 ? "idle" : "warning"} label="Temperature" />
      </div>

      <div className="flex items-center justify-between">
        <h1 className="text-3xl font-bold neon-green-glow">Mission Control Dashboard</h1>
        <div className="flex gap-2 text-sm">
          <span
            className="px-3 py-1 rounded-full neon-green blink-indicator"
            style={{
              backgroundColor: "rgba(0, 255, 0, 0.1)",
              textShadow: "0 0 6px rgba(0, 255, 0, 0.5)",
            }}
          >
            {isConnected ? "● Live WebSocket" : "● Simulated"}
          </span>
          <span
            className="px-3 py-1 rounded-full neon-green"
            style={{
              backgroundColor: "rgba(0, 255, 0, 0.1)",
              textShadow: "0 0 6px rgba(0, 255, 0, 0.5)",
            }}
          >
            ● Connected
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        <StatusCard
          icon={<Battery size={20} />}
          title="Battery"
          value={`${telemetry.batteryLevel.toFixed(1)}%`}
          status={telemetry.batteryLevel > 50 ? "good" : "warning"}
        />
        <StatusCard
          icon={<MapPin size={20} />}
          title="GPS Accuracy"
          value={`${telemetry.gpsAccuracy.toFixed(2)}m`}
          status={telemetry.gpsAccuracy < 2 ? "good" : "warning"}
        />
        <StatusCard
          icon={<Activity size={20} />}
          title="Speed"
          value={`${telemetry.speed.toFixed(1)} m/s`}
          status="good"
        />
        <StatusCard
          icon={<Wifi size={20} />}
          title="Signal"
          value={`${telemetry.signalStrength.toFixed(0)}%`}
          status={telemetry.signalStrength > 70 ? "good" : "warning"}
        />
        <StatusCard
          icon={<Server size={20} />}
          title="System Load"
          value={`${telemetry.systemLoad.toFixed(0)}%`}
          status={telemetry.systemLoad < 70 ? "good" : "warning"}
        />
        <StatusCard
          icon={<AlertCircle size={20} />}
          title="Temperature"
          value={`${telemetry.temperature.toFixed(1)}°C`}
          status={telemetry.temperature < 60 ? "good" : "warning"}
        />
      </div>

      {/* Charts */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <TelemetryChart title="Battery Level" data="battery" />
        <TelemetryChart title="Signal Strength" data="signal" />
        <TelemetryChart title="System Temperature" data="temperature" />
        <TelemetryChart title="GPS Accuracy" data="accuracy" />
      </div>

      {/* Mission Info */}
      <Card className="neon-border-green-strong" style={{ backgroundColor: "#0f1419" }}>
        <CardHeader style={{ borderBottomColor: "#00ff00", borderBottomWidth: "1px" }}>
          <CardTitle className="neon-green-glow">Current Mission</CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <div>
              <p className="text-sm" style={{ color: "#6b7280" }}>
                Mission ID
              </p>
              <p className="text-lg font-bold neon-green" style={{ textShadow: "0 0 8px rgba(0, 255, 0, 0.4)" }}>
                MISSION-001
              </p>
            </div>
            <div>
              <p className="text-sm" style={{ color: "#6b7280" }}>
                Duration
              </p>
              <p className="text-lg font-bold neon-green" style={{ textShadow: "0 0 8px rgba(0, 255, 0, 0.4)" }}>
                2h 34m
              </p>
            </div>
            <div>
              <p className="text-sm" style={{ color: "#6b7280" }}>
                Distance
              </p>
              <p className="text-lg font-bold neon-green" style={{ textShadow: "0 0 8px rgba(0, 255, 0, 0.4)" }}>
                12.4 km
              </p>
            </div>
            <div>
              <p className="text-sm" style={{ color: "#6b7280" }}>
                Status
              </p>
              <p className="text-lg font-bold neon-green rover-scan">Active</p>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}

interface StatusCardProps {
  icon: React.ReactNode
  title: string
  value: string
  status: "good" | "warning" | "error"
}

function StatusCard({ icon, title, value, status }: StatusCardProps) {
  const statusColor = status === "good" ? "#00ff00" : "#ff9900"

  return (
    <Card
      className="neon-border-green pulse-green cursor-pointer transition-all hover:scale-105"
      style={{ backgroundColor: "#0f1419" }}
    >
      <CardContent className="p-4">
        <div className="flex items-start justify-between">
          <div>
            <p className="text-sm" style={{ color: "#6b7280" }}>
              {title}
            </p>
            <p
              className="text-2xl font-bold mt-2 neon-green"
              style={{ color: statusColor, textShadow: "0 0 8px rgba(0, 255, 0, 0.4)" }}
            >
              {value}
            </p>
          </div>
          <div style={{ color: statusColor }}>{icon}</div>
        </div>
      </CardContent>
    </Card>
  )
}
