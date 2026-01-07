"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Activity, AlertTriangle, CheckCircle, Zap } from "lucide-react"
import MetricCard from "@/components/MetricCard" // Declare the MetricCard variable

interface SystemStatus {
  cpuUsage: number
  memoryUsage: number
  diskUsage: number
  networkLatency: number
  uptime: string
}

export default function SystemMonitor() {
  const [system, setSystem] = useState<SystemStatus>({
    cpuUsage: 42,
    memoryUsage: 58,
    diskUsage: 35,
    networkLatency: 12,
    uptime: "48h 23m",
  })

  const [processes, setProcesses] = useState([
    { name: "Navigation Engine", cpu: 25, memory: 15, status: "running" },
    { name: "Vision Processing", cpu: 35, memory: 28, status: "running" },
    { name: "Telemetry Logger", cpu: 8, memory: 5, status: "running" },
    { name: "Database Handler", cpu: 12, memory: 20, status: "running" },
  ])

  useEffect(() => {
    const interval = setInterval(() => {
      setSystem((prev) => ({
        ...prev,
        cpuUsage: Math.max(20, Math.min(90, prev.cpuUsage + (Math.random() - 0.5) * 10)),
        memoryUsage: Math.max(30, Math.min(85, prev.memoryUsage + (Math.random() - 0.5) * 8)),
        networkLatency: Math.max(5, Math.min(50, prev.networkLatency + (Math.random() - 0.5) * 5)),
      }))
    }, 2000)

    return () => clearInterval(interval)
  }, [])

  const getStatusColor = (value: number) => {
    if (value < 50) return "text-green-400 neon-green"
    if (value < 75) return "text-yellow-400"
    return "text-red-400"
  }

  return (
    <div className="p-6 space-y-6 bg-background">
      <div className="flex items-center gap-3">
        <Activity className="text-accent" size={32} />
        <h1 className="text-3xl font-bold neon-cyan">System Monitor</h1>
      </div>

      {/* System Metrics Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
        <MetricCard
          title="CPU Usage"
          value={system.cpuUsage.toFixed(1)}
          unit="%"
          status={getStatusColor(system.cpuUsage)}
        />
        <MetricCard
          title="Memory Usage"
          value={system.memoryUsage.toFixed(1)}
          unit="%"
          status={getStatusColor(system.memoryUsage)}
        />
        <MetricCard
          title="Disk Usage"
          value={system.diskUsage.toFixed(1)}
          unit="%"
          status={getStatusColor(system.diskUsage)}
        />
        <MetricCard
          title="Network Latency"
          value={system.networkLatency.toFixed(0)}
          unit="ms"
          status={system.networkLatency > 20 ? "text-yellow-400" : "text-green-400 neon-green"}
        />
      </div>

      {/* Status Overview */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <Card className="neon-border-green">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <CheckCircle className="text-green-400 neon-green" size={24} />
              <div>
                <p className="text-sm text-muted-foreground">System Status</p>
                <p className="text-lg font-bold text-green-400">Operational</p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="neon-border-cyan">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <Zap className="text-cyan-400" size={24} />
              <div>
                <p className="text-sm text-muted-foreground">Uptime</p>
                <p className="text-lg font-bold text-cyan-400">{system.uptime}</p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="neon-border-purple">
          <CardContent className="p-4">
            <div className="flex items-center gap-3">
              <AlertTriangle className="text-purple-400" size={24} />
              <div>
                <p className="text-sm text-muted-foreground">Alerts</p>
                <p className="text-lg font-bold text-purple-400">0</p>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Process Monitor */}
      <Card className="neon-border-cyan">
        <CardHeader>
          <CardTitle className="text-sm">Running Processes</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="space-y-3">
            {processes.map((proc, idx) => (
              <div key={idx} className="p-3 bg-card border border-border rounded-lg">
                <div className="flex items-center justify-between mb-2">
                  <p className="font-semibold text-cyan-400">{proc.name}</p>
                  <span className="px-2 py-1 text-xs bg-green-500/20 text-green-400 rounded neon-green">
                    {proc.status}
                  </span>
                </div>
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <p className="text-muted-foreground">CPU</p>
                    <div className="mt-1 h-2 bg-muted rounded-full overflow-hidden">
                      <div className="h-full bg-cyan-500" style={{ width: `${proc.cpu}%` }} />
                    </div>
                    <p className="text-xs text-cyan-400 mt-1">{proc.cpu}%</p>
                  </div>
                  <div>
                    <p className="text-muted-foreground">Memory</p>
                    <div className="mt-1 h-2 bg-muted rounded-full overflow-hidden">
                      <div className="h-full bg-purple-500" style={{ width: `${proc.memory}%` }} />
                    </div>
                    <p className="text-xs text-purple-400 mt-1">{proc.memory}%</p>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
