"use client"

import { useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Rocket, Pause, Play, Square } from "lucide-react"
import { MissionLaunchSequence } from "@/components/mission-launch-sequence"
import { ProgressBarAnimated } from "@/components/progress-bar-animated"

interface Mission {
  id: string
  name: string
  description: string
  status: "ready" | "running" | "paused" | "completed"
  progress: number
}

export default function LaunchControl() {
  const [missions, setMissions] = useState<Mission[]>([
    {
      id: "1",
      name: "Warehouse Perimeter Scan",
      description: "Complete inventory of perimeter zones",
      status: "ready",
      progress: 0,
    },
    {
      id: "2",
      name: "Item Location Mapping",
      description: "Map all detected items in storage",
      status: "running",
      progress: 45,
    },
    {
      id: "3",
      name: "Obstacle Detection",
      description: "Scan for obstacles and hazards",
      status: "ready",
      progress: 0,
    },
  ])

  const [launchingMission, setLaunchingMission] = useState<string | null>(null)

  const launchMission = (id: string) => {
    const mission = missions.find((m) => m.id === id)
    setLaunchingMission(mission?.name || null)

    // Simulate launch sequence delay
    setTimeout(() => {
      setMissions(missions.map((m) => (m.id === id ? { ...m, status: "running" as const, progress: 5 } : m)))
      setLaunchingMission(null)

      // Simulate progress
      const progressInterval = setInterval(() => {
        setMissions((prev) => {
          const updated = prev.map((m) => {
            if (m.id === id && m.status === "running") {
              const newProgress = Math.min(m.progress + Math.random() * 15, 100)
              if (newProgress >= 100) {
                return { ...m, progress: 100, status: "completed" as const }
              }
              return { ...m, progress: newProgress }
            }
            return m
          })
          return updated
        })
      }, 1500)

      return () => clearInterval(progressInterval)
    }, 4500)
  }

  const pauseMission = (id: string) => {
    setMissions(missions.map((m) => (m.id === id ? { ...m, status: "paused" as const } : m)))
  }

  const resumeMission = (id: string) => {
    setMissions(missions.map((m) => (m.id === id ? { ...m, status: "running" as const } : m)))
  }

  const stopMission = (id: string) => {
    setMissions(missions.map((m) => (m.id === id ? { ...m, status: "ready" as const, progress: 0 } : m)))
  }

  const getStatusColor = (status: Mission["status"]) => {
    switch (status) {
      case "ready":
        return "bg-blue-500/20 text-blue-400"
      case "running":
        return "bg-green-500/20 text-green-400 neon-glow"
      case "paused":
        return "bg-yellow-500/20 text-yellow-400"
      case "completed":
        return "bg-purple-500/20 text-purple-400"
    }
  }

  return (
    <div className="p-6 space-y-6 bg-background">
      <MissionLaunchSequence isLaunching={!!launchingMission} missionName={launchingMission || ""} />

      <div className="flex items-center gap-3">
        <Rocket className="text-accent" size={32} />
        <h1 className="text-3xl font-bold neon-green-glow">Launch Control Center</h1>
      </div>

      <div className="grid grid-cols-1 gap-4">
        {missions.map((mission) => (
          <Card
            key={mission.id}
            className="neon-border-green hover:shadow-lg transition-shadow hover:shadow-green-500/20"
          >
            <CardHeader className="pb-3">
              <div className="flex items-start justify-between">
                <div className="flex-1">
                  <CardTitle className="neon-green">{mission.name}</CardTitle>
                  <p className="text-sm text-muted-foreground mt-1">{mission.description}</p>
                </div>
                <span className={`px-3 py-1 rounded-full text-xs font-semibold ${getStatusColor(mission.status)}`}>
                  {mission.status.toUpperCase()}
                </span>
              </div>
            </CardHeader>
            <CardContent className="space-y-4">
              <ProgressBarAnimated progress={mission.progress} status={mission.status} />

              {/* Controls */}
              <div className="flex gap-2 justify-end">
                {mission.status === "ready" && (
                  <Button
                    onClick={() => launchMission(mission.id)}
                    className="bg-green-500 hover:bg-green-600 text-white gap-2 neon-glow"
                  >
                    <Play size={16} /> Launch
                  </Button>
                )}
                {mission.status === "running" && (
                  <>
                    <Button onClick={() => pauseMission(mission.id)} variant="outline" className="gap-2">
                      <Pause size={16} /> Pause
                    </Button>
                    <Button
                      onClick={() => stopMission(mission.id)}
                      variant="outline"
                      className="gap-2 border-red-500/50 text-red-400 hover:bg-red-500/10"
                    >
                      <Square size={16} /> Abort
                    </Button>
                  </>
                )}
                {mission.status === "paused" && (
                  <>
                    <Button
                      onClick={() => resumeMission(mission.id)}
                      className="bg-green-500 hover:bg-green-600 text-white gap-2"
                    >
                      <Play size={16} /> Resume
                    </Button>
                    <Button
                      onClick={() => stopMission(mission.id)}
                      variant="outline"
                      className="gap-2 border-red-500/50 text-red-400 hover:bg-red-500/10"
                    >
                      <Square size={16} /> Abort
                    </Button>
                  </>
                )}
              </div>
            </CardContent>
          </Card>
        ))}
      </div>

      {/* Create New Mission */}
      <Card className="neon-border-green bg-green-500/5">
        <CardHeader>
          <CardTitle className="neon-green">Create New Mission</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="space-y-4">
            <input
              type="text"
              placeholder="Mission Name"
              className="w-full px-4 py-2 bg-input border border-border rounded-lg focus:outline-none focus:ring-2 focus:ring-primary"
            />
            <textarea
              placeholder="Mission Description"
              className="w-full px-4 py-2 bg-input border border-border rounded-lg focus:outline-none focus:ring-2 focus:ring-primary min-h-24"
            />
            <Button className="w-full bg-accent hover:bg-accent/80 text-accent-foreground">
              Create & Queue Mission
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
