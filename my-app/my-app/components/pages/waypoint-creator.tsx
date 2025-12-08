"use client"

import { useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { MapPin, Trash2, Play } from "lucide-react"

interface Waypoint {
  id: string
  name: string
  x: number
  y: number
  action: string
}

export default function WaypointCreator() {
  const [waypoints, setWaypoints] = useState<Waypoint[]>([
    { id: "1", name: "Zone A", x: 10, y: 20, action: "Scan" },
    { id: "2", name: "Zone B", x: 50, y: 40, action: "Grab" },
  ])

  const [newWaypoint, setNewWaypoint] = useState({ name: "", x: "", y: "", action: "Scan" })

  const addWaypoint = () => {
    if (newWaypoint.name && newWaypoint.x && newWaypoint.y) {
      setWaypoints([
        ...waypoints,
        {
          id: Date.now().toString(),
          name: newWaypoint.name,
          x: Number.parseFloat(newWaypoint.x),
          y: Number.parseFloat(newWaypoint.y),
          action: newWaypoint.action,
        },
      ])
      setNewWaypoint({ name: "", x: "", y: "", action: "Scan" })
    }
  }

  const removeWaypoint = (id: string) => {
    setWaypoints(waypoints.filter((w) => w.id !== id))
  }

  return (
    <div className="p-6 space-y-6 bg-background">
      <div className="flex items-center gap-3">
        <MapPin className="text-accent" size={32} />
        <h1 className="text-3xl font-bold neon-green">Waypoint Creator</h1>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Create Waypoint */}
        <Card className="neon-border-green">
          <CardHeader>
            <CardTitle className="text-sm neon-green">Create New Waypoint</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <input
              type="text"
              placeholder="Waypoint Name"
              value={newWaypoint.name}
              onChange={(e) => setNewWaypoint({ ...newWaypoint, name: e.target.value })}
              className="w-full px-3 py-2 text-sm bg-input border border-border rounded focus:outline-none focus:ring-2 focus:ring-accent"
            />

            <div className="grid grid-cols-2 gap-2">
              <input
                type="number"
                placeholder="X Coordinate"
                value={newWaypoint.x}
                onChange={(e) => setNewWaypoint({ ...newWaypoint, x: e.target.value })}
                className="px-3 py-2 text-sm bg-input border border-border rounded focus:outline-none focus:ring-2 focus:ring-accent"
              />
              <input
                type="number"
                placeholder="Y Coordinate"
                value={newWaypoint.y}
                onChange={(e) => setNewWaypoint({ ...newWaypoint, y: e.target.value })}
                className="px-3 py-2 text-sm bg-input border border-border rounded focus:outline-none focus:ring-2 focus:ring-accent"
              />
            </div>

            <select
              value={newWaypoint.action}
              onChange={(e) => setNewWaypoint({ ...newWaypoint, action: e.target.value })}
              className="w-full px-3 py-2 text-sm bg-input border border-border rounded focus:outline-none focus:ring-2 focus:ring-accent"
            >
              <option>Scan</option>
              <option>Grab</option>
              <option>Place</option>
              <option>Inspect</option>
            </select>

            <Button onClick={addWaypoint} className="w-full bg-accent hover:bg-accent/80 text-accent-foreground">
              Add Waypoint
            </Button>
          </CardContent>
        </Card>

        {/* Waypoint List */}
        <Card className="neon-border-purple">
          <CardHeader>
            <CardTitle className="text-sm neon-purple">Route Waypoints ({waypoints.length})</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="space-y-2 max-h-96 overflow-y-auto">
              {waypoints.map((wp, idx) => (
                <div
                  key={wp.id}
                  className="p-3 bg-card border border-border rounded-lg neon-border-cyan hover:shadow-lg transition-shadow"
                >
                  <div className="flex items-start justify-between gap-2">
                    <div className="flex-1">
                      <div className="flex items-center gap-2">
                        <span className="px-2 py-1 text-xs font-bold bg-primary text-primary-foreground rounded">
                          {idx + 1}
                        </span>
                        <span className="font-semibold text-cyan-400">{wp.name}</span>
                      </div>
                      <p className="text-xs text-muted-foreground mt-1">
                        Pos: ({wp.x}, {wp.y}) • Action: <span className="text-accent">{wp.action}</span>
                      </p>
                    </div>
                    <Button
                      onClick={() => removeWaypoint(wp.id)}
                      variant="ghost"
                      size="sm"
                      className="text-red-400 hover:bg-red-500/10"
                    >
                      <Trash2 size={14} />
                    </Button>
                  </div>
                </div>
              ))}
            </div>

            <Button className="w-full mt-4 bg-green-500 hover:bg-green-600 text-white gap-2">
              <Play size={16} /> Execute Route
            </Button>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
