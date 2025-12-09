"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Database, Trash2, Edit } from "lucide-react"

interface Record {
  id: string
  timestamp: string
  missionId: string
  type: string
  data: string
}

export default function DatabasePage() {
  const [records, setRecords] = useState<Record[]>([])

  const [filter, setFilter] = useState("all")
  const [loading, setLoading] = useState(false)
  const [resource, setResource] = useState<'missions' | 'qr' | 'waypoints'>('missions')

  useEffect(() => {
    // Load initial sample or remote records when component mounts
    fetchFromDb(resource)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  async function fetchFromDb(r: string) {
    setLoading(true)
    try {
      const res = await fetch(`/api/database?resource=${r}`)
      const json = await res.json()
      if (json.ok && Array.isArray(json.rows)) {
        // Map rows to UI records where possible
        const mapped = json.rows.map((row: any, idx: number) => {
          // Heuristic mapping
          const id = row.id ?? row._id ?? String(idx + 1)
          const timestamp = row.detected_at ?? row.started_at ?? row.created_at ?? new Date().toISOString()
          const missionId = row.mission_id ?? row.missionid ?? 'N/A'
          let type = 'Record'
          if (r === 'missions') type = 'Mission'
          if (r === 'qr') type = 'QR'
          if (r === 'waypoints') type = 'Waypoint'
          const data = JSON.stringify(row)
          return { id: String(id), timestamp: String(timestamp), missionId: String(missionId), type, data }
        })
        setRecords(mapped)
      } else {
        setRecords([])
      }
    } catch (e) {
      console.error('DB fetch error', e)
    } finally {
      setLoading(false)
    }
  }

  const filteredRecords = records.filter((record) => {
    if (filter === "all") return true
    return record.type.toLowerCase() === filter.toLowerCase()
  })

  const deleteRecord = (id: string) => {
    setRecords(records.filter((r) => r.id !== id))
  }

  return (
    <div className="p-6 space-y-6 bg-background">
      <div className="flex items-center gap-3">
        <Database className="text-secondary" size={32} />
        <h1 className="text-3xl font-bold neon-purple">Database Records</h1>
      </div>

      <div className="flex gap-2 items-center">
        <select
          aria-label="resource"
          value={resource}
          onChange={(e) => setResource(e.target.value as any)}
          className="bg-background border px-3 py-2 rounded text-sm"
        >
          <option value="missions">Missions</option>
          <option value="qr">QR Detections</option>
          <option value="waypoints">Waypoints</option>
        </select>
        <Button onClick={() => fetchFromDb(resource)} disabled={loading}>
          {loading ? 'Loading...' : 'Load from DB'}
        </Button>
      </div>

      {/* Stats */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <Card className="neon-border-cyan">
          <CardContent className="p-4">
            <p className="text-sm text-muted-foreground">Total Records</p>
            <p className="text-2xl font-bold neon-cyan mt-1">{records.length}</p>
          </CardContent>
        </Card>
        <Card className="neon-border-purple">
          <CardContent className="p-4">
            <p className="text-sm text-muted-foreground">Missions Logged</p>
            <p className="text-2xl font-bold neon-purple mt-1">{new Set(records.map((r) => r.missionId)).size}</p>
          </CardContent>
        </Card>
        <Card className="neon-border-green">
          <CardContent className="p-4">
            <p className="text-sm text-muted-foreground">Event Types</p>
            <p className="text-2xl font-bold neon-green mt-1">{new Set(records.map((r) => r.type)).size}</p>
          </CardContent>
        </Card>
      </div>

      {/* Filters & Controls */}
      <div className="flex gap-2 flex-wrap">
        {["all", "Telemetry", "Event", "Navigation"].map((type) => (
          <Button
            key={type}
            onClick={() => setFilter(type)}
            variant={filter === type ? "default" : "outline"}
            className={filter === type ? "bg-primary text-primary-foreground" : ""}
          >
            {type === "all" ? "All Records" : type}
          </Button>
        ))}
      </div>

      {/* Records Table */}
      <Card className="neon-border-cyan">
        <CardHeader>
          <CardTitle className="text-sm">Recent Records</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="overflow-x-auto">
            <table className="w-full text-sm">
              <thead>
                <tr className="border-b border-border">
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Timestamp</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Mission ID</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Type</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Data</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Actions</th>
                </tr>
              </thead>
              <tbody>
                {filteredRecords.map((record) => (
                  <tr key={record.id} className="border-b border-border hover:bg-muted/30 transition-colors">
                    <td className="py-3 px-4 text-muted-foreground">{record.timestamp}</td>
                    <td className="py-3 px-4 text-cyan-400 font-semibold">{record.missionId}</td>
                    <td className="py-3 px-4">
                      <span
                        className={`px-2 py-1 rounded text-xs font-semibold ${
                          record.type === "Telemetry"
                            ? "bg-blue-500/20 text-blue-400"
                            : record.type === "Event"
                              ? "bg-yellow-500/20 text-yellow-400"
                              : "bg-green-500/20 text-green-400"
                        }`}
                      >
                        {record.type}
                      </span>
                    </td>
                    <td className="py-3 px-4 text-purple-400">{record.data}</td>
                    <td className="py-3 px-4">
                      <div className="flex gap-2">
                        <Button variant="ghost" size="sm" className="text-muted-foreground hover:text-primary">
                          <Edit size={14} />
                        </Button>
                        <Button
                          variant="ghost"
                          size="sm"
                          className="text-muted-foreground hover:text-red-400"
                          onClick={() => deleteRecord(record.id)}
                        >
                          <Trash2 size={14} />
                        </Button>
                      </div>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
