"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Database, Trash2, Edit, RefreshCw } from "lucide-react"

interface Record {
  id: string
  timestamp: string
  missionId: string
  type: string
  data: string
  details?: any
}

// Dummy data generator for testing
function generateDummyData(resource: string): Record[] {
  if (resource === 'missions') {
    return [
      {
        id: '1',
        timestamp: new Date(Date.now() - 300000).toISOString(),
        missionId: '1',
        type: 'Mission',
        data: 'Test Mission 1',
        details: { name: 'Test Mission 1', status: 'completed', duration: '5 mins' }
      },
      {
        id: '2',
        timestamp: new Date(Date.now() - 600000).toISOString(),
        missionId: '2',
        type: 'Mission',
        data: 'Warehouse Scan',
        details: { name: 'Warehouse Scan', status: 'in_progress', duration: '10 mins' }
      },
      {
        id: '3',
        timestamp: new Date(Date.now() - 900000).toISOString(),
        missionId: '3',
        type: 'Mission',
        data: 'Rack Detection',
        details: { name: 'Rack Detection', status: 'completed', duration: '8 mins' }
      },
    ]
  } else if (resource === 'qr') {
    return [
      {
        id: '1',
        timestamp: new Date(Date.now() - 60000).toISOString(),
        missionId: '1',
        type: 'QR Detection',
        data: 'RACK_1_SHELF_1_ABC123',
        details: { rack: 1, shelf: 1, item: 'ABC123', confidence: 0.95 }
      },
      {
        id: '2',
        timestamp: new Date(Date.now() - 120000).toISOString(),
        missionId: '1',
        type: 'QR Detection',
        data: 'RACK_1_SHELF_2_DEF456',
        details: { rack: 1, shelf: 2, item: 'DEF456', confidence: 0.92 }
      },
      {
        id: '3',
        timestamp: new Date(Date.now() - 180000).toISOString(),
        missionId: '2',
        type: 'QR Detection',
        data: 'RACK_2_SHELF_1_GHI789',
        details: { rack: 2, shelf: 1, item: 'GHI789', confidence: 0.88 }
      },
      {
        id: '4',
        timestamp: new Date(Date.now() - 240000).toISOString(),
        missionId: '2',
        type: 'QR Detection',
        data: 'RACK_2_SHELF_3_JKL012',
        details: { rack: 2, shelf: 3, item: 'JKL012', confidence: 0.97 }
      },
      {
        id: '5',
        timestamp: new Date(Date.now() - 300000).toISOString(),
        missionId: '3',
        type: 'QR Detection',
        data: 'RACK_3_SHELF_2_MNO345',
        details: { rack: 3, shelf: 2, item: 'MNO345', confidence: 0.91 }
      },
    ]
  } else if (resource === 'waypoints') {
    return [
      {
        id: '1',
        timestamp: new Date().toISOString(),
        missionId: '1',
        type: 'Waypoint',
        data: 'WP-1: (0.0, 0.0)',
        details: { seq: 1, x: 0.0, y: 0.0, z: 0.0, orientation: 'NORTH' }
      },
      {
        id: '2',
        timestamp: new Date().toISOString(),
        missionId: '1',
        type: 'Waypoint',
        data: 'WP-2: (5.0, 0.0)',
        details: { seq: 2, x: 5.0, y: 0.0, z: 0.0, orientation: 'NORTH' }
      },
      {
        id: '3',
        timestamp: new Date().toISOString(),
        missionId: '1',
        type: 'Waypoint',
        data: 'WP-3: (5.0, 5.0)',
        details: { seq: 3, x: 5.0, y: 5.0, z: 0.0, orientation: 'EAST' }
      },
      {
        id: '4',
        timestamp: new Date().toISOString(),
        missionId: '1',
        type: 'Waypoint',
        data: 'WP-4: (0.0, 5.0)',
        details: { seq: 4, x: 0.0, y: 5.0, z: 0.0, orientation: 'SOUTH' }
      },
      {
        id: '5',
        timestamp: new Date().toISOString(),
        missionId: '1',
        type: 'Waypoint',
        data: 'WP-5: (0.0, 0.0)',
        details: { seq: 5, x: 0.0, y: 0.0, z: 0.0, orientation: 'WEST' }
      },
    ]
  }
  return []
}

export default function DatabasePage() {
  const [records, setRecords] = useState<Record[]>([])
  const [filter, setFilter] = useState("all")
  const [loading, setLoading] = useState(false)
  const [resource, setResource] = useState<'missions' | 'qr' | 'waypoints'>('missions')
  const [useDummyData, setUseDummyData] = useState(true)

  useEffect(() => {
    // Load initial sample or remote records when component mounts
    fetchFromDb(resource)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [resource])

  async function fetchFromDb(r: string) {
    setLoading(true)
    
    // Try to fetch from actual database first
    if (!useDummyData) {
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
            if (r === 'qr') type = 'QR Detection'
            if (r === 'waypoints') type = 'Waypoint'
            const data = JSON.stringify(row)
            return { id: String(id), timestamp: String(timestamp), missionId: String(missionId), type, data, details: row }
          })
          setRecords(mapped)
          setLoading(false)
          return
        }
      } catch (e) {
        console.error('DB fetch error, falling back to dummy data:', e)
      }
    }
    
    // Use dummy data
    const dummyRecords = generateDummyData(r)
    setRecords(dummyRecords)
    setLoading(false)
  }

  const filteredRecords = records.filter((record) => {
    if (filter === "all") return true
    return record.type.toLowerCase() === filter.toLowerCase()
  })

  const deleteRecord = (id: string) => {
    setRecords(records.filter((r) => r.id !== id))
  }

  // Get statistics
  const totalRecords = records.length
  const uniqueMissions = new Set(records.map((r) => r.missionId)).size
  const recordTypes = new Set(records.map((r) => r.type)).size

  return (
    <div className="p-6 space-y-6 bg-background">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-3">
          <Database className="text-secondary" size={32} />
          <h1 className="text-3xl font-bold neon-purple">Database Records</h1>
        </div>
      </div>

      {/* Controls */}
      <div className="flex gap-3 items-center flex-wrap">
        <select
          aria-label="resource"
          value={resource}
          onChange={(e) => setResource(e.target.value as any)}
          className="bg-background border border-border px-3 py-2 rounded text-sm text-foreground"
        >
          <option value="missions">📋 Missions</option>
          <option value="qr">🔍 QR Detections</option>
          <option value="waypoints">🗺️ Waypoints</option>
        </select>
        <Button 
          onClick={() => fetchFromDb(resource)} 
          disabled={loading}
          className="flex items-center gap-2"
        >
          <RefreshCw size={16} className={loading ? 'animate-spin' : ''} />
          {loading ? 'Loading...' : 'Refresh'}
        </Button>
      </div>

      {/* Stats Cards */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <Card className="neon-border-cyan">
          <CardContent className="p-4">
            <p className="text-sm text-muted-foreground">Total Records</p>
            <p className="text-3xl font-bold neon-cyan mt-2">{totalRecords}</p>
          </CardContent>
        </Card>
        <Card className="neon-border-purple">
          <CardContent className="p-4">
            <p className="text-sm text-muted-foreground">Unique Missions</p>
            <p className="text-3xl font-bold neon-purple mt-2">{uniqueMissions}</p>
          </CardContent>
        </Card>
        <Card className="neon-border-green">
          <CardContent className="p-4">
            <p className="text-sm text-muted-foreground">Record Types</p>
            <p className="text-3xl font-bold neon-green mt-2">{recordTypes}</p>
          </CardContent>
        </Card>
      </div>

      {/* Filter Buttons */}
      <div className="flex gap-2 flex-wrap">
        <Button
          onClick={() => setFilter("all")}
          variant={filter === "all" ? "default" : "outline"}
          className={`text-xs ${filter === "all" ? "bg-primary text-primary-foreground" : ""}`}
        >
          All Records
        </Button>
        {Array.from(new Set(records.map(r => r.type))).map((type) => (
          <Button
            key={type}
            onClick={() => setFilter(type)}
            variant={filter === type ? "default" : "outline"}
            className={`text-xs ${filter === type ? "bg-primary text-primary-foreground" : ""}`}
          >
            {type}
          </Button>
        ))}
      </div>

      {/* Records Table */}
      <Card className="neon-border-cyan overflow-hidden">
        <CardHeader className="pb-3">
          <CardTitle className="text-lg flex items-center gap-2">
            <span>Recent Records</span>
            <span className="text-sm font-normal text-muted-foreground">({filteredRecords.length} items)</span>
          </CardTitle>
        </CardHeader>
        <CardContent className="p-0">
          <div className="overflow-x-auto">
            <table className="w-full text-sm">
              <thead>
                <tr className="border-b border-border bg-muted/30">
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">ID</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Timestamp</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Mission</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Type</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Data</th>
                  <th className="text-left py-3 px-4 text-muted-foreground font-semibold">Actions</th>
                </tr>
              </thead>
              <tbody>
                {filteredRecords.length > 0 ? (
                  filteredRecords.map((record) => (
                    <tr key={record.id} className="border-b border-border hover:bg-muted/50 transition-colors">
                      <td className="py-3 px-4 text-muted-foreground font-mono text-xs">{record.id}</td>
                      <td className="py-3 px-4 text-muted-foreground text-xs">{new Date(record.timestamp).toLocaleString()}</td>
                      <td className="py-3 px-4 text-cyan-400 font-semibold">{record.missionId}</td>
                      <td className="py-3 px-4">
                        <span className={`px-2 py-1 rounded text-xs font-semibold inline-block ${
                          record.type === "Mission"
                            ? "bg-blue-500/20 text-blue-400"
                            : record.type === "QR Detection"
                              ? "bg-yellow-500/20 text-yellow-400"
                              : "bg-green-500/20 text-green-400"
                        }`}>
                          {record.type}
                        </span>
                      </td>
                      <td className="py-3 px-4 text-purple-400 font-mono text-xs max-w-xs truncate">
                        {typeof record.data === 'string' ? record.data : JSON.stringify(record.data)}
                      </td>
                      <td className="py-3 px-4">
                        <div className="flex gap-2">
                          <Button 
                            variant="ghost" 
                            size="sm" 
                            className="text-muted-foreground hover:text-primary h-7 w-7 p-0"
                            title="Edit record"
                          >
                            <Edit size={14} />
                          </Button>
                          <Button
                            variant="ghost"
                            size="sm"
                            className="text-muted-foreground hover:text-red-400 h-7 w-7 p-0"
                            onClick={() => deleteRecord(record.id)}
                            title="Delete record"
                          >
                            <Trash2 size={14} />
                          </Button>
                        </div>
                      </td>
                    </tr>
                  ))
                ) : (
                  <tr>
                    <td colSpan={6} className="py-8 px-4 text-center text-muted-foreground">
                      No records found for the selected filter
                    </td>
                  </tr>
                )}
              </tbody>
            </table>
          </div>
        </CardContent>
      </Card>

      {/* JSON Preview */}
      {filteredRecords.length > 0 && (
        <Card className="neon-border-purple">
          <CardHeader>
            <CardTitle className="text-sm">First Record (JSON Preview)</CardTitle>
          </CardHeader>
          <CardContent>
            <pre className="bg-muted p-4 rounded text-xs overflow-x-auto text-green-400">
              {JSON.stringify(filteredRecords[0].details || filteredRecords[0], null, 2)}
            </pre>
          </CardContent>
        </Card>
      )}
    </div>
  )
}
