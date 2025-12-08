"use client"

import type React from "react"
import { useState, useEffect, useRef } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Map, Crosshair, AlertTriangle } from "lucide-react"
import gsap from "gsap"

export default function NavigationPage() {
  const [destination, setDestination] = useState({ x: 0, y: 0 })
  const [currentPos, setCurrentPos] = useState({ x: 15, y: 25 })
  const [obstacles, setObstacles] = useState([
    { x: 30, y: 40, radius: 5 },
    { x: 60, y: 20, radius: 8 },
    { x: 45, y: 50, radius: 3 },
  ])

  const [navActive, setNavActive] = useState(false)
  const currentPosRef = useRef<HTMLDivElement>(null)
  const pathLineRef = useRef<SVGLineElement>(null)

  useEffect(() => {
    if (!navActive || destination.x === 0) return

    gsap.to(currentPos, {
      x: destination.x,
      y: destination.y,
      duration: 3,
      ease: "power2.inOut",
      onUpdate: () => {
        // Update display manually for real-time feedback
      },
      onComplete: () => {
        setNavActive(false)
      },
    })
  }, [navActive, destination])

  useEffect(() => {
    if (!currentPosRef.current) return

    gsap.to(currentPosRef.current, {
      left: `${currentPos.x}%`,
      top: `${currentPos.y}%`,
      duration: 0.1,
      ease: "power1.out",
    })
  }, [currentPos])

  useEffect(() => {
    const obstacleElements = document.querySelectorAll(".obstacle-marker")
    obstacleElements.forEach((el) => {
      gsap.to(el, {
        boxShadow: [
          "0 0 8px rgba(239, 68, 68, 0.3)",
          "0 0 16px rgba(239, 68, 68, 0.8)",
          "0 0 8px rgba(239, 68, 68, 0.3)",
        ],
        duration: 2,
        repeat: -1,
      })
    })
  }, [obstacles])

  const handleMapClick = (e: React.MouseEvent<HTMLDivElement>) => {
    const rect = e.currentTarget.getBoundingClientRect()
    const x = ((e.clientX - rect.left) / rect.width) * 100
    const y = ((e.clientY - rect.top) / rect.height) * 100
    setDestination({ x, y })
  }

  return (
    <div className="p-6 space-y-6 bg-background">
      <div className="flex items-center gap-3">
        <Map className="text-primary" size={32} />
        <h1 className="text-3xl font-bold neon-green-glow">Navigation Control</h1>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Map Display */}
        <div className="lg:col-span-2">
          <Card className="neon-border-green h-96">
            <CardHeader>
              <CardTitle className="text-sm">Warehouse Map</CardTitle>
            </CardHeader>
            <CardContent className="p-0 h-full">
              <div
                onClick={handleMapClick}
                className="w-full h-80 bg-gradient-to-br from-slate-900 to-slate-950 relative rounded-b-lg cursor-crosshair border-t border-border overflow-hidden"
              >
                {/* Grid */}
                <svg className="absolute inset-0 w-full h-full opacity-10">
                  <defs>
                    <pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse">
                      <path d="M 20 0 L 0 0 0 20" fill="none" stroke="currentColor" strokeWidth="0.5" />
                    </pattern>
                  </defs>
                  <rect width="100%" height="100%" fill="url(#grid)" stroke="currentColor" strokeWidth="1" />
                </svg>

                {/* Obstacles */}
                {obstacles.map((obs, idx) => (
                  <div
                    key={idx}
                    className="obstacle-marker absolute bg-red-500/40 border border-red-500 rounded-full"
                    style={{
                      left: `${obs.x}%`,
                      top: `${obs.y}%`,
                      width: `${obs.radius * 2}%`,
                      height: `${obs.radius * 2}%`,
                      transform: "translate(-50%, -50%)",
                    }}
                  />
                ))}

                {/* Current Position */}
                <div
                  ref={currentPosRef}
                  className="absolute w-3 h-3 bg-green-500 rounded-full border-2 border-green-300 neon-glow"
                  style={{
                    left: `${currentPos.x}%`,
                    top: `${currentPos.y}%`,
                    transform: "translate(-50%, -50%)",
                  }}
                />

                {/* Destination */}
                {destination.x > 0 && (
                  <div
                    className="absolute w-4 h-4 border-2 border-blue-400 animate-pulse"
                    style={{
                      left: `${destination.x}%`,
                      top: `${destination.y}%`,
                      transform: "translate(-50%, -50%) rotate(45deg)",
                    }}
                  />
                )}

                {/* Path Line */}
                {destination.x > 0 && (
                  <svg className="absolute inset-0 w-full h-full">
                    <line
                      ref={pathLineRef}
                      x1={`${currentPos.x}%`}
                      y1={`${currentPos.y}%`}
                      x2={`${destination.x}%`}
                      y2={`${destination.y}%`}
                      stroke="rgba(34, 211, 238, 0.5)"
                      strokeWidth="2"
                      strokeDasharray="5,5"
                    />
                  </svg>
                )}
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Control Panel */}
        <div className="space-y-4">
          <Card className="neon-border-green">
            <CardHeader>
              <CardTitle className="text-sm flex items-center gap-2">
                <Crosshair size={16} /> Navigation
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-4">
              <div>
                <p className="text-xs text-muted-foreground mb-2">Current Position</p>
                <div className="grid grid-cols-2 gap-2">
                  <input
                    type="number"
                    value={currentPos.x.toFixed(1)}
                    readOnly
                    className="px-2 py-1 text-sm bg-muted border border-border rounded"
                  />
                  <input
                    type="number"
                    value={currentPos.y.toFixed(1)}
                    readOnly
                    className="px-2 py-1 text-sm bg-muted border border-border rounded"
                  />
                </div>
              </div>

              <div>
                <p className="text-xs text-muted-foreground mb-2">Target Position</p>
                <div className="grid grid-cols-2 gap-2">
                  <input
                    type="number"
                    value={destination.x.toFixed(1)}
                    onChange={(e) => setDestination({ ...destination, x: Number.parseFloat(e.target.value) || 0 })}
                    className="px-2 py-1 text-sm bg-input border border-border rounded focus:outline-none focus:ring-2 focus:ring-primary"
                  />
                  <input
                    type="number"
                    value={destination.y.toFixed(1)}
                    onChange={(e) => setDestination({ ...destination, y: Number.parseFloat(e.target.value) || 0 })}
                    className="px-2 py-1 text-sm bg-input border border-border rounded focus:outline-none focus:ring-2 focus:ring-primary"
                  />
                </div>
              </div>

              <Button
                onClick={() => setNavActive(!navActive)}
                className={`w-full ${navActive ? "bg-red-500 hover:bg-red-600" : "bg-green-500 hover:bg-green-600"} text-white neon-glow`}
              >
                {navActive ? "Stop Navigation" : "Start Navigation"}
              </Button>
            </CardContent>
          </Card>

          <Card className="neon-border-green">
            <CardHeader>
              <CardTitle className="text-sm flex items-center gap-2">
                <AlertTriangle size={16} /> Obstacles
              </CardTitle>
            </CardHeader>
            <CardContent>
              <p className="text-sm text-muted-foreground mb-3">{obstacles.length} obstacles detected</p>
              <div className="space-y-2 max-h-32 overflow-y-auto">
                {obstacles.map((obs, idx) => (
                  <div key={idx} className="text-xs p-2 bg-red-500/10 rounded border border-red-500/30">
                    Obstacle {idx + 1}: ({obs.x.toFixed(0)}, {obs.y.toFixed(0)})
                  </div>
                ))}
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  )
}
