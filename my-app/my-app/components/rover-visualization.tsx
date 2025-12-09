"use client"

import { useEffect, useRef } from "react"

interface RoverVisualizationProps {
  position?: { x: number; y: number; theta: number }
  velocity?: { linear: number; angular: number }
  liftHeight?: number
  maxLiftHeight?: number
  isMoving?: boolean
  scanActive?: boolean
}

export function RoverVisualization({
  position = { x: 0, y: 0, theta: 0 },
  velocity = { linear: 0, angular: 0 },
  liftHeight = 0,
  maxLiftHeight = 1.5,
  isMoving = false,
  scanActive = false
}: RoverVisualizationProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const animationRef = useRef<number | null>(null)
  const scanAngleRef = useRef(0)
  const wheelRotationRef = useRef(0)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const width = canvas.width
    const height = canvas.height
    const centerX = width / 2
    const centerY = height / 2

    const draw = () => {
      // Clear canvas
      ctx.fillStyle = '#0a0e0f'
      ctx.fillRect(0, 0, width, height)

      // Draw grid
      ctx.strokeStyle = 'rgba(34, 211, 238, 0.1)'
      ctx.lineWidth = 1
      for (let i = 0; i < width; i += 40) {
        ctx.beginPath()
        ctx.moveTo(i, 0)
        ctx.lineTo(i, height)
        ctx.stroke()
      }
      for (let i = 0; i < height; i += 40) {
        ctx.beginPath()
        ctx.moveTo(0, i)
        ctx.lineTo(width, i)
        ctx.stroke()
      }

      // Draw scanning rings if active
      if (scanActive) {
        scanAngleRef.current += 0.02
        for (let i = 1; i <= 3; i++) {
          const radius = 60 + i * 30
          ctx.beginPath()
          ctx.arc(centerX, centerY, radius, scanAngleRef.current, scanAngleRef.current + Math.PI * 1.5)
          ctx.strokeStyle = `rgba(34, 211, 238, ${0.4 - i * 0.1})`
          ctx.lineWidth = 2
          ctx.stroke()
        }
      }

      // Save context for rotation
      ctx.save()
      ctx.translate(centerX, centerY)
      ctx.rotate(position.theta)

      // Draw robot body (top-down view)
      const bodyWidth = 80
      const bodyHeight = 100

      // Main body
      ctx.fillStyle = '#1e293b'
      ctx.strokeStyle = '#22d3ee'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.roundRect(-bodyWidth/2, -bodyHeight/2, bodyWidth, bodyHeight, 8)
      ctx.fill()
      ctx.stroke()

      // Draw lift platform (shows height)
      const liftPercent = liftHeight / maxLiftHeight
      const liftIndicatorHeight = 30
      ctx.fillStyle = `rgba(34, 211, 238, ${0.3 + liftPercent * 0.5})`
      ctx.fillRect(-bodyWidth/2 + 10, -bodyHeight/2 + 10, bodyWidth - 20, liftIndicatorHeight * liftPercent)
      ctx.strokeStyle = '#22d3ee'
      ctx.strokeRect(-bodyWidth/2 + 10, -bodyHeight/2 + 10, bodyWidth - 20, liftIndicatorHeight)

      // Draw mecanum wheels
      const wheelWidth = 15
      const wheelHeight = 25
      const wheelPositions = [
        { x: -bodyWidth/2 - wheelWidth/2 + 5, y: -bodyHeight/2 + 20 },
        { x: bodyWidth/2 + wheelWidth/2 - 5, y: -bodyHeight/2 + 20 },
        { x: -bodyWidth/2 - wheelWidth/2 + 5, y: bodyHeight/2 - 20 },
        { x: bodyWidth/2 + wheelWidth/2 - 5, y: bodyHeight/2 - 20 },
      ]

      // Update wheel rotation if moving
      if (isMoving) {
        wheelRotationRef.current += velocity.linear * 0.1
      }

      wheelPositions.forEach((pos, idx) => {
        ctx.save()
        ctx.translate(pos.x, pos.y)
        
        // Wheel body
        ctx.fillStyle = '#0f172a'
        ctx.strokeStyle = '#4ade80'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.roundRect(-wheelWidth/2, -wheelHeight/2, wheelWidth, wheelHeight, 3)
        ctx.fill()
        ctx.stroke()

        // Mecanum roller pattern (diagonal lines)
        ctx.strokeStyle = 'rgba(74, 222, 128, 0.5)'
        ctx.lineWidth = 1
        const rollerAngle = idx < 2 ? Math.PI/4 : -Math.PI/4
        for (let i = -wheelHeight/2; i < wheelHeight/2; i += 6) {
          ctx.beginPath()
          ctx.moveTo(-wheelWidth/2, i + (wheelRotationRef.current % 6))
          ctx.lineTo(wheelWidth/2, i + (wheelRotationRef.current % 6) + Math.tan(rollerAngle) * wheelWidth)
          ctx.stroke()
        }
        ctx.restore()
      })

      // Draw camera/sensor on front
      ctx.fillStyle = scanActive ? '#22d3ee' : '#64748b'
      ctx.beginPath()
      ctx.arc(0, -bodyHeight/2 - 10, 12, 0, Math.PI * 2)
      ctx.fill()
      
      // Camera lens
      ctx.fillStyle = scanActive ? '#4ade80' : '#334155'
      ctx.beginPath()
      ctx.arc(0, -bodyHeight/2 - 10, 6, 0, Math.PI * 2)
      ctx.fill()

      // Draw direction arrow
      ctx.fillStyle = '#4ade80'
      ctx.beginPath()
      ctx.moveTo(0, -bodyHeight/2 - 30)
      ctx.lineTo(-10, -bodyHeight/2 - 20)
      ctx.lineTo(10, -bodyHeight/2 - 20)
      ctx.closePath()
      ctx.fill()

      // Draw LIDAR indicator on top
      ctx.strokeStyle = '#ef4444'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.arc(0, 0, 15, 0, Math.PI * 2)
      ctx.stroke()
      
      // LIDAR center dot
      ctx.fillStyle = '#ef4444'
      ctx.beginPath()
      ctx.arc(0, 0, 4, 0, Math.PI * 2)
      ctx.fill()

      ctx.restore()

      // Draw position info
      ctx.fillStyle = '#94a3b8'
      ctx.font = '12px monospace'
      ctx.fillText(`X: ${position.x.toFixed(2)}m`, 10, 20)
      ctx.fillText(`Y: ${position.y.toFixed(2)}m`, 10, 35)
      ctx.fillText(`θ: ${(position.theta * 180 / Math.PI).toFixed(1)}°`, 10, 50)
      
      // Draw velocity indicator
      ctx.fillText(`V: ${velocity.linear.toFixed(2)} m/s`, 10, height - 35)
      ctx.fillText(`ω: ${velocity.angular.toFixed(2)} rad/s`, 10, height - 20)

      // Draw lift height indicator on right
      ctx.fillStyle = '#94a3b8'
      ctx.fillText(`Lift: ${(liftHeight * 100).toFixed(0)}cm`, width - 80, 20)

      // Request next frame
      animationRef.current = requestAnimationFrame(draw)
    }

    draw()

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current)
      }
    }
  }, [position, velocity, liftHeight, maxLiftHeight, isMoving, scanActive])

  return (
    <div className="relative">
      <canvas
        ref={canvasRef}
        width={400}
        height={350}
        className="rounded-lg border border-slate-700"
      />
      <div className="absolute bottom-2 right-2 flex gap-2">
        {isMoving && (
          <span className="px-2 py-1 bg-green-500/20 text-green-400 text-xs rounded">
            MOVING
          </span>
        )}
        {scanActive && (
          <span className="px-2 py-1 bg-cyan-500/20 text-cyan-400 text-xs rounded animate-pulse">
            SCANNING
          </span>
        )}
      </div>
    </div>
  )
}
