"use client"

import { useEffect, useRef, useState } from "react"

export function RoverAnimation() {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [isVisible, setIsVisible] = useState(false)

  useEffect(() => {
    const showTimer = setTimeout(() => {
      setIsVisible(true)
    }, 2000)

    return () => clearTimeout(showTimer)
  }, [])

  useEffect(() => {
    if (!canvasRef.current || !isVisible) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext("2d")
    if (!ctx) return

    let animationId: number
    let frameCount = 0

    const draw = () => {
      frameCount++
      const t = frameCount / 60 // Time in seconds

      // Clear canvas with dark background
      ctx.fillStyle = "#0a0e0f"
      ctx.fillRect(0, 0, canvas.width, canvas.height)

      const centerX = canvas.width / 2
      const centerY = canvas.height / 2

      // Draw rover body (tracked vehicle shape)
      drawRoverBody(ctx, centerX, centerY, t)

      // Draw sensor scanning effects
      drawSensorScan(ctx, centerX, centerY, t)

      // Draw movement trail
      drawMovementTrail(ctx, centerX, centerY, t)

      // Draw rotating radar/scanner ring
      drawRadarRing(ctx, centerX, centerY, t)

      // Draw detection indicators
      drawDetectionPoints(ctx, centerX, centerY, t)

      animationId = requestAnimationFrame(draw)
    }

    animationId = requestAnimationFrame(draw)

    return () => cancelAnimationFrame(animationId)
  }, [isVisible])

  return (
    <div
      style={{
        opacity: isVisible ? 1 : 0,
        transition: "opacity 0.8s ease-in-out",
      }}
    >
      <canvas
        ref={canvasRef}
        width={400}
        height={300}
        style={{
          borderRadius: "12px",
          border: "2px solid #00ff00",
          boxShadow: "0 0 30px rgba(0, 255, 0, 0.4), inset 0 0 30px rgba(0, 255, 0, 0.1)",
          display: "block",
          backgroundColor: "#0a0e0f",
        }}
      />
    </div>
  )
}

function drawRoverBody(ctx: CanvasRenderingContext2D, x: number, y: number, t: number) {
  // Main body (tracked vehicle)
  ctx.fillStyle = "#00ff00"
  ctx.globalAlpha = 0.9

  // Left track
  ctx.fillRect(x - 45, y - 15, 20, 30)
  ctx.fillStyle = "rgba(0, 255, 0, 0.6)"
  for (let i = 0; i < 3; i++) {
    const offset = ((t * 50 + i * 10) % 20) - 10
    ctx.fillRect(x - 45 + offset, y - 12 + i * 8, 4, 4)
  }

  // Right track
  ctx.fillStyle = "#00ff00"
  ctx.fillRect(x + 25, y - 15, 20, 30)
  ctx.fillStyle = "rgba(0, 255, 0, 0.6)"
  for (let i = 0; i < 3; i++) {
    const offset = ((t * 50 + i * 10) % 20) - 10
    ctx.fillRect(x + 25 + offset, y - 12 + i * 8, 4, 4)
  }

  // Center chassis
  ctx.fillStyle = "#00ff00"
  ctx.globalAlpha = 0.85
  ctx.fillRect(x - 25, y - 10, 50, 20)

  // Sensor head (rotates)
  ctx.save()
  ctx.translate(x, y - 18)
  ctx.rotate(t * 1.5)
  ctx.fillStyle = "#00ff00"
  ctx.fillRect(-12, -8, 24, 16)
  ctx.fillStyle = "rgba(0, 255, 0, 0.5)"
  ctx.fillRect(-8, -4, 16, 8)
  ctx.restore()

  // Scanning laser indicator
  ctx.fillStyle = "#00ff00"
  ctx.globalAlpha = 0.6 + Math.sin(t * 4) * 0.4
  ctx.beginPath()
  ctx.arc(x + 20, y - 20, 4, 0, Math.PI * 2)
  ctx.fill()

  ctx.globalAlpha = 1
}

function drawSensorScan(ctx: CanvasRenderingContext2D, x: number, y: number, t: number) {
  // Scanning lines emanating from rover
  ctx.strokeStyle = "rgba(0, 255, 0, 0.3)"
  ctx.lineWidth = 1

  for (let i = 0; i < 6; i++) {
    const angle = (t * 2 + (i / 6) * Math.PI * 2) % (Math.PI * 2)
    const distance = 80 + Math.sin(t * 3 + i) * 20

    const x1 = x + Math.cos(angle) * 20
    const y1 = y + Math.sin(angle) * 20
    const x2 = x + Math.cos(angle) * distance
    const y2 = y + Math.sin(angle) * distance

    ctx.beginPath()
    ctx.moveTo(x1, y1)
    ctx.lineTo(x2, y2)
    ctx.stroke()

    // Scan wave effect
    ctx.strokeStyle = `rgba(0, 255, 0, ${0.6 * (1 - ((t * 2 + i / 6) % 1))})`
    ctx.beginPath()
    ctx.arc(x, y, 20 + ((t * 30 + i * 5) % 60), 0, Math.PI * 2)
    ctx.stroke()
  }
}

function drawMovementTrail(ctx: CanvasRenderingContext2D, x: number, y: number, t: number) {
  // Movement direction indicator
  ctx.strokeStyle = "rgba(0, 255, 0, 0.4)"
  ctx.lineWidth = 2

  const moveOffset = Math.sin(t * 1.5) * 30
  ctx.beginPath()
  ctx.moveTo(x, y + 20)
  ctx.lineTo(x, y + 20 + moveOffset)
  ctx.stroke()

  // Arrow head
  ctx.fillStyle = "rgba(0, 255, 0, 0.4)"
  ctx.beginPath()
  const arrowSize = 6
  ctx.moveTo(x, y + 20 + moveOffset + arrowSize)
  ctx.lineTo(x - arrowSize, y + 20 + moveOffset)
  ctx.lineTo(x + arrowSize, y + 20 + moveOffset)
  ctx.closePath()
  ctx.fill()
}

function drawRadarRing(ctx: CanvasRenderingContext2D, x: number, y: number, t: number) {
  // Multiple rotating radar rings
  for (let ring = 1; ring <= 3; ring++) {
    ctx.strokeStyle = `rgba(0, 255, 0, ${0.4 / ring})`
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.arc(x, y, 50 + ring * 30, 0, Math.PI * 2)
    ctx.stroke()

    // Rotating scan line on each ring
    const angle = (t * 1.2 * ring) % (Math.PI * 2)
    const radius = 50 + ring * 30
    ctx.strokeStyle = `rgba(0, 255, 0, ${0.6 / ring})`
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(x, y)
    ctx.lineTo(x + Math.cos(angle) * radius, y + Math.sin(angle) * radius)
    ctx.stroke()
  }
}

function drawDetectionPoints(ctx: CanvasRenderingContext2D, x: number, y: number, t: number) {
  // Randomly distributed detection points
  ctx.fillStyle = "#00ff00"

  const points = [
    { angle: 0.5, distance: 60 },
    { angle: 1.2, distance: 75 },
    { angle: 2.1, distance: 85 },
    { angle: 3.8, distance: 70 },
    { angle: 4.9, distance: 80 },
    { angle: 5.5, distance: 65 },
  ]

  points.forEach((point, i) => {
    const pulse = Math.sin(t * 3 + i) * 0.5 + 0.5
    ctx.globalAlpha = pulse * 0.7
    ctx.beginPath()
    ctx.arc(
      x + Math.cos(point.angle) * point.distance,
      y + Math.sin(point.angle) * point.distance,
      3 + pulse * 2,
      0,
      Math.PI * 2,
    )
    ctx.fill()
  })

  ctx.globalAlpha = 1
}
