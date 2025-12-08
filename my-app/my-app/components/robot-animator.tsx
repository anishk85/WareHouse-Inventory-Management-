"use client"

import { useEffect, useRef } from "react"

interface RobotAnimatorProps {
  active?: boolean
}

export function RobotAnimator({ active = true }: RobotAnimatorProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    if (!canvasRef.current || !active) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext("2d")
    if (!ctx) return

    let animationId: number

    const draw = (time: number) => {
      // Clear canvas
      ctx.fillStyle = "#0a0e0f"
      ctx.fillRect(0, 0, canvas.width, canvas.height)

      const centerX = canvas.width / 2
      const centerY = canvas.height / 2
      const t = time / 1000

      // Draw rotating scanner ring
      ctx.strokeStyle = "#00ff00"
      ctx.lineWidth = 2
      ctx.globalAlpha = 0.6
      ctx.beginPath()
      ctx.arc(centerX, centerY, 60 + Math.sin(t * 2) * 10, 0, Math.PI * 2)
      ctx.stroke()

      // Draw pulsing center
      ctx.fillStyle = "#00ff00"
      ctx.globalAlpha = 0.8 + Math.sin(t * 3) * 0.2
      ctx.beginPath()
      ctx.arc(centerX, centerY, 8, 0, Math.PI * 2)
      ctx.fill()

      // Draw rotating points
      for (let i = 0; i < 8; i++) {
        const angle = (t + (i / 8) * Math.PI * 2) * 1.5
        const x = centerX + Math.cos(angle) * 50
        const y = centerY + Math.sin(angle) * 50
        ctx.fillStyle = `rgba(0, 255, 0, ${0.3 + Math.sin(angle) * 0.3})`
        ctx.beginPath()
        ctx.arc(x, y, 4, 0, Math.PI * 2)
        ctx.fill()
      }

      // Draw scanning lines
      ctx.strokeStyle = "rgba(0, 255, 0, 0.2)"
      ctx.lineWidth = 1
      for (let i = 0; i < 12; i++) {
        const angle = (i / 12) * Math.PI * 2 + t * 0.5
        const x1 = centerX + Math.cos(angle) * 40
        const y1 = centerY + Math.sin(angle) * 40
        const x2 = centerX + Math.cos(angle) * 80
        const y2 = centerY + Math.sin(angle) * 80
        ctx.beginPath()
        ctx.moveTo(x1, y1)
        ctx.lineTo(x2, y2)
        ctx.stroke()
      }

      ctx.globalAlpha = 1

      animationId = requestAnimationFrame(draw)
    }

    animationId = requestAnimationFrame(draw)

    return () => cancelAnimationFrame(animationId)
  }, [active])

  return (
    <canvas
      ref={canvasRef}
      width={200}
      height={200}
      style={{
        borderRadius: "8px",
        border: "1px solid #00ff00",
        boxShadow: "0 0 20px rgba(0, 255, 0, 0.3)",
      }}
    />
  )
}
