"use client"

import { useEffect, useRef } from "react"
import gsap from "gsap"

interface ProgressBarAnimatedProps {
  progress: number
  status: "ready" | "running" | "paused" | "completed"
}

export function ProgressBarAnimated({ progress, status }: ProgressBarAnimatedProps) {
  const barRef = useRef<HTMLDivElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    if (!barRef.current) return

    // Animate progress bar smoothly
    gsap.to(barRef.current, {
      width: `${progress}%`,
      duration: 0.8,
      ease: "power2.out",
    })
  }, [progress])

  // Determine color and glow effect based on status
  const getStatusStyles = () => {
    const baseStyles = {
      ready: { color: "#00a5ff", glow: "drop-shadow(0 0 8px rgba(0, 165, 255, 0.6))" },
      running: { color: "#00ff00", glow: "drop-shadow(0 0 12px rgba(0, 255, 0, 0.8))" },
      paused: { color: "#ffaa00", glow: "drop-shadow(0 0 8px rgba(255, 170, 0, 0.6))" },
      completed: { color: "#7fff00", glow: "drop-shadow(0 0 10px rgba(127, 255, 0, 0.7))" },
    }
    return baseStyles[status]
  }

  const styles = getStatusStyles()

  return (
    <div ref={containerRef} className="space-y-2">
      <div className="flex justify-between text-xs text-muted-foreground">
        <span>Progress</span>
        <span>{Math.round(progress)}%</span>
      </div>
      <div className="h-2 bg-muted rounded-full overflow-hidden border border-gray-700">
        <div
          ref={barRef}
          className="h-full rounded-full transition-all duration-300"
          style={{
            backgroundColor: styles.color,
            filter: styles.glow,
            width: "0%",
          }}
        />
      </div>
    </div>
  )
}
