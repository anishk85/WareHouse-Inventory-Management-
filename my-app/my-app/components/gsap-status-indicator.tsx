"use client"

import { useEffect, useRef } from "react"
import gsap from "gsap"

interface StatusIndicatorProps {
  status: "idle" | "active" | "error" | "warning"
  label: string
}

export function GSAPStatusIndicator({ status, label }: StatusIndicatorProps) {
  const iconRef = useRef<SVGSVGElement>(null)

  useEffect(() => {
    if (!iconRef.current) return

    // Clear existing animations
    gsap.killTweensOf(iconRef.current)

    const statusAnimations = {
      idle: () => {
        // Gentle breathing
        gsap.to(iconRef.current, {
          scale: 1.05,
          duration: 2,
          repeat: -1,
          yoyo: true,
          ease: "sine.inOut",
        })
        gsap.to(iconRef.current, {
          filter: "drop-shadow(0 0 8px rgba(0, 255, 136, 0.4))",
          duration: 2,
          repeat: -1,
          yoyo: true,
        })
      },
      active: () => {
        // Fast pulsing with glow
        gsap.to(iconRef.current, {
          scale: [1, 1.1, 1],
          duration: 0.8,
          repeat: -1,
          ease: "power1.inOut",
        })
        gsap.to(iconRef.current, {
          filter: [
            "drop-shadow(0 0 12px rgba(0, 255, 136, 0.3))",
            "drop-shadow(0 0 24px rgba(0, 255, 136, 0.7))",
            "drop-shadow(0 0 12px rgba(0, 255, 136, 0.3))",
          ],
          duration: 0.8,
          repeat: -1,
        })
      },
      error: () => {
        // Shake with red glow
        gsap.to(iconRef.current, {
          x: "+=5",
          duration: 0.05,
          repeat: 10,
          yoyo: true,
        })
        gsap.to(iconRef.current, {
          filter: "drop-shadow(0 0 16px rgba(255, 0, 85, 0.8))",
          duration: 0.5,
          repeat: -1,
          yoyo: true,
        })
      },
      warning: () => {
        // Slow pulse with yellow glow
        gsap.to(iconRef.current, {
          scale: 1.08,
          duration: 1.5,
          repeat: -1,
          yoyo: true,
          ease: "sine.inOut",
        })
        gsap.to(iconRef.current, {
          filter: "drop-shadow(0 0 14px rgba(255, 170, 0, 0.6))",
          duration: 1.5,
          repeat: -1,
          yoyo: true,
        })
      },
    }

    statusAnimations[status]()
  }, [status])

  const colors = {
    idle: "#00ff88",
    active: "#00ff00",
    error: "#ff0055",
    warning: "#ffaa00",
  }

  return (
    <div className="flex items-center gap-3">
      <svg ref={iconRef} className="w-6 h-6" fill="currentColor" viewBox="0 0 24 24" style={{ color: colors[status] }}>
        <circle cx="12" cy="12" r="10" fill="none" stroke="currentColor" strokeWidth="2" />
        <circle cx="12" cy="12" r="6" fill="currentColor" />
      </svg>
      <span style={{ color: colors[status] }} className="text-sm font-medium">
        {label}
      </span>
    </div>
  )
}
