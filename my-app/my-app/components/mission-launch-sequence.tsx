"use client"

import { useEffect, useRef } from "react"
import gsap from "gsap"

interface MissionLaunchSequenceProps {
  isLaunching: boolean
  missionName: string
}

export function MissionLaunchSequence({ isLaunching, missionName }: MissionLaunchSequenceProps) {
  const containerRef = useRef<HTMLDivElement>(null)
  const timelineRef = useRef<gsap.core.Timeline | null>(null)

  useEffect(() => {
    if (!containerRef.current) return

    if (isLaunching) {
      if (timelineRef.current) timelineRef.current.kill()

      timelineRef.current = gsap.timeline()

      // Countdown numbers
      const numbers = containerRef.current.querySelectorAll(".countdown-number")
      numbers.forEach((num, idx) => {
        timelineRef.current?.from(
          num,
          {
            scale: 5,
            opacity: 0,
            duration: 0.5,
            ease: "power4.out",
          },
          idx * 0.6,
        )

        timelineRef.current?.to(
          num,
          {
            scale: 0,
            opacity: 0,
            duration: 0.3,
          },
          idx * 0.6 + 0.5,
        )
      })

      // Energy bars charge
      const energyBars = containerRef.current.querySelectorAll(".energy-bar")
      energyBars.forEach((bar, idx) => {
        timelineRef.current?.fromTo(
          bar,
          { scaleX: 0 },
          {
            scaleX: 1,
            duration: 2.5,
            ease: "power2.inOut",
            transformOrigin: "left",
          },
          0.5 + idx * 0.3,
        )
      })

      // Particles burst
      const particles = containerRef.current.querySelectorAll(".particle")
      timelineRef.current?.to(
        particles,
        {
          scale: 2,
          opacity: 0,
          duration: 0.8,
          stagger: 0.05,
        },
        3.5,
      )

      // Flash effect
      const flash = containerRef.current.querySelector(".flash")
      if (flash) {
        timelineRef.current?.to(
          flash,
          {
            opacity: [0, 1, 0],
            duration: 0.2,
            repeat: 2,
          },
          3,
        )
      }
    }
  }, [isLaunching])

  if (!isLaunching) return null

  return (
    <div
      ref={containerRef}
      className="fixed inset-0 flex items-center justify-center z-50 pointer-events-none bg-black/50 backdrop-blur-sm"
    >
      {/* Flash */}
      <div className="flash absolute inset-0 bg-white opacity-0" />

      {/* Countdown */}
      <div className="relative space-y-4">
        {["3", "2", "1", "LAUNCH"].map((num, idx) => (
          <div
            key={idx}
            className="countdown-number text-6xl font-bold text-center absolute left-1/2 transform -translate-x-1/2"
            style={{ color: "#00ff00", textShadow: "0 0 20px rgba(0, 255, 0, 0.8)" }}
          >
            {num}
          </div>
        ))}
      </div>

      {/* Energy Bars */}
      <div className="absolute bottom-32 left-1/2 transform -translate-x-1/2 flex gap-4">
        {[0, 1, 2].map((idx) => (
          <div
            key={idx}
            className="flex flex-col items-center gap-2"
            style={{ filter: "drop-shadow(0 0 8px rgba(0, 255, 0, 0.5))" }}
          >
            <div className="text-xs text-green-400">POWER</div>
            <div className="energy-bar h-16 w-4 bg-green-500 rounded" style={{ scaleX: 0, transformOrigin: "left" }} />
          </div>
        ))}
      </div>

      {/* Particles */}
      <div className="absolute inset-0">
        {[...Array(12)].map((_, idx) => (
          <div
            key={idx}
            className="particle absolute w-2 h-2 bg-green-400 rounded-full"
            style={{
              left: `${Math.random() * 100}%`,
              top: `${Math.random() * 100}%`,
              filter: "drop-shadow(0 0 4px rgba(0, 255, 0, 0.6))",
            }}
          />
        ))}
      </div>

      {/* Mission Name */}
      <div className="absolute top-20 text-center">
        <p className="text-sm text-gray-400">LAUNCHING MISSION</p>
        <p className="text-2xl font-bold neon-green-glow">{missionName}</p>
      </div>
    </div>
  )
}
