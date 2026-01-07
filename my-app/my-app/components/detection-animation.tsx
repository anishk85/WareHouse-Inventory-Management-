"use client"

import { useEffect, useRef } from "react"
import gsap from "gsap"

interface DetectionAnimationProps {
  isDetecting: boolean
}

export function DetectionAnimation({ isDetecting }: DetectionAnimationProps) {
  const containerRef = useRef<HTMLDivElement>(null)
  const timelineRef = useRef<gsap.core.Timeline | null>(null)

  useEffect(() => {
    if (!containerRef.current) return

    if (isDetecting) {
      // Kill existing timeline
      if (timelineRef.current) timelineRef.current.kill()

      timelineRef.current = gsap.timeline()

      // Create ripples
      const ripples = containerRef.current.querySelectorAll(".ripple")
      ripples.forEach((ripple, index) => {
        timelineRef.current?.fromTo(
          ripple,
          { scale: 0, opacity: 1 },
          {
            scale: 3,
            opacity: 0,
            duration: 1.5,
            ease: "power2.out",
            delay: index * 0.15,
          },
        )
      })

      // Success checkmark animation
      const checkmark = containerRef.current.querySelector(".checkmark")
      if (checkmark) {
        const checkTimeline = gsap.timeline()
        checkTimeline
          .from(checkmark, {
            scale: 0,
            rotation: -45,
            duration: 0.5,
            ease: "back.out(1.7)",
          })
          .to(checkmark, {
            scale: 1.2,
            duration: 0.2,
            yoyo: true,
            repeat: 1,
          })

        timelineRef.current?.add(checkTimeline, 0.3)
      }
    }
  }, [isDetecting])

  if (!isDetecting) return null

  return (
    <div ref={containerRef} className="absolute inset-0 flex items-center justify-center pointer-events-none">
      {/* Ripples */}
      {[0, 1, 2].map((i) => (
        <div key={i} className="ripple absolute w-16 h-16 rounded-full border-2" style={{ borderColor: "#00ff00" }} />
      ))}

      {/* Checkmark */}
      <svg className="checkmark absolute w-20 h-20" fill="none" stroke="#00ff00" strokeWidth="2" viewBox="0 0 24 24">
        <polyline points="20 6 9 17 4 12" />
      </svg>
    </div>
  )
}
