"use client"

import { useRealTime } from "@/components/real-time-provider"
import { Wifi, WifiOff } from "lucide-react"

export function ConnectionStatus() {
  const { isConnected } = useRealTime()

  return (
    <div className="flex items-center gap-2 px-3 py-1 text-xs font-semibold rounded-full">
      {isConnected ? (
        <>
          <Wifi size={14} className="neon-green" />
          <span className="text-green-400 neon-green">Connected</span>
        </>
      ) : (
        <>
          <WifiOff size={14} className="text-yellow-400" />
          <span className="text-yellow-400">Reconnecting...</span>
        </>
      )}
    </div>
  )
}
