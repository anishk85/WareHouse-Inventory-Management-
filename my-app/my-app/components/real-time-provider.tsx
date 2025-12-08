"use client"

import { createContext, useContext, type ReactNode } from "react"
import { useWebSocket } from "@/hooks/use-websocket"

interface RealTimeContextType {
  isConnected: boolean
  subscribe: (messageType: string, handler: (payload: unknown) => void) => () => void
  send: (type: string, payload: unknown) => void
}

const RealTimeContext = createContext<RealTimeContextType | undefined>(undefined)

interface RealTimeProviderProps {
  children: ReactNode
}

export function RealTimeProvider({ children }: RealTimeProviderProps) {
  const { isConnected, subscribe, send } = useWebSocket({
    autoConnect: true,
    onConnect: () => {
      console.log("[v0] Real-time connection established")
    },
    onDisconnect: () => {
      console.log("[v0] Real-time connection lost")
    },
  })

  return <RealTimeContext.Provider value={{ isConnected, subscribe, send }}>{children}</RealTimeContext.Provider>
}

export function useRealTime() {
  const context = useContext(RealTimeContext)
  if (!context) {
    throw new Error("useRealTime must be used within RealTimeProvider")
  }
  return context
}
