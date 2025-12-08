"use client"

import { useEffect, useState, useCallback, useRef } from "react"
import { getWebSocketService } from "@/lib/websocket-service"

interface UseWebSocketOptions {
  autoConnect?: boolean
  onConnect?: () => void
  onDisconnect?: () => void
}

export function useWebSocket(options: UseWebSocketOptions = {}) {
  const { autoConnect = true, onConnect, onDisconnect } = options
  const [isConnected, setIsConnected] = useState(false)
  const serviceRef = useRef(getWebSocketService())

  useEffect(() => {
    const service = serviceRef.current

    if (autoConnect && !service.isConnected()) {
      service.connect().catch((error) => {
        console.error("[v0] Failed to auto-connect WebSocket:", error)
      })
    }

    const unsubscribeConnection = service.onConnectionChange((connected) => {
      setIsConnected(connected)
      if (connected) {
        onConnect?.()
      } else {
        onDisconnect?.()
      }
    })

    return () => {
      unsubscribeConnection()
    }
  }, [autoConnect, onConnect, onDisconnect])

  const subscribe = useCallback((messageType: string, handler: (payload: unknown) => void) => {
    return serviceRef.current.subscribe(messageType, handler)
  }, [])

  const send = useCallback((type: string, payload: unknown) => {
    serviceRef.current.send(type, payload)
  }, [])

  return {
    isConnected,
    subscribe,
    send,
    service: serviceRef.current,
  }
}
