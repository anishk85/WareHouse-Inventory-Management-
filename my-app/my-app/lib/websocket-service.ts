interface WebSocketMessage {
  type: string
  payload: unknown
}

class WebSocketService {
  private ws: WebSocket | null = null
  private url: string
  private reconnectAttempts = 0
  private maxReconnectAttempts = 5
  private reconnectDelay = 3000
  private messageHandlers: Map<string, (payload: unknown) => void> = new Map()
  private connectionCallbacks: Array<(connected: boolean) => void> = []

  constructor(url: string = process.env.NEXT_PUBLIC_WS_URL || "ws://10.76.114.60:9090") {
    this.url = url
  }

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(this.url)

        this.ws.onopen = () => {
          console.log("[v0] WebSocket connected")
          this.reconnectAttempts = 0
          this.notifyConnectionChange(true)
          resolve()
        }

        this.ws.onmessage = (event) => {
          try {
            const message: WebSocketMessage = JSON.parse(event.data)
            this.handleMessage(message)
          } catch (error) {
            console.error("[v0] Failed to parse WebSocket message:", error)
          }
        }

        this.ws.onerror = (error) => {
          console.error("[v0] WebSocket error:", error)
          reject(error)
        }

        this.ws.onclose = () => {
          console.log("[v0] WebSocket disconnected")
          this.notifyConnectionChange(false)
          this.attemptReconnect()
        }
      } catch (error) {
        console.error("[v0] WebSocket connection failed:", error)
        reject(error)
      }
    })
  }

  private attemptReconnect(): void {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++
      console.log(`[v0] Attempting to reconnect (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`)
      setTimeout(() => {
        this.connect().catch(() => {
          // Reconnection failed, will try again
        })
      }, this.reconnectDelay)
    } else {
      console.error("[v0] Max reconnection attempts reached")
      this.notifyConnectionChange(false)
    }
  }

  private handleMessage(message: WebSocketMessage): void {
    const handler = this.messageHandlers.get(message.type)
    if (handler) {
      handler(message.payload)
    } else {
      console.log("[v0] Unhandled message type:", message.type)
    }
  }

  send(type: string, payload: unknown): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      // Send as action format for ROS2 bridge
      const message = { action: type, ...payload as object }
      console.log("[WS] Sending:", JSON.stringify(message))
      this.ws.send(JSON.stringify(message))
    } else {
      console.warn("[WS] Not connected, cannot send:", type, payload)
    }
  }

  subscribe(messageType: string, handler: (payload: unknown) => void): () => void {
    this.messageHandlers.set(messageType, handler)

    // Return unsubscribe function
    return () => {
      this.messageHandlers.delete(messageType)
    }
  }

  onConnectionChange(callback: (connected: boolean) => void): () => void {
    this.connectionCallbacks.push(callback)

    // Return unsubscribe function
    return () => {
      this.connectionCallbacks = this.connectionCallbacks.filter((cb) => cb !== callback)
    }
  }

  private notifyConnectionChange(connected: boolean): void {
    this.connectionCallbacks.forEach((callback) => callback(connected))
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
  }

  isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN
  }
}

// Singleton instance
let wsService: WebSocketService | null = null

export function getWebSocketService(): WebSocketService {
  if (!wsService) {
    wsService = new WebSocketService()
  }
  return wsService
}

export type { WebSocketMessage }
