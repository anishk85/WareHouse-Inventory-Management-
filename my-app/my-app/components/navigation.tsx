"use client"

import { useState } from "react"
import { Menu, X, Radio } from "lucide-react"
import { Button } from "@/components/ui/button"

type PageType = "dashboard" | "launch" | "navigation" | "waypoint" | "inventory" | "database" | "monitor" | "control"

interface NavigationProps {
  currentPage: PageType
  onPageChange: (page: PageType) => void
}

const navItems: { id: PageType; label: string; icon: string }[] = [
  { id: "control", label: "Control Panel", icon: "🎮" },
  { id: "dashboard", label: "Dashboard", icon: "📡" },
  { id: "launch", label: "Launch Control", icon: "🚀" },
  { id: "navigation", label: "Navigation", icon: "🗺️" },
  { id: "waypoint", label: "Waypoint Creator", icon: "🎯" },
  { id: "inventory", label: "Inventory & QR", icon: "📦" },
  { id: "database", label: "Database", icon: "💾" },
  { id: "monitor", label: "System Monitor", icon: "⚙️" },
]

export default function Navigation({ currentPage, onPageChange }: NavigationProps) {
  const [isOpen, setIsOpen] = useState(true)

  return (
    <nav
      className={`border-r transition-all duration-300 flex flex-col ${isOpen ? "w-64" : "w-20"}`}
      style={{
        backgroundColor: "#0f1419",
        borderColor: "#00ff00",
        boxShadow: isOpen ? "inset -20px 0 20px rgba(0, 255, 0, 0.05)" : "none",
      }}
    >
      <div
        className="p-4 border-b flex items-center justify-between"
        style={{
          borderColor: "#00ff00",
          boxShadow: "0 0 10px rgba(0, 255, 0, 0.1)",
        }}
      >
        {isOpen && (
          <div className="flex items-center gap-2">
            <Radio size={20} className="neon-green animate-spin" style={{ animationDuration: "4s" }} />
            <h1 className="text-lg font-bold neon-green-glow">ROVER</h1>
          </div>
        )}
        <Button
          variant="ghost"
          size="sm"
          onClick={() => setIsOpen(!isOpen)}
          className="text-green-400 hover:text-green-300 hover:bg-green-500/10"
        >
          {isOpen ? <X size={20} /> : <Menu size={20} />}
        </Button>
      </div>

      {/* Navigation Items */}
      <div className="flex-1 overflow-y-auto py-4 space-y-2 px-2">
        {navItems.map((item) => (
          <button
            key={item.id}
            onClick={() => onPageChange(item.id)}
            className={`w-full px-4 py-3 rounded-lg transition-all duration-200 text-left flex items-center gap-3 ${
              currentPage === item.id ? "neon-border-green-strong" : "neon-border-green"
            }`}
            style={{
              backgroundColor: currentPage === item.id ? "rgba(0, 255, 0, 0.1)" : "rgba(0, 255, 0, 0.02)",
              color: currentPage === item.id ? "#00ff00" : "#ffffff",
              textShadow: currentPage === item.id ? "0 0 8px rgba(0, 255, 0, 0.4)" : "none",
            }}
          >
            <span className="text-xl">{item.icon}</span>
            {isOpen && <span className="text-sm font-medium">{item.label}</span>}
          </button>
        ))}
      </div>

      <div
        className="border-t p-4 text-xs"
        style={{
          borderColor: "#00ff00",
          boxShadow: "inset 0 10px 15px rgba(0, 255, 0, 0.05)",
        }}
      >
        {isOpen ? (
          <div className="space-y-2">
            <p className="text-gray-400">Status</p>
            <div className="flex items-center gap-2">
              <div className="w-2 h-2 rounded-full neon-green blink-indicator"></div>
              <p className="neon-green">Connected</p>
            </div>
          </div>
        ) : (
          <div className="text-center neon-green blink-indicator">●</div>
        )}
      </div>
    </nav>
  )
}
