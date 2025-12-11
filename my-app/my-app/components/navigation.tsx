"use client"

import { useState } from "react"
import { Menu, X, Radio } from "lucide-react"
import { Button } from "@/components/ui/button"
import { ThemeToggle } from "@/components/theme-toggle"

type PageType = "dashboard" | "launch" | "waypoint" | "database" | "control"

interface NavigationProps {
  currentPage: PageType
  onPageChange: (page: PageType) => void
}

const navItems: { id: PageType; label: string; icon: string }[] = [
  { id: "control", label: "Control Panel", icon: "🎮" },
  { id: "dashboard", label: "Dashboard", icon: "📡" },
  { id: "launch", label: "Launch Control", icon: "🚀" },
  { id: "waypoint", label: "Waypoint Creator", icon: "🎯" },
  { id: "database", label: "Database", icon: "💾" },
]

export default function Navigation({ currentPage, onPageChange }: NavigationProps) {
  const [isOpen, setIsOpen] = useState(true)
  const [isMobileOpen, setIsMobileOpen] = useState(false)

  return (
    <>
      {/* Mobile Menu Button */}
      <Button
        variant="ghost"
        size="icon"
        onClick={() => setIsMobileOpen(!isMobileOpen)}
        className="fixed top-4 left-4 z-50 md:hidden bg-slate-900/90 backdrop-blur-sm border border-cyan-500/50"
      >
        {isMobileOpen ? <X size={20} className="text-cyan-400" /> : <Menu size={20} className="text-cyan-400" />}
      </Button>

      {/* Theme Toggle - Top Right on Mobile */}
      <div className="fixed top-4 right-4 z-50 md:hidden">
        <ThemeToggle />
      </div>

      {/* Navigation Sidebar */}
      <nav
        className={`
          fixed md:relative inset-y-0 left-0 z-40
          border-r transition-all duration-300 flex flex-col
          bg-slate-900 dark:bg-slate-950 border-cyan-500/30
          ${isOpen ? "w-64" : "w-20"}
          ${isMobileOpen ? "translate-x-0" : "-translate-x-full md:translate-x-0"}
        `}
      >
        <div className="p-4 border-b border-cyan-500/30 flex items-center justify-between backdrop-blur-sm">
          {isOpen && (
            <div className="flex items-center gap-2">
              <Radio size={20} className="text-cyan-400 animate-spin" style={{ animationDuration: "4s" }} />
              <h1 className="text-lg font-bold text-cyan-400">ROVER</h1>
            </div>
          )}
          <div className="flex items-center gap-2">
            <div className="hidden md:block">
              <ThemeToggle />
            </div>
            <Button
              variant="ghost"
              size="sm"
              onClick={() => setIsOpen(!isOpen)}
              className="hidden md:flex text-cyan-400 hover:text-cyan-300 hover:bg-cyan-500/10"
            >
              {isOpen ? <X size={20} /> : <Menu size={20} />}
            </Button>
          </div>
        </div>

        {/* Navigation Items */}
        <div className="flex-1 overflow-y-auto py-4 space-y-2 px-2">
          {navItems.map((item) => (
            <button
              key={item.id}
              onClick={() => {
                onPageChange(item.id)
                setIsMobileOpen(false) // Close mobile menu on selection
              }}
              className={`
                w-full px-4 py-3 rounded-lg transition-all duration-200 text-left flex items-center gap-3
                hover:bg-cyan-500/10
                ${currentPage === item.id 
                  ? "bg-cyan-500/20 border border-cyan-500/50 text-cyan-400" 
                  : "border border-transparent text-slate-300 dark:text-slate-400"
                }
              `}
            >
              <span className="text-xl">{item.icon}</span>
              {isOpen && <span className="text-sm font-medium">{item.label}</span>}
            </button>
          ))}
        </div>

        <div className="border-t border-cyan-500/30 p-4 text-xs backdrop-blur-sm">
          {isOpen ? (
            <div className="space-y-2">
              <p className="text-slate-400">Status</p>
              <div className="flex items-center gap-2">
                <div className="w-2 h-2 rounded-full bg-cyan-400 animate-pulse"></div>
                <p className="text-cyan-400">Connected</p>
              </div>
            </div>
          ) : (
            <div className="text-center text-cyan-400 animate-pulse">●</div>
          )}
        </div>
      </nav>

      {/* Mobile Overlay */}
      {isMobileOpen && (
        <div
          className="fixed inset-0 bg-black/50 z-30 md:hidden backdrop-blur-sm"
          onClick={() => setIsMobileOpen(false)}
        />
      )}
    </>
  )
}
