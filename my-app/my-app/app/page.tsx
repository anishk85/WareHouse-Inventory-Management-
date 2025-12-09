"use client"

import { useState } from "react"
import { RealTimeProvider } from "@/components/real-time-provider"
import Navigation from "@/components/navigation"
import Dashboard from "@/components/pages/dashboard"
import ROS2LaunchControl from "@/components/pages/ros2-launch-control"
import NavigationPage from "@/components/pages/navigation-page"
import WaypointCreator from "@/components/pages/waypoint-creator"
import InventoryQR from "@/components/pages/inventory-qr"
import Database from "@/components/pages/database"
import SystemMonitor from "@/components/pages/system-monitor"
import EnhancedDashboard from "@/components/pages/enhanced-dashboard"

type PageType =
  | "dashboard"
  | "launch"
  | "navigation"
  | "waypoint"
  | "inventory"
  | "database"
  | "monitor"
  | "control"

function PageContent() {
  const [currentPage, setCurrentPage] = useState<PageType>("control")

  const renderPage = () => {
    switch (currentPage) {
      case "dashboard":
        return <Dashboard />
      case "control":
        return <EnhancedDashboard />
      case "launch":
        return <ROS2LaunchControl />
      case "navigation":
        return <NavigationPage />
      case "waypoint":
        return <WaypointCreator />
      case "inventory":
        return <InventoryQR />
      case "database":
        return <Database />
      case "monitor":
        return <SystemMonitor />
      default:
        return <EnhancedDashboard />
    }
  }

  return (
    <div className="flex h-screen bg-background">
      <Navigation currentPage={currentPage} onPageChange={setCurrentPage} />
      <main className="flex-1 overflow-auto">{renderPage()}</main>
    </div>
  )
}

export default function Home() {
  return (
    <RealTimeProvider>
      <PageContent />
    </RealTimeProvider>
  )
}
