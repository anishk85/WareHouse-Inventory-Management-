"use client"

import { useState } from "react"
import { RealTimeProvider } from "@/components/real-time-provider"
import Navigation from "@/components/navigation"
import Dashboard from "@/components/pages/dashboard"
import ROS2LaunchControl from "@/components/pages/ros2-launch-control"
import Database from "@/components/pages/database"
import EnhancedDashboard from "@/components/pages/enhanced-dashboard"

type PageType =
  | "dashboard"
  | "launch"
  | "database"
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
      case "database":
        return <Database />
      default:
        return <EnhancedDashboard />
    }
  }

  return (
    <div className="flex h-screen bg-background overflow-hidden">
      <Navigation currentPage={currentPage} onPageChange={setCurrentPage} />
      <main className="flex-1 overflow-auto w-full">{renderPage()}</main>
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
