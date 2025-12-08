"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from "recharts"

interface TelemetryChartProps {
  title: string
  data: "battery" | "signal" | "temperature" | "accuracy"
}

export default function TelemetryChart({ title, data: dataType }: TelemetryChartProps) {
  const [data, setData] = useState<{ time: string; value: number }[]>([])

  useEffect(() => {
    const generateData = () => {
      const now = new Date()
      const points = []

      for (let i = 30; i >= 0; i--) {
        const time = new Date(now.getTime() - i * 60000)
        let value: number

        switch (dataType) {
          case "battery":
            value = 60 + Math.sin(i / 5) * 20 + Math.random() * 10
            break
          case "signal":
            value = 75 + Math.sin(i / 5) * 15 + Math.random() * 8
            break
          case "temperature":
            value = 45 + Math.sin(i / 5) * 10 + Math.random() * 5
            break
          case "accuracy":
            value = 2 + Math.sin(i / 5) * 0.5 + Math.random() * 0.3
            break
          default:
            value = 50
        }

        points.push({
          time: time.toLocaleTimeString("en-US", { hour: "2-digit", minute: "2-digit" }),
          value: Math.max(0, Math.min(100, value)),
        })
      }

      return points
    }

    setData(generateData())

    const interval = setInterval(() => {
      setData((prev) => {
        const updated = [...prev.slice(1)]
        let newValue: number

        switch (dataType) {
          case "battery":
            newValue = Math.max(20, Math.min(100, prev[prev.length - 1].value + (Math.random() - 0.5) * 5))
            break
          case "signal":
            newValue = Math.max(50, Math.min(100, prev[prev.length - 1].value + (Math.random() - 0.5) * 5))
            break
          case "temperature":
            newValue = Math.max(30, Math.min(60, prev[prev.length - 1].value + (Math.random() - 0.5) * 2))
            break
          case "accuracy":
            newValue = Math.max(0.5, Math.min(5, prev[prev.length - 1].value + (Math.random() - 0.5) * 0.5))
            break
          default:
            newValue = 50
        }

        updated.push({
          time: new Date().toLocaleTimeString("en-US", { hour: "2-digit", minute: "2-digit" }),
          value: newValue,
        })

        return updated
      })
    }, 2000)

    return () => clearInterval(interval)
  }, [dataType])

  const getColor = () => {
    switch (dataType) {
      case "battery":
        return "rgba(34, 211, 238, 1)"
      case "signal":
        return "rgba(168, 85, 247, 1)"
      case "temperature":
        return "rgba(74, 222, 128, 1)"
      case "accuracy":
        return "rgba(251, 146, 60, 1)"
      default:
        return "rgba(34, 211, 238, 1)"
    }
  }

  return (
    <Card className="neon-border-cyan">
      <CardHeader>
        <CardTitle className="text-sm">{title}</CardTitle>
      </CardHeader>
      <CardContent>
        <ResponsiveContainer width="100%" height={200}>
          <LineChart data={data} margin={{ top: 5, right: 10, left: -20, bottom: 0 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="rgba(255, 255, 255, 0.1)" vertical={false} />
            <XAxis dataKey="time" tick={{ fontSize: 12, fill: "rgba(255, 255, 255, 0.5)" }} axisLine={false} />
            <YAxis tick={{ fontSize: 12, fill: "rgba(255, 255, 255, 0.5)" }} axisLine={false} />
            <Tooltip
              contentStyle={{
                backgroundColor: "rgba(12, 22, 44, 0.9)",
                border: "1px solid rgba(34, 211, 238, 0.5)",
                borderRadius: "4px",
              }}
              labelStyle={{ color: "rgba(255, 255, 255, 0.8)" }}
            />
            <Line
              type="monotone"
              dataKey="value"
              stroke={getColor()}
              strokeWidth={2}
              dot={false}
              isAnimationActive={false}
            />
          </LineChart>
        </ResponsiveContainer>
      </CardContent>
    </Card>
  )
}
