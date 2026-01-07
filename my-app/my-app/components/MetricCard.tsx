"use client"

import { Card, CardContent } from "@/components/ui/card"

interface MetricCardProps {
  title: string
  value: string
  unit: string
  status: string
}

export default function MetricCard({ title, value, unit, status }: MetricCardProps) {
  return (
    <Card className="neon-border-cyan">
      <CardContent className="p-4">
        <p className="text-sm text-muted-foreground">{title}</p>
        <div className="flex items-end gap-2 mt-2">
          <p className={`text-3xl font-bold ${status}`}>{value}</p>
          <p className="text-sm text-muted-foreground mb-1">{unit}</p>
        </div>
      </CardContent>
    </Card>
  )
}
