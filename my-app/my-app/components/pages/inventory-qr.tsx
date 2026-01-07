"use client"

import { useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Package, QrCode, Search } from "lucide-react"

interface Item {
  id: string
  name: string
  sku: string
  location: string
  quantity: number
  detected: boolean
}

export default function InventoryQR() {
  const [items, setItems] = useState<Item[]>([
    { id: "1", name: "Component A", sku: "SKU-001", location: "Zone A", quantity: 45, detected: true },
    { id: "2", name: "Component B", sku: "SKU-002", location: "Zone B", quantity: 12, detected: false },
    { id: "3", name: "Component C", sku: "SKU-003", location: "Zone A", quantity: 87, detected: true },
  ])

  const [scanning, setScanning] = useState(false)
  const [searchTerm, setSearchTerm] = useState("")

  const filteredItems = items.filter(
    (item) =>
      item.name.toLowerCase().includes(searchTerm.toLowerCase()) ||
      item.sku.toLowerCase().includes(searchTerm.toLowerCase()),
  )

  const toggleDetection = (id: string) => {
    setItems(items.map((item) => (item.id === id ? { ...item, detected: !item.detected } : item)))
  }

  return (
    <div className="p-6 space-y-6 bg-background">
      <div className="flex items-center gap-3">
        <Package className="text-primary" size={32} />
        <h1 className="text-3xl font-bold neon-cyan">Inventory & QR Detection</h1>
      </div>

      {/* QR Scanner */}
      <Card className="neon-border-purple">
        <CardHeader>
          <CardTitle className="text-sm flex items-center gap-2">
            <QrCode size={16} /> QR Code Scanner
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="space-y-4">
            <div className="w-full aspect-square bg-gradient-to-br from-slate-900 to-slate-950 rounded-lg border-2 border-dashed border-primary/30 flex items-center justify-center">
              <div className="text-center">
                {scanning ? (
                  <>
                    <div className="w-48 h-48 border-2 border-primary rounded-lg mx-auto mb-4 animate-pulse" />
                    <p className="text-primary neon-glow">Scanning...</p>
                  </>
                ) : (
                  <>
                    <QrCode size={48} className="mx-auto mb-4 text-muted-foreground" />
                    <p className="text-muted-foreground">Ready to scan QR codes</p>
                  </>
                )}
              </div>
            </div>

            <Button
              onClick={() => setScanning(!scanning)}
              className={`w-full ${scanning ? "bg-red-500 hover:bg-red-600" : "bg-green-500 hover:bg-green-600"} text-white`}
            >
              {scanning ? "Stop Scanning" : "Start Scanning"}
            </Button>
          </div>
        </CardContent>
      </Card>

      {/* Inventory List */}
      <Card className="neon-border-cyan">
        <CardHeader>
          <CardTitle className="text-sm">Inventory Items</CardTitle>
          <div className="mt-4 relative">
            <Search className="absolute left-3 top-1/2 -translate-y-1/2 text-muted-foreground" size={18} />
            <input
              type="text"
              placeholder="Search by name or SKU..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="w-full pl-10 pr-4 py-2 text-sm bg-input border border-border rounded focus:outline-none focus:ring-2 focus:ring-primary"
            />
          </div>
        </CardHeader>
        <CardContent>
          <div className="overflow-x-auto">
            <table className="w-full text-sm">
              <thead>
                <tr className="border-b border-border">
                  <th className="text-left py-2 px-3 text-muted-foreground font-semibold">Name</th>
                  <th className="text-left py-2 px-3 text-muted-foreground font-semibold">SKU</th>
                  <th className="text-left py-2 px-3 text-muted-foreground font-semibold">Location</th>
                  <th className="text-left py-2 px-3 text-muted-foreground font-semibold">Qty</th>
                  <th className="text-left py-2 px-3 text-muted-foreground font-semibold">Status</th>
                </tr>
              </thead>
              <tbody>
                {filteredItems.map((item) => (
                  <tr
                    key={item.id}
                    className="border-b border-border hover:bg-muted/30 transition-colors cursor-pointer"
                    onClick={() => toggleDetection(item.id)}
                  >
                    <td className="py-3 px-3 text-cyan-400">{item.name}</td>
                    <td className="py-3 px-3 text-muted-foreground">{item.sku}</td>
                    <td className="py-3 px-3 text-purple-400">{item.location}</td>
                    <td className="py-3 px-3 font-semibold">{item.quantity}</td>
                    <td className="py-3 px-3">
                      <span
                        className={`px-2 py-1 rounded text-xs font-semibold ${
                          item.detected
                            ? "bg-green-500/20 text-green-400 neon-green"
                            : "bg-yellow-500/20 text-yellow-400"
                        }`}
                      >
                        {item.detected ? "Detected" : "Pending"}
                      </span>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
