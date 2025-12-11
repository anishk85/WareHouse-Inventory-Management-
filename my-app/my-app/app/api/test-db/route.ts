import { NextResponse } from 'next/server'
import { Client } from '@neondatabase/serverless'

const connectionString = process.env.NEON_DB_URL || process.env.DATABASE_URL

export async function GET(req: Request) {
  try {
    if (!connectionString) {
      return NextResponse.json(
        { error: 'DATABASE_URL or NEON_DB_URL not configured', hint: 'Add to .env.local' },
        { status: 500 }
      )
    }

    const client = new Client({ connectionString })

    try {
      // Test connection
      const result = await client.query('SELECT NOW()')
      
      return NextResponse.json({
        success: true,
        message: 'Connected to Neon database',
        timestamp: result.rows[0]?.now || new Date().toISOString()
      })
    } finally {
      try {
        // @ts-ignore
        if (client.end) await client.end()
      } catch (_) {}
    }
  } catch (err: any) {
    return NextResponse.json(
      { 
        error: 'Failed to connect to Neon database',
        details: err.message || String(err)
      },
      { status: 500 }
    )
  }
}
