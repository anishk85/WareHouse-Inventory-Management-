import { NextResponse } from 'next/server'
import { Client } from '@neondatabase/serverless'

const connectionString = process.env.NEON_DB_URL || process.env.DATABASE_URL

async function initializeDatabase() {
  if (!connectionString) {
    throw new Error('DATABASE_URL or NEON_DB_URL not configured')
  }

  const client = new Client({ connectionString })

  try {
    // Create qr_detections table if it doesn't exist
    await client.query(`
      CREATE TABLE IF NOT EXISTS qr_detections (
        id SERIAL PRIMARY KEY,
        mission_id VARCHAR(255),
        rack_id VARCHAR(50),
        shelf_id VARCHAR(50),
        item_code VARCHAR(100),
        confidence DECIMAL(5, 2),
        metadata JSONB,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
      );
      
      CREATE INDEX IF NOT EXISTS idx_mission_id ON qr_detections(mission_id);
      CREATE INDEX IF NOT EXISTS idx_rack_id ON qr_detections(rack_id);
      CREATE INDEX IF NOT EXISTS idx_created_at ON qr_detections(created_at);
    `)

    // Create missions table if it doesn't exist
    await client.query(`
      CREATE TABLE IF NOT EXISTS missions (
        id SERIAL PRIMARY KEY,
        name VARCHAR(255),
        started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        finished_at TIMESTAMP,
        status VARCHAR(50),
        total_detections INT DEFAULT 0
      );
      
      CREATE INDEX IF NOT EXISTS idx_missions_started_at ON missions(started_at);
    `)

    // Create waypoints table if it doesn't exist
    await client.query(`
      CREATE TABLE IF NOT EXISTS waypoints (
        id SERIAL PRIMARY KEY,
        mission_id INT REFERENCES missions(id) ON DELETE CASCADE,
        seq INT,
        x DECIMAL(10, 4),
        y DECIMAL(10, 4),
        z DECIMAL(10, 4),
        orientation VARCHAR(50),
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
      );
      
      CREATE INDEX IF NOT EXISTS idx_waypoints_mission_id ON waypoints(mission_id);
    `)

    return { success: true, message: 'Database tables initialized successfully' }
  } finally {
    try {
      // @ts-ignore
      if (client.end) await client.end()
    } catch (_) {}
  }
}

export async function GET(req: Request) {
  try {
    const result = await initializeDatabase()
    return NextResponse.json(result)
  } catch (err: any) {
    return NextResponse.json(
      {
        error: 'Failed to initialize database',
        details: err.message || String(err)
      },
      { status: 500 }
    )
  }
}
