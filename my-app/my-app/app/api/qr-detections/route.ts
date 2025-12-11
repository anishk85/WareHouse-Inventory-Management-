import { NextRequest, NextResponse } from 'next/server';
import { Pool } from '@neondatabase/serverless';

// Initialize Neon database client
const pool = new Pool({
  connectionString: process.env.DATABASE_URL
});

// Initialize database table if needed
async function initializeTable() {
  const client = await pool.connect();
  try {
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
    `);
  } catch (error) {
    console.error('Error initializing table:', error);
  } finally {
    client.release();
  }
}

// POST: Add QR detection
export async function POST(request: NextRequest) {
  const client = await pool.connect();
  try {
    await initializeTable();
    
    const body = await request.json();
    const {
      mission_id,
      rack_id,
      shelf_id,
      item_code,
      confidence,
      metadata = {}
    } = body;

    // Validate required fields
    if (!rack_id || !shelf_id || !item_code) {
      return NextResponse.json(
        { error: 'Missing required fields: rack_id, shelf_id, item_code' },
        { status: 400 }
      );
    }

    const result = await client.query(
      `INSERT INTO qr_detections 
      (mission_id, rack_id, shelf_id, item_code, confidence, metadata)
      VALUES ($1, $2, $3, $4, $5, $6)
      RETURNING id, mission_id, rack_id, shelf_id, item_code, confidence, metadata, created_at;`,
      [mission_id || null, rack_id, shelf_id, item_code, confidence || null, JSON.stringify(metadata)]
    );

    return NextResponse.json({
      success: true,
      data: result.rows[0]
    }, { status: 201 });
  } catch (error) {
    console.error('Error inserting QR detection:', error);
    return NextResponse.json(
      { error: 'Failed to insert QR detection' },
      { status: 500 }
    );
  } finally {
    client.release();
  }
}

// GET: Retrieve QR detections with optional filters
export async function GET(request: NextRequest) {
  const client = await pool.connect();
  try {
    await initializeTable();
    
    const searchParams = request.nextUrl.searchParams;
    const mission_id = searchParams.get('mission_id');
    const rack_id = searchParams.get('rack_id');
    const limit = parseInt(searchParams.get('limit') || '100', 10);
    const offset = parseInt(searchParams.get('offset') || '0', 10);

    let query = 'SELECT * FROM qr_detections WHERE 1=1';
    const params: (string | number)[] = [];

    if (mission_id) {
      params.push(mission_id);
      query += ` AND mission_id = $${params.length}`;
    }

    if (rack_id) {
      params.push(rack_id);
      query += ` AND rack_id = $${params.length}`;
    }

    query += ` ORDER BY created_at DESC LIMIT ${limit} OFFSET ${offset}`;

    const result = await client.query(query, params);

    return NextResponse.json({
      success: true,
      data: result.rows,
      count: result.rows.length
    }, { status: 200 });
  } catch (error) {
    console.error('Error retrieving QR detections:', error);
    return NextResponse.json(
      { error: 'Failed to retrieve QR detections' },
      { status: 500 }
    );
  } finally {
    client.release();
  }
}

// DELETE: Remove a QR detection (by ID)
export async function DELETE(request: NextRequest) {
  const client = await pool.connect();
  try {
    const searchParams = request.nextUrl.searchParams;
    const id = searchParams.get('id');

    if (!id) {
      return NextResponse.json(
        { error: 'Missing required parameter: id' },
        { status: 400 }
      );
    }

    await client.query(
      `DELETE FROM qr_detections WHERE id = $1;`,
      [parseInt(id, 10)]
    );

    return NextResponse.json({
      success: true,
      message: 'QR detection deleted successfully'
    }, { status: 200 });
  } catch (error) {
    console.error('Error deleting QR detection:', error);
    return NextResponse.json(
      { error: 'Failed to delete QR detection' },
      { status: 500 }
    );
  } finally {
    client.release();
  }
}
