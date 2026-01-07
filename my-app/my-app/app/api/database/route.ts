
import { NextResponse } from 'next/server'
import { Client } from '@neondatabase/serverless'

// Server-side route to proxy simple queries to Neon DB.
// Expects environment variable NEON_DB_URL to be set in the runtime environment.

const connectionString = process.env.NEON_DB_URL || process.env.DATABASE_URL

if (!connectionString) {
	// Note: Next will load this file at build time; returning at runtime from the handler.
}

async function queryNeon(sql: string, params: any[] = []) {
	if (!connectionString) throw new Error('NEON_DB_URL not configured')
		const client = new Client({ connectionString })
	try {
		const res = await client.query(sql, params)
		return res.rows
	} finally {
		// client doesn't need explicit close in serverless lib, but close if available
		try {
			// @ts-ignore
			if (client.end) await client.end()
		} catch (_) {}
	}
}

export async function GET(req: Request) {
	try {
		const url = new URL(req.url)
		const resource = url.searchParams.get('resource') || 'missions'

		let rows = []
		switch (resource) {
			case 'missions':
				// Example missions table: id, name, started_at, finished_at, status
				try {
					rows = await queryNeon('SELECT id, name, started_at, finished_at, status FROM missions ORDER BY started_at DESC LIMIT 50')
				} catch (e) {
					rows = []
				}
				break
			case 'waypoints':
				// Example waypoints table: id, mission_id, seq, x, y, z, orientation
				try {
					rows = await queryNeon('SELECT id, mission_id, seq, x, y, z, orientation FROM waypoints ORDER BY mission_id, seq')
				} catch (e) {
					rows = []
				}
				break
			case 'qr':
			case 'qr_detections':
				// Example qr_detections table: id, mission_id, detected_at, qr_data, rack_id, shelf_id
				try {
					rows = await queryNeon('SELECT id, mission_id, detected_at, qr_data, rack_id, shelf_id FROM qr_detections ORDER BY detected_at DESC LIMIT 200')
				} catch (e) {
					rows = []
				}
				break
			default:
				return NextResponse.json({ error: 'Unknown resource' }, { status: 400 })
		}

		return NextResponse.json({ ok: true, resource, rows })
	} catch (err: any) {
		return NextResponse.json({ ok: false, error: err.message || String(err) }, { status: 500 })
	}
}
