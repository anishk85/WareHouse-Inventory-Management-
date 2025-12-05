#!/usr/bin/env python3
from flask import Flask, render_template_string
import sqlite3
from datetime import datetime

app = Flask(__name__)
DB_PATH = '/tmp/warehouse_inventory.db'

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Warehouse Inventory Dashboard</title>
    <meta http-equiv="refresh" content="3">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 20px;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
            background: white;
            border-radius: 10px;
            box-shadow: 0 10px 40px rgba(0,0,0,0.3);
            overflow: hidden;
        }
        header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            text-align: center;
        }
        h1 { font-size: 2.5em; margin-bottom: 10px; }
        .refresh-info { 
            background: rgba(255,255,255,0.2);
            padding: 8px 15px;
            border-radius: 20px;
            display: inline-block;
            margin-top: 10px;
        }
        .stats {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            padding: 30px;
            background: #f8f9fa;
        }
        .stat-card {
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            text-align: center;
            border-left: 4px solid #667eea;
        }
        .stat-number {
            font-size: 2.5em;
            font-weight: bold;
            color: #667eea;
        }
        .stat-label {
            color: #666;
            margin-top: 5px;
            text-transform: uppercase;
            font-size: 0.85em;
            letter-spacing: 1px;
        }
        .section {
            padding: 30px;
        }
        h2 {
            color: #333;
            margin-bottom: 20px;
            padding-bottom: 10px;
            border-bottom: 3px solid #667eea;
        }
        table {
            width: 100%;
            border-collapse: collapse;
            background: white;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        th {
            background: #667eea;
            color: white;
            padding: 15px;
            text-align: left;
            font-weight: 600;
            text-transform: uppercase;
            font-size: 0.85em;
            letter-spacing: 1px;
        }
        td {
            padding: 12px 15px;
            border-bottom: 1px solid #e0e0e0;
        }
        tr:hover td {
            background: #f5f5f5;
        }
        .status-active { 
            background: #4caf50;
            color: white;
            padding: 5px 12px;
            border-radius: 20px;
            font-size: 0.85em;
        }
        .status-complete {
            background: #2196f3;
            color: white;
            padding: 5px 12px;
            border-radius: 20px;
            font-size: 0.85em;
        }
        .confidence {
            font-weight: bold;
        }
        .confidence-high { color: #4caf50; }
        .confidence-medium { color: #ff9800; }
        .confidence-low { color: #f44336; }
        .empty-state {
            text-align: center;
            padding: 40px;
            color: #999;
            font-style: italic;
        }
        .timestamp {
            color: #666;
            font-size: 0.9em;
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>🏭 Warehouse Inventory System</h1>
            <div class="refresh-info">
                ⟳ Auto-refresh every 3 seconds | 🕐 Last update: {{current_time}}
            </div>
        </header>
        
        <div class="stats">
            <div class="stat-card">
                <div class="stat-number">{{stats.total_missions}}</div>
                <div class="stat-label">Total Missions</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">{{stats.active_missions}}</div>
                <div class="stat-label">Active Missions</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">{{stats.total_detections}}</div>
                <div class="stat-label">Total Detections</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">{{stats.unique_racks}}</div>
                <div class="stat-label">Unique Racks</div>
            </div>
        </div>
        
        <div class="section">
            <h2>🎯 Active Missions</h2>
            {% if missions %}
            <table>
                <tr>
                    <th>Mission ID</th>
                    <th>Start Time</th>
                    <th>End Time</th>
                    <th>Status</th>
                    <th>Detections</th>
                </tr>
                {% for mission in missions %}
                <tr>
                    <td><strong>{{mission[0]}}</strong></td>
                    <td class="timestamp">{{mission[1]}}</td>
                    <td class="timestamp">{{mission[2] or 'In Progress'}}</td>
                    <td>
                        <span class="{% if mission[3] == 'in_progress' %}status-active{% else %}status-complete{% endif %}">
                            {{mission[3]}}
                        </span>
                    </td>
                    <td><strong>{{mission[4]}}</strong></td>
                </tr>
                {% endfor %}
            </table>
            {% else %}
            <div class="empty-state">No missions recorded yet. Start scanning to see data!</div>
            {% endif %}
        </div>
        
        <div class="section">
            <h2>📦 Recent Detections (Last 30)</h2>
            {% if detections %}
            <table>
                <tr>
                    <th>Timestamp</th>
                    <th>Mission</th>
                    <th>Rack ID</th>
                    <th>Shelf ID</th>
                    <th>Item Code</th>
                    <th>Confidence</th>
                </tr>
                {% for det in detections %}
                <tr>
                    <td class="timestamp">{{det[5]}}</td>
                    <td>{{det[1]}}</td>
                    <td><strong>{{det[2]}}</strong></td>
                    <td>{{det[3]}}</td>
                    <td><strong>{{det[4]}}</strong></td>
                    <td>
                        <span class="confidence {% if det[6] >= 0.9 %}confidence-high{% elif det[6] >= 0.7 %}confidence-medium{% else %}confidence-low{% endif %}">
                            {{(det[6] * 100)|round(1)}}%
                        </span>
                    </td>
                </tr>
                {% endfor %}
            </table>
            {% else %}
            <div class="empty-state">No detections yet. Robot will populate this as it scans!</div>
            {% endif %}
        </div>
        
        <div class="section">
            <h2>📊 Detection Summary by Rack</h2>
            {% if rack_summary %}
            <table>
                <tr>
                    <th>Rack ID</th>
                    <th>Total Detections</th>
                    <th>Unique Items</th>
                    <th>Avg Confidence</th>
                </tr>
                {% for rack in rack_summary %}
                <tr>
                    <td><strong>{{rack[0]}}</strong></td>
                    <td>{{rack[1]}}</td>
                    <td>{{rack[2]}}</td>
                    <td>
                        <span class="confidence {% if rack[3] >= 0.9 %}confidence-high{% elif rack[3] >= 0.7 %}confidence-medium{% else %}confidence-low{% endif %}">
                            {{(rack[3] * 100)|round(1)}}%
                        </span>
                    </td>
                </tr>
                {% endfor %}
            </table>
            {% else %}
            <div class="empty-state">Rack summary will appear after scanning starts</div>
            {% endif %}
        </div>
    </div>
</body>
</html>
'''

@app.route('/')
def dashboard():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    # Get statistics
    stats = {
        'total_missions': cursor.execute("SELECT COUNT(*) FROM missions").fetchone()[0],
        'active_missions': cursor.execute("SELECT COUNT(*) FROM missions WHERE status='in_progress'").fetchone()[0],
        'total_detections': cursor.execute("SELECT COUNT(*) FROM detections").fetchone()[0],
        'unique_racks': cursor.execute("SELECT COUNT(DISTINCT rack_id) FROM detections").fetchone()[0]
    }
    
    # Get missions
    missions = cursor.execute("""
        SELECT mission_id, start_time, end_time, status, total_detections 
        FROM missions 
        ORDER BY start_time DESC 
        LIMIT 10
    """).fetchall()
    
    # Get recent detections
    detections = cursor.execute("""
        SELECT * FROM detections 
        ORDER BY timestamp DESC 
        LIMIT 30
    """).fetchall()
    
    # Get rack summary
    rack_summary = cursor.execute("""
        SELECT 
            rack_id,
            COUNT(*) as total_detections,
            COUNT(DISTINCT item_code) as unique_items,
            AVG(confidence) as avg_confidence
        FROM detections
        GROUP BY rack_id
        ORDER BY rack_id
    """).fetchall()
    
    conn.close()
    
    return render_template_string(
        HTML_TEMPLATE,
        current_time=datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        stats=stats,
        missions=missions,
        detections=detections,
        rack_summary=rack_summary
    )

if __name__ == '__main__':
    print(f"\n✅ Warehouse Database Dashboard Starting...")
    print(f"📂 Database: {DB_PATH}")
    print(f"🌐 Open browser: http://localhost:5000")
    print(f"⟳ Auto-refresh: Every 3 seconds\n")
    app.run(host='0.0.0.0', port=7000, debug=True)