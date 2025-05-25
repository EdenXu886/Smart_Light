from flask import Flask, request, jsonify, render_template
from datetime import datetime, timedelta
import sqlite3
import json
from collections import defaultdict

app = Flask(__name__)

# 数据库初始化
def init_db():
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS sensor_data
                 (timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                  mac TEXT,
                  light_state INTEGER,
                  motion_detected BOOLEAN,
                  motion_count INTEGER,
                  lux_value REAL,
                  left_neighbor_connected BOOLEAN,
                  right_neighbor_connected BOOLEAN,
                  emergency_mode BOOLEAN,
                  emergency_event BOOLEAN,
                  emergency_timestamp INTEGER,
                  power_usage REAL,
                  motion_count_today INTEGER,
                  motion_count_total INTEGER)''')
    
    c.execute('''CREATE TABLE IF NOT EXISTS emergency_events
                 (timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                  mac TEXT,
                  duration INTEGER,
                  triggered_by TEXT,
                  count INTEGER DEFAULT 1)''')
    conn.commit()
    conn.close()

init_db()

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/update', methods=['POST'])
def update():
    data = request.json
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    
    # 插入传感器数据
    c.execute('''INSERT INTO sensor_data 
                 (mac, light_state, motion_detected, motion_count, lux_value,
                  left_neighbor_connected, right_neighbor_connected, emergency_mode,
                  emergency_event, emergency_timestamp, power_usage,
                  motion_count_today, motion_count_total)
                 VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''',
              (data['mac'], data['light_state'], data['motion_detected'],
               data['motion_count'], data['lux_value'], data['left_neighbor_connected'],
               data['right_neighbor_connected'], data['emergency_mode'],
               data['emergency_event'], data['emergency_timestamp'],
               data['power_usage'], data['motion_count_today'],
               data['motion_count_total']))
    
    # emergency事件去重：同一节点3秒内只记录一次
    if data.get('emergency_event'):
        today = datetime.now().strftime('%Y-%m-%d')
        c.execute('''SELECT count FROM emergency_events WHERE mac=? AND date(timestamp)=?''', (data['mac'], today))
        row = c.fetchone()
        if row:
            c.execute('''UPDATE emergency_events SET count = count + 1, timestamp=CURRENT_TIMESTAMP WHERE mac=? AND date(timestamp)=?''', (data['mac'], today))
        else:
            c.execute('''INSERT INTO emergency_events (mac, duration, triggered_by, count) VALUES (?, ?, ?, 1)''', (data['mac'], 3000, 'button'))
    
    conn.commit()
    conn.close()
    return jsonify({"status": "success"})

@app.route('/latest/<mac>')
def get_latest_by_mac(mac):
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    c.execute('''SELECT * FROM sensor_data WHERE mac=? ORDER BY timestamp DESC LIMIT 1''', (mac,))
    row = c.fetchone()
    conn.close()
    columns = ['timestamp', 'mac', 'light_state', 'motion_detected', 'motion_count',
               'lux_value', 'left_neighbor_connected', 'right_neighbor_connected',
               'emergency_mode', 'emergency_event', 'emergency_timestamp',
               'power_usage', 'motion_count_today', 'motion_count_total']
    if row:
        return jsonify(dict(zip(columns, row)))
    else:
        return jsonify({})

@app.route('/emergency/stats')
def get_emergency_stats():
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    
    # 获取最近24小时的emergency事件统计
    c.execute('''SELECT mac, COUNT(*) as count, 
                 strftime('%H', timestamp) as hour
                 FROM emergency_events 
                 WHERE timestamp > datetime('now', '-24 hours')
                 GROUP BY mac, hour
                 ORDER BY hour''')
    
    hourly_stats = defaultdict(lambda: defaultdict(int))
    for row in c.fetchall():
        mac, count, hour = row
        hourly_stats[mac][int(hour)] = count
    
    # 获取每个节点的总emergency事件数
    c.execute('''SELECT mac, COUNT(*) as total_count
                 FROM emergency_events
                 GROUP BY mac''')
    
    total_stats = {row[0]: row[1] for row in c.fetchall()}
    
    # 获取最后一次emergency事件时间
    c.execute('''SELECT timestamp
                 FROM emergency_events
                 ORDER BY timestamp DESC
                 LIMIT 1''')
    
    last_event = c.fetchone()
    last_event_time = last_event[0] if last_event else None
    
    conn.close()
    
    return jsonify({
        'hourly_stats': dict(hourly_stats),
        'total_stats': total_stats,
        'last_event': last_event_time
    })

@app.route('/emergency/today')
def get_emergency_today():
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    # 查询今天所有emergency事件
    c.execute('''SELECT mac, timestamp, count FROM emergency_events WHERE date(timestamp) = date('now', 'localtime')''')
    rows = c.fetchall()
    events = [{'mac': row[0], 'timestamp': row[1], 'count': row[2]} for row in rows]
    by_node = {row[0]: row[2] for row in rows}
    total = sum(row[2] for row in rows)
    result = {
        'total': total,
        'by_node': by_node,
        'events': events
    }
    conn.close()
    return jsonify(result)

@app.route('/nodes')
def get_nodes():
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    c.execute("""SELECT DISTINCT mac FROM sensor_data WHERE timestamp > datetime('now', '-1 minute')""")
    nodes = [row[0] for row in c.fetchall()]
    conn.close()
    return jsonify(nodes)

@app.route('/history/daily')
def history_daily():
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    # 查询最近24小时每节点的历史数据
    c.execute('''SELECT mac, timestamp, light_state, motion_count_today, power_usage
                 FROM sensor_data
                 WHERE timestamp > datetime('now', '-1 day')
                 ORDER BY mac, timestamp''')
    rows = c.fetchall()
    conn.close()
    from collections import defaultdict
    result = defaultdict(lambda: {'timestamps': [], 'light_state': [], 'motion_count_today': [], 'power_usage': []})
    for mac, ts, ls, mc, pu in rows:
        result[mac]['timestamps'].append(ts)
        result[mac]['light_state'].append(ls)
        result[mac]['motion_count_today'].append(mc)
        result[mac]['power_usage'].append(pu)
    return jsonify(result)

@app.route('/stats/peak')
def stats_peak():
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    # 统计每节点24小时内每小时的motion_count_today峰值区间
    c.execute('''SELECT mac, strftime('%H', timestamp) as hour, MAX(motion_count_today) FROM sensor_data WHERE timestamp > datetime('now', '-1 day') GROUP BY mac, hour''')
    rows = c.fetchall()
    from collections import defaultdict
    result = defaultdict(lambda: {'hourly': [0]*24, 'peak_start': '-', 'peak_end': '-'})
    for mac, hour, peak in rows:
        if hour is not None:
            result[mac]['hourly'][int(hour)] = peak if peak is not None else 0
    # 计算峰值3小时窗口
    for mac in result:
        hourly = result[mac]['hourly']
        max_sum = 0
        peak_start = 0
        for i in range(22):
            s = sum(hourly[i:i+3])
            if s > max_sum:
                max_sum = s
                peak_start = i
        result[mac]['peak_start'] = peak_start
        result[mac]['peak_end'] = peak_start+2
    conn.close()
    return jsonify(result)

@app.route('/stats/daily')
def stats_daily():
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    # 统计每节点每天的平均lux和总能耗
    c.execute('''SELECT mac, date(timestamp), AVG(lux_value), SUM(power_usage) FROM sensor_data GROUP BY mac, date(timestamp)''')
    rows = c.fetchall()
    from collections import defaultdict
    result = defaultdict(lambda: {'days': [], 'lux': [], 'energy': []})
    for mac, day, avg_lux, total_energy in rows:
        result[mac]['days'].append(day)
        result[mac]['lux'].append(avg_lux)
        result[mac]['energy'].append(total_energy)
    conn.close()
    return jsonify(result)

@app.route('/power/usage')
def power_usage():
    conn = sqlite3.connect('streetlight.db')
    c = conn.cursor()
    # 每节点每小时总和
    c.execute('''SELECT mac, strftime('%Y-%m-%d %H:00:00', timestamp) as hour, SUM(power_usage) FROM sensor_data GROUP BY mac, hour ORDER BY mac, hour''')
    rows = c.fetchall()
    from collections import defaultdict
    result = defaultdict(lambda: {'hours': [], 'energy': []})
    for mac, hour, total_energy in rows:
        result[mac]['hours'].append(hour)
        result[mac]['energy'].append(total_energy)
    conn.close()
    return jsonify(result)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5050, debug=True) 