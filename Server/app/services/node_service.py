from app import socketio
from flask_socketio import emit, join_room, leave_room
from app.models.node import Node
from datetime import datetime, timedelta

@socketio.on('connect')
def handle_connect():
    print('Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('subscribe_node')
def handle_subscribe_node(data):
    node_id = data.get('node_id')
    if node_id:
        join_room(f'node_{node_id}')
        # 发送当前节点状态
        node = Node.query.filter_by(node_id=node_id).first()
        if node:
            emit('node_status', {
                'node_id': node_id,
                'status': {
                    'distance': node.distance,
                    'motion_count': node.motion_count,
                    'emergency_triggered': node.emergency_triggered,
                    'brightness': node.brightness,
                    'last_update': node.last_update.isoformat()
                }
            })

@socketio.on('unsubscribe_node')
def handle_unsubscribe_node(data):
    node_id = data.get('node_id')
    if node_id:
        leave_room(f'node_{node_id}')

def get_node_statistics(node_id, hours=24):
    """获取节点在过去指定小时内的统计数据"""
    node = Node.query.filter_by(node_id=node_id).first()
    if not node:
        return None
    
    start_time = datetime.utcnow() - timedelta(hours=hours)
    history = NodeHistory.query.filter(
        NodeHistory.node_id == node.id,
        NodeHistory.timestamp >= start_time
    ).all()
    
    return {
        'total_motion': sum(h.motion_count for h in history),
        'avg_brightness': sum(h.brightness for h in history) / len(history) if history else 0,
        'emergency_count': sum(1 for h in history if h.emergency_triggered)
    } 