from flask import Blueprint, jsonify, request
from app.models.node import Node, NodeHistory
from app import db, socketio
from datetime import datetime

bp = Blueprint('api', __name__)

@bp.route('/api/nodes', methods=['GET'])
def get_nodes():
    nodes = Node.query.all()
    return jsonify([{
        'id': node.id,
        'node_id': node.node_id,
        'location': node.location,
        'status': {
            'distance': node.distance,
            'motion_count': node.motion_count,
            'emergency_triggered': node.emergency_triggered,
            'brightness': node.brightness
        }
    } for node in nodes])

@bp.route('/api/nodes/<node_id>/update', methods=['POST'])
def update_node(node_id):
    data = request.json
    node = Node.query.filter_by(node_id=node_id).first()
    
    if node:
        # 更新节点状态
        node.distance = data.get('distance')
        node.motion_count = data.get('motion_count')
        node.emergency_triggered = data.get('emergency_triggered')
        node.brightness = data.get('brightness')
        node.last_update = datetime.utcnow()
        
        # 创建历史记录
        history = NodeHistory(
            node=node,
            distance=data.get('distance'),
            motion_count=data.get('motion_count'),
            emergency_triggered=data.get('emergency_triggered'),
            brightness=data.get('brightness')
        )
        
        db.session.add(history)
        db.session.commit()
        
        # 通过WebSocket广播更新
        socketio.emit('node_update', {
            'node_id': node_id,
            'data': data
        })
        
        return jsonify({'status': 'success'})
    
    return jsonify({'status': 'error', 'message': 'Node not found'}), 404 