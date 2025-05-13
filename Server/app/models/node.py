from app import db
from datetime import datetime

class Node(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    node_id = db.Column(db.String(50), unique=True, nullable=False)
    location = db.Column(db.String(100))
    last_update = db.Column(db.DateTime, default=datetime.utcnow)
    
    # 当前状态
    distance = db.Column(db.Float)
    motion_count = db.Column(db.Integer, default=0)
    emergency_triggered = db.Column(db.Boolean, default=False)
    brightness = db.Column(db.Integer)
    
    # 历史记录
    history = db.relationship('NodeHistory', backref='node', lazy=True)

class NodeHistory(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    node_id = db.Column(db.Integer, db.ForeignKey('node.id'))
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    distance = db.Column(db.Float)
    motion_count = db.Column(db.Integer)
    emergency_triggered = db.Column(db.Boolean)
    brightness = db.Column(db.Integer) 