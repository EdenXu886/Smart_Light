from flask import Blueprint, jsonify, request
from functools import wraps
import jwt
from datetime import datetime, timedelta
from app.config import Config

bp = Blueprint('auth', __name__)

def token_required(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        token = request.headers.get('Authorization')
        if not token:
            return jsonify({'message': 'Token is missing!'}), 401
        try:
            token = token.split(' ')[1]  # Remove 'Bearer ' prefix
            data = jwt.decode(token, Config.SECRET_KEY, algorithms=["HS256"])
        except:
            return jsonify({'message': 'Token is invalid!'}), 401
        return f(*args, **kwargs)
    return decorated

@bp.route('/api/auth/login', methods=['POST'])
def login():
    auth = request.json
    if not auth or not auth.get('username') or not auth.get('password'):
        return jsonify({'message': 'Could not verify'}), 401
    
    # 这里应该添加实际的用户验证逻辑
    if auth.get('username') == 'admin' and auth.get('password') == 'admin':
        token = jwt.encode({
            'user': auth.get('username'),
            'exp': datetime.utcnow() + timedelta(hours=24)
        }, Config.SECRET_KEY)
        return jsonify({'token': token})
    
    return jsonify({'message': 'Invalid credentials'}), 401 