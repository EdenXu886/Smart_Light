from flask import Flask, send_from_directory
from flask_socketio import SocketIO
from flask_sqlalchemy import SQLAlchemy
from flask_cors import CORS
from app.config import Config
from flask_migrate import Migrate

db = SQLAlchemy()
migrate = Migrate()
socketio = SocketIO()

def create_app(config_class=Config):
    app = Flask(__name__, static_folder="static", static_url_path="/static")
    app.config.from_object(config_class)
    
    # 初始化扩展
    db.init_app(app)
    migrate.init_app(app, db)
    socketio.init_app(app, cors_allowed_origins="*")
    CORS(app)
    
    # 强制导入 models
    from app.models import node

    # 注册蓝图
    from app.routes import api, auth
    app.register_blueprint(api.bp)
    app.register_blueprint(auth.bp)
    
    # 可选：添加根路由，便于测试
    @app.route('/')
    def index():
        return send_from_directory(app.static_folder, 'index.html')
    
    return app 