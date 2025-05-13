from datetime import datetime
import json

def format_datetime(dt):
    """格式化日期时间"""
    if isinstance(dt, datetime):
        return dt.isoformat()
    return dt

def calculate_brightness(distance, motion_count):
    """根据距离和移动物体数量计算亮度"""
    # 基础亮度
    base_brightness = 100
    
    # 根据距离调整
    if distance < 100:  # 距离小于1米
        distance_factor = 1.0
    elif distance < 300:  # 距离1-3米
        distance_factor = 0.8
    else:  # 距离大于3米
        distance_factor = 0.6
    
    # 根据移动物体数量调整
    motion_factor = min(1.0, 0.2 + (motion_count * 0.1))
    
    # 计算最终亮度
    brightness = int(base_brightness * distance_factor * motion_factor)
    
    # 确保亮度在0-255范围内
    return max(0, min(255, brightness))

def validate_node_data(data):
    """验证节点数据"""
    required_fields = ['distance', 'motion_count', 'emergency_triggered', 'brightness']
    
    if not isinstance(data, dict):
        return False, "数据必须是JSON对象"
    
    for field in required_fields:
        if field not in data:
            return False, f"缺少必需字段: {field}"
    
    if not isinstance(data['distance'], (int, float)) or data['distance'] < 0:
        return False, "距离必须是正数"
    
    if not isinstance(data['motion_count'], int) or data['motion_count'] < 0:
        return False, "移动计数必须是非负整数"
    
    if not isinstance(data['emergency_triggered'], bool):
        return False, "紧急状态必须是布尔值"
    
    if not isinstance(data['brightness'], int) or data['brightness'] < 0 or data['brightness'] > 255:
        return False, "亮度必须在0-255范围内"
    
    return True, "数据验证通过" 