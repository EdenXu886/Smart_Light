const API_URL = '/api/nodes';
const dashboard = document.getElementById('dashboard');
const errorDiv = document.getElementById('error');

function showError(msg) {
    errorDiv.textContent = msg;
    errorDiv.style.display = 'block';
}
function hideError() {
    errorDiv.style.display = 'none';
}

function renderLamps(nodes) {
    dashboard.innerHTML = '';
    nodes.forEach(node => {
        const card = document.createElement('div');
        card.className = 'lamp-card' + (node.status.iot_ok === false ? ' error' : '');
        card.innerHTML = `
            <div class="lamp-title">节点ID: ${node.node_id}</div>
            <div class="lamp-status">亮度: <span class="lamp-brightness">${node.status.brightness}</span></div>
            <div class="lamp-status">距离: ${node.status.distance ?? '-'} mm</div>
            <div class="lamp-status">移动计数: ${node.status.motion_count ?? '-'} </div>
            <div class="lamp-status">紧急状态: <span style="color:${node.status.emergency_triggered ? 'red':'green'}">${node.status.emergency_triggered ? '紧急' : '正常'}</span></div>
            <div class="lamp-iot">IOT通信: <b>${node.status.iot_ok === false ? '未连接' : '正常'}</b></div>
        `;
        dashboard.appendChild(card);
    });
}

function fetchLamps() {
    fetch(API_URL)
        .then(res => {
            if (!res.ok) throw new Error('无法获取路灯数据');
            return res.json();
        })
        .then(data => {
            hideError();
            renderLamps(data);
        })
        .catch(err => {
            showError('数据获取失败: ' + err.message);
        });
}

// 初始加载
fetchLamps();
// 每5秒刷新一次
setInterval(fetchLamps, 5000); 