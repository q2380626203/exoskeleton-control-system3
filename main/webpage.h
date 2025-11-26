#ifndef WEBPAGE_H
#define WEBPAGE_H

#ifdef __cplusplus
extern "C" {
#endif

const char webpage_html[] = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>电机速度跟随控制系统</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Arial', 'Microsoft YaHei', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }

        .container {
            background: white;
            border-radius: 15px;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.2);
            padding: 25px;
            max-width: 900px;
            margin: 0 auto;
        }

        h1 {
            text-align: center;
            color: #333;
            margin-bottom: 25px;
            font-size: 26px;
        }

        .section-title {
            font-size: 18px;
            color: #667eea;
            margin: 20px 0 10px;
            font-weight: bold;
            cursor: pointer;
            user-select: none;
        }

        .section-title:hover {
            color: #764ba2;
        }

        .collapsible-content {
            max-height: 0;
            overflow: hidden;
            transition: max-height 0.3s ease;
        }

        .collapsible-content.active {
            max-height: 3000px;
        }

        /* 控制按钮区 */
        .control-section {
            background: #f8f9fa;
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 20px;
        }

        .button-group {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin-bottom: 15px;
        }

        .btn {
            padding: 12px 20px;
            font-size: 16px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.2s ease;
            font-weight: bold;
            text-transform: uppercase;
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        }

        .btn-success {
            background: linear-gradient(135deg, #84fab0 0%, #8fd3f4 100%);
            color: #333;
        }

        .btn-danger {
            background: linear-gradient(135deg, #fa709a 0%, #fee140 100%);
            color: white;
        }

        .btn-primary {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }

        .btn-warning {
            background: linear-gradient(135deg, #ffecd2 0%, #fcb69f 100%);
            color: #333;
        }

        /* 状态显示区 */
        .status-card {
            background: linear-gradient(135deg, #e0e7ff 0%, #f3e7ff 100%);
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
        }

        .status-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 12px;
            padding: 10px;
            background: white;
            border-radius: 6px;
        }

        .status-label {
            font-weight: bold;
            color: #555;
            font-size: 14px;
        }

        .status-value {
            font-size: 16px;
            color: #667eea;
            font-weight: bold;
        }

        .status-badge {
            padding: 6px 12px;
            border-radius: 20px;
            font-size: 14px;
            font-weight: bold;
        }

        .badge-idle { background: #e0e0e0; color: #666; }
        .badge-waiting { background: #fff3cd; color: #856404; }
        .badge-working { background: #d1ecf1; color: #0c5460; }
        .badge-phase { background: #d4edda; color: #155724; }

        /* 助力显示 */
        .torque-display {
            text-align: center;
            padding: 15px;
            background: white;
            border-radius: 8px;
            margin-bottom: 15px;
        }

        .torque-value {
            font-size: 32px;
            font-weight: bold;
            color: #667eea;
            margin: 10px 0;
        }

        /* 参数调整区 */
        .param-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-bottom: 15px;
        }

        .param-group {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 15px;
        }

        .param-group h3 {
            color: #555;
            font-size: 14px;
            margin-bottom: 10px;
        }

        .input-group {
            margin-bottom: 12px;
        }

        .input-group label {
            display: block;
            margin-bottom: 5px;
            color: #666;
            font-size: 13px;
            font-weight: bold;
        }

        .input-group input {
            width: 100%;
            padding: 8px;
            border: 2px solid #e0e0e0;
            border-radius: 5px;
            font-size: 14px;
        }

        .input-group input:focus {
            outline: none;
            border-color: #667eea;
        }

        .response {
            margin-top: 15px;
            padding: 12px;
            border-radius: 6px;
            background: #e7f3ff;
            color: #0066cc;
            display: none;
            font-size: 13px;
        }

        .response.show {
            display: block;
        }

        .response.error {
            background: #ffe7e7;
            color: #cc0000;
        }

        .response.success {
            background: #e7ffe7;
            color: #00cc00;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 电机速度跟随控制系统</h1>

        <!-- 控制按钮区 -->
        <div class="control-section">
            <h3 style="margin-bottom: 15px; color: #333;">基本控制</h3>
            <div class="button-group">
                <button class="btn btn-success" onclick="sendCommand('start')">▶ 启动</button>
                <button class="btn btn-danger" onclick="sendCommand('stop')">⏸ 关闭</button>
            </div>

            <h3 style="margin-bottom: 15px; color: #333;">助力调整</h3>
            <div class="button-group">
                <button class="btn btn-primary" onclick="sendCommand('assist_up')">➕ 增加助力</button>
                <button class="btn btn-warning" onclick="sendCommand('assist_down')">➖ 减少助力</button>
            </div>

            <!-- 助力值显示 -->
            <div class="torque-display">
                <div style="font-size: 14px; color: #666;">当前助力值</div>
                <div class="torque-value" id="torque-value">0.0</div>
            </div>
        </div>

        <!-- 状态机显示区 -->
        <div class="status-card">
            <h3 style="margin-bottom: 15px; color: #333;">📊 状态机信息</h3>
            <div class="status-row">
                <span class="status-label">当前状态:</span>
                <span class="status-badge badge-idle" id="state-text">空闲</span>
            </div>
            <div class="status-row">
                <span class="status-label">活动电机:</span>
                <span class="status-value" id="active-motor">无</span>
            </div>
            <div class="status-row">
                <span class="status-label">抬腿电机:</span>
                <span class="status-value" id="lifting-motor">无</span>
            </div>
        </div>

        <!-- 全局参数 -->
        <h2 class="section-title" onclick="toggleSection('global-params')">▼ 全局参数</h2>
        <div id="global-params" class="collapsible-content">
            <div class="input-group">
                <label>速度跟随阈值 (global_speed_follow_threshold):</label>
                <input type="number" id="threshold" step="0.1" value="6.0">
            </div>
            <button class="btn btn-primary" style="width: 100%;" onclick="setThreshold()">应用阈值</button>
        </div>

        <!-- 电机1参数 -->
        <h2 class="section-title" onclick="toggleSection('motor1-params')">▼ 电机1参数</h2>
        <div id="motor1-params" class="collapsible-content">
            <div class="input-group">
                <label>触发速度 (trigger_speed):</label>
                <input type="number" id="m1_trigger_speed" step="0.01" value="5.0">
            </div>

            <div class="param-grid">
                <div class="input-group">
                    <label>Phase1持续时间(ms):</label>
                    <input type="number" id="m1_phase1_duration" value="500">
                </div>
                <div class="input-group">
                    <label>Phase2持续时间(ms):</label>
                    <input type="number" id="m1_phase2_duration" value="350">
                </div>
                <div class="input-group">
                    <label>等待时间(ms):</label>
                    <input type="number" id="m1_waiting_duration" value="300">
                </div>
                <div class="input-group">
                    <label>空闲时间(ms):</label>
                    <input type="number" id="m1_idle_duration" value="100">
                </div>
            </div>

            <div class="param-group">
                <h3>Phase1 参数</h3>
                <div class="param-grid">
                    <div class="input-group">
                        <label>Vel:</label>
                        <input type="number" id="m1_p1_vel" step="0.1" value="10.0">
                    </div>
                    <div class="input-group">
                        <label>Torque:</label>
                        <input type="number" id="m1_p1_torque" step="0.1" value="0.7">
                    </div>
                    <div class="input-group">
                        <label>Kp:</label>
                        <input type="number" id="m1_p1_kp" step="0.01" value="0.0">
                    </div>
                    <div class="input-group">
                        <label>Kd:</label>
                        <input type="number" id="m1_p1_kd" step="0.01" value="0.08">
                    </div>
                </div>
            </div>

            <div class="param-group">
                <h3>Phase2 参数</h3>
                <div class="param-grid">
                    <div class="input-group">
                        <label>Vel:</label>
                        <input type="number" id="m1_p2_vel" step="0.1" value="-10.0">
                    </div>
                    <div class="input-group">
                        <label>Torque:</label>
                        <input type="number" id="m1_p2_torque" step="0.1" value="-0.3">
                    </div>
                    <div class="input-group">
                        <label>Kp:</label>
                        <input type="number" id="m1_p2_kp" step="0.01" value="0.0">
                    </div>
                    <div class="input-group">
                        <label>Kd:</label>
                        <input type="number" id="m1_p2_kd" step="0.01" value="0.03">
                    </div>
                </div>
            </div>

            <div class="button-group" style="margin-top: 10px;">
                <button class="btn btn-warning" onclick="loadMotor1Params()">读取当前值</button>
                <button class="btn btn-primary" onclick="setMotor1Params()">应用参数</button>
            </div>
        </div>

        <!-- 电机2参数 -->
        <h2 class="section-title" onclick="toggleSection('motor2-params')">▼ 电机2参数</h2>
        <div id="motor2-params" class="collapsible-content">
            <div class="input-group">
                <label>触发速度 (trigger_speed):</label>
                <input type="number" id="m2_trigger_speed" step="0.01" value="-5.0">
            </div>

            <div class="param-grid">
                <div class="input-group">
                    <label>Phase1持续时间(ms):</label>
                    <input type="number" id="m2_phase1_duration" value="500">
                </div>
                <div class="input-group">
                    <label>Phase2持续时间(ms):</label>
                    <input type="number" id="m2_phase2_duration" value="350">
                </div>
                <div class="input-group">
                    <label>等待时间(ms):</label>
                    <input type="number" id="m2_waiting_duration" value="300">
                </div>
                <div class="input-group">
                    <label>空闲时间(ms):</label>
                    <input type="number" id="m2_idle_duration" value="100">
                </div>
            </div>

            <div class="param-group">
                <h3>Phase1 参数</h3>
                <div class="param-grid">
                    <div class="input-group">
                        <label>Vel:</label>
                        <input type="number" id="m2_p1_vel" step="0.1" value="-10.0">
                    </div>
                    <div class="input-group">
                        <label>Torque:</label>
                        <input type="number" id="m2_p1_torque" step="0.1" value="-0.7">
                    </div>
                    <div class="input-group">
                        <label>Kp:</label>
                        <input type="number" id="m2_p1_kp" step="0.01" value="0.0">
                    </div>
                    <div class="input-group">
                        <label>Kd:</label>
                        <input type="number" id="m2_p1_kd" step="0.01" value="0.08">
                    </div>
                </div>
            </div>

            <div class="param-group">
                <h3>Phase2 参数</h3>
                <div class="param-grid">
                    <div class="input-group">
                        <label>Vel:</label>
                        <input type="number" id="m2_p2_vel" step="0.1" value="10.0">
                    </div>
                    <div class="input-group">
                        <label>Torque:</label>
                        <input type="number" id="m2_p2_torque" step="0.1" value="0.3">
                    </div>
                    <div class="input-group">
                        <label>Kp:</label>
                        <input type="number" id="m2_p2_kp" step="0.01" value="0.0">
                    </div>
                    <div class="input-group">
                        <label>Kd:</label>
                        <input type="number" id="m2_p2_kd" step="0.01" value="0.03">
                    </div>
                </div>
            </div>

            <div class="button-group" style="margin-top: 10px;">
                <button class="btn btn-warning" onclick="loadMotor2Params()">读取当前值</button>
                <button class="btn btn-primary" onclick="setMotor2Params()">应用参数</button>
            </div>
        </div>

        <div class="response" id="response"></div>
    </div>

    <script>
        // 页面加载时初始化
        window.onload = function() {
            loadMotor1Params();
            loadMotor2Params();
            startStatePolling();
        };

        // 状态轮询
        let statePollingInterval = null;
        function startStatePolling() {
            updateState(); // 立即更新一次
            statePollingInterval = setInterval(updateState, 500); // 每500ms更新一次
        }

        function updateState() {
            fetch('/api/state')
                .then(response => response.json())
                .then(data => {
                    // 更新状态文本和样式
                    const stateText = document.getElementById('state-text');
                    stateText.textContent = data.state;

                    // 根据状态ID设置不同样式
                    stateText.className = 'status-badge';
                    if (data.state_id === 0) {
                        stateText.classList.add('badge-idle');
                    } else if (data.state_id === 1 || data.state_id === 2) {
                        stateText.classList.add('badge-waiting');
                    } else if (data.state_id === 3 || data.state_id === 4) {
                        stateText.classList.add('badge-working');
                    } else if (data.state_id === 5 || data.state_id === 6) {
                        stateText.classList.add('badge-phase');
                    }

                    // 更新活动电机
                    document.getElementById('active-motor').textContent =
                        data.active_motor > 0 ? data.active_motor + '号电机' : '无';

                    // 更新抬腿电机
                    document.getElementById('lifting-motor').textContent =
                        data.lifting_motor > 0 ? data.lifting_motor + '号电机' : '无';

                    // 更新助力值
                    document.getElementById('torque-value').textContent = data.torque.toFixed(1);
                })
                .catch(error => {
                    console.error('获取状态失败:', error);
                });
        }

        function toggleSection(id) {
            const section = document.getElementById(id);
            section.classList.toggle('active');
        }

        function showResponse(message, type = 'info') {
            const responseDiv = document.getElementById('response');
            responseDiv.textContent = message;
            responseDiv.className = 'response show ' + type;
            setTimeout(() => {
                responseDiv.classList.remove('show');
            }, 3000);
        }

        function sendCommand(cmd) {
            fetch('/api/command?cmd=' + cmd)
                .then(response => response.json())
                .then(data => {
                    showResponse('命令执行成功: ' + cmd, 'success');
                    // 命令执行后立即更新状态
                    setTimeout(updateState, 100);
                })
                .catch(error => {
                    showResponse('命令执行失败: ' + error, 'error');
                });
        }

        function setThreshold() {
            const threshold = document.getElementById('threshold').value;
            fetch('/api/params?threshold=' + threshold)
                .then(response => response.json())
                .then(data => {
                    showResponse('阈值已设置: ' + threshold, 'success');
                })
                .catch(error => {
                    showResponse('设置失败: ' + error, 'error');
                });
        }

        function loadMotor1Params() {
            fetch('/api/get_motor_params?motor=1')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('m1_trigger_speed').value = data.trigger_speed;
                    document.getElementById('m1_phase1_duration').value = data.phase1_duration;
                    document.getElementById('m1_phase2_duration').value = data.phase2_duration;
                    document.getElementById('m1_waiting_duration').value = data.waiting_duration;
                    document.getElementById('m1_idle_duration').value = data.idle_duration;
                    document.getElementById('m1_p1_vel').value = data.p1_vel;
                    document.getElementById('m1_p1_torque').value = data.p1_torque;
                    document.getElementById('m1_p1_kp').value = data.p1_kp;
                    document.getElementById('m1_p1_kd').value = data.p1_kd;
                    document.getElementById('m1_p2_vel').value = data.p2_vel;
                    document.getElementById('m1_p2_torque').value = data.p2_torque;
                    document.getElementById('m1_p2_kp').value = data.p2_kp;
                    document.getElementById('m1_p2_kd').value = data.p2_kd;
                })
                .catch(error => {
                    console.error('读取电机1参数失败:', error);
                });
        }

        function loadMotor2Params() {
            fetch('/api/get_motor_params?motor=2')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('m2_trigger_speed').value = data.trigger_speed;
                    document.getElementById('m2_phase1_duration').value = data.phase1_duration;
                    document.getElementById('m2_phase2_duration').value = data.phase2_duration;
                    document.getElementById('m2_waiting_duration').value = data.waiting_duration;
                    document.getElementById('m2_idle_duration').value = data.idle_duration;
                    document.getElementById('m2_p1_vel').value = data.p1_vel;
                    document.getElementById('m2_p1_torque').value = data.p1_torque;
                    document.getElementById('m2_p1_kp').value = data.p1_kp;
                    document.getElementById('m2_p1_kd').value = data.p1_kd;
                    document.getElementById('m2_p2_vel').value = data.p2_vel;
                    document.getElementById('m2_p2_torque').value = data.p2_torque;
                    document.getElementById('m2_p2_kp').value = data.p2_kp;
                    document.getElementById('m2_p2_kd').value = data.p2_kd;
                })
                .catch(error => {
                    console.error('读取电机2参数失败:', error);
                });
        }

        function setMotor1Params() {
            const params = {
                motor: 1,
                trigger_speed: document.getElementById('m1_trigger_speed').value,
                phase1_duration: document.getElementById('m1_phase1_duration').value,
                phase2_duration: document.getElementById('m1_phase2_duration').value,
                waiting_duration: document.getElementById('m1_waiting_duration').value,
                idle_duration: document.getElementById('m1_idle_duration').value,
                p1_vel: document.getElementById('m1_p1_vel').value,
                p1_torque: document.getElementById('m1_p1_torque').value,
                p1_kp: document.getElementById('m1_p1_kp').value,
                p1_kd: document.getElementById('m1_p1_kd').value,
                p2_vel: document.getElementById('m1_p2_vel').value,
                p2_torque: document.getElementById('m1_p2_torque').value,
                p2_kp: document.getElementById('m1_p2_kp').value,
                p2_kd: document.getElementById('m1_p2_kd').value
            };

            const query = Object.keys(params).map(k => k + '=' + params[k]).join('&');
            fetch('/api/motor_params?' + query)
                .then(response => response.json())
                .then(data => {
                    showResponse('电机1参数已更新', 'success');
                })
                .catch(error => {
                    showResponse('设置失败: ' + error, 'error');
                });
        }

        function setMotor2Params() {
            const params = {
                motor: 2,
                trigger_speed: document.getElementById('m2_trigger_speed').value,
                phase1_duration: document.getElementById('m2_phase1_duration').value,
                phase2_duration: document.getElementById('m2_phase2_duration').value,
                waiting_duration: document.getElementById('m2_waiting_duration').value,
                idle_duration: document.getElementById('m2_idle_duration').value,
                p1_vel: document.getElementById('m2_p1_vel').value,
                p1_torque: document.getElementById('m2_p1_torque').value,
                p1_kp: document.getElementById('m2_p1_kp').value,
                p1_kd: document.getElementById('m2_p1_kd').value,
                p2_vel: document.getElementById('m2_p2_vel').value,
                p2_torque: document.getElementById('m2_p2_torque').value,
                p2_kp: document.getElementById('m2_p2_kp').value,
                p2_kd: document.getElementById('m2_p2_kd').value
            };

            const query = Object.keys(params).map(k => k + '=' + params[k]).join('&');
            fetch('/api/motor_params?' + query)
                .then(response => response.json())
                .then(data => {
                    showResponse('电机2参数已更新', 'success');
                })
                .catch(error => {
                    showResponse('设置失败: ' + error, 'error');
                });
        }
    </script>
</body>
</html>
)rawliteral";

#ifdef __cplusplus
}
#endif

#endif // WEBPAGE_H
