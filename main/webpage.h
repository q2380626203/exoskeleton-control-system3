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
    <title>电机控制系统</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Arial', sans-serif;
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

        .btn {
            padding: 12px 20px;
            font-size: 14px;
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

        .btn-primary {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }

        .btn-success {
            background: linear-gradient(135deg, #84fab0 0%, #8fd3f4 100%);
            color: #333;
        }

        .btn-danger {
            background: linear-gradient(135deg, #fa709a 0%, #fee140 100%);
            color: white;
        }

        .btn-warning {
            background: linear-gradient(135deg, #ffecd2 0%, #fcb69f 100%);
            color: #333;
        }

        .button-group {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin-bottom: 15px;
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
        <h1>🤖 电机控制系统</h1>

        <!-- 基本控制 -->
        <h2 class="section-title" onclick="toggleSection('basic-control')">▼ 基本控制</h2>
        <div id="basic-control" class="collapsible-content active">
            <div class="button-group">
                <button class="btn btn-success" onclick="sendCommand('start')">启动</button>
                <button class="btn btn-danger" onclick="sendCommand('stop')">停止</button>
            </div>
        </div>

        <!-- 全局参数 -->
        <h2 class="section-title" onclick="toggleSection('global-params')">▼ 全局参数</h2>
        <div id="global-params" class="collapsible-content active">
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
                <input type="number" id="m1_trigger_speed" step="0.01" value="0.75">
            </div>

            <div class="param-grid">
                <div class="input-group">
                    <label>Phase1持续时间(ms):</label>
                    <input type="number" id="m1_phase1_duration" value="400">
                </div>
                <div class="input-group">
                    <label>Phase2持续时间(ms):</label>
                    <input type="number" id="m1_phase2_duration" value="400">
                </div>
                <div class="input-group">
                    <label>等待时间(ms):</label>
                    <input type="number" id="m1_waiting_duration" value="300">
                </div>
                <div class="input-group">
                    <label>空闲时间(ms):</label>
                    <input type="number" id="m1_idle_duration" value="50">
                </div>
            </div>

            <div class="param-group">
                <h3>Phase1 参数</h3>
                <div class="param-grid">
                    <div class="input-group">
                        <label>Vel:</label>
                        <input type="number" id="m1_p1_vel" step="0.1" value="15.0">
                    </div>
                    <div class="input-group">
                        <label>Torque:</label>
                        <input type="number" id="m1_p1_torque" step="0.1" value="0.9">
                    </div>
                    <div class="input-group">
                        <label>Kp:</label>
                        <input type="number" id="m1_p1_kp" step="0.01" value="0.0">
                    </div>
                    <div class="input-group">
                        <label>Kd:</label>
                        <input type="number" id="m1_p1_kd" step="0.01" value="0.04">
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
                        <input type="number" id="m1_p2_torque" step="0.1" value="-0.5">
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
                <input type="number" id="m2_trigger_speed" step="0.01" value="-0.75">
            </div>

            <div class="param-grid">
                <div class="input-group">
                    <label>Phase1持续时间(ms):</label>
                    <input type="number" id="m2_phase1_duration" value="400">
                </div>
                <div class="input-group">
                    <label>Phase2持续时间(ms):</label>
                    <input type="number" id="m2_phase2_duration" value="400">
                </div>
                <div class="input-group">
                    <label>等待时间(ms):</label>
                    <input type="number" id="m2_waiting_duration" value="300">
                </div>
                <div class="input-group">
                    <label>空闲时间(ms):</label>
                    <input type="number" id="m2_idle_duration" value="50">
                </div>
            </div>

            <div class="param-group">
                <h3>Phase1 参数</h3>
                <div class="param-grid">
                    <div class="input-group">
                        <label>Vel:</label>
                        <input type="number" id="m2_p1_vel" step="0.1" value="-15.0">
                    </div>
                    <div class="input-group">
                        <label>Torque:</label>
                        <input type="number" id="m2_p1_torque" step="0.1" value="-0.9">
                    </div>
                    <div class="input-group">
                        <label>Kp:</label>
                        <input type="number" id="m2_p1_kp" step="0.01" value="0.0">
                    </div>
                    <div class="input-group">
                        <label>Kd:</label>
                        <input type="number" id="m2_p1_kd" step="0.01" value="0.04">
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
                        <input type="number" id="m2_p2_torque" step="0.1" value="0.5">
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
                    showResponse('电机1参数已读取', 'success');
                })
                .catch(error => {
                    showResponse('读取失败: ' + error, 'error');
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
                    showResponse('电机2参数已读取', 'success');
                })
                .catch(error => {
                    showResponse('读取失败: ' + error, 'error');
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
