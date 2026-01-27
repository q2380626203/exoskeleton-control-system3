#ifndef WEBPAGE_H
#define WEBPAGE_H

static const char webpage_html[] = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>外骨骼控制系统</title>
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
            max-width: 800px;
            margin: 0 auto;
        }
        .header {
            text-align: center;
            color: white;
            margin-bottom: 30px;
        }
        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .server-time {
            font-size: 1.2em;
            color: #ffffff;
            margin-top: 10px;
            text-shadow: 1px 1px 2px rgba(0,0,0,0.3);
        }
        .server-time.disconnected {
            color: #ffcccc;
        }
        .card {
            background: white;
            border-radius: 15px;
            padding: 25px;
            margin-bottom: 20px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        .card-title {
            font-size: 1.5em;
            color: #667eea;
            margin-bottom: 20px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        .motor-display {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        .motor-item {
            background: #f8f9fa;
            padding: 20px;
            border-radius: 10px;
            text-align: center;
        }
        .motor-label {
            font-size: 1.1em;
            color: #666;
            margin-bottom: 10px;
        }
        .motor-value {
            font-size: 2.5em;
            font-weight: bold;
            color: #667eea;
            margin: 10px 0;
        }
        .motor-unit {
            font-size: 0.9em;
            color: #999;
        }
        .btn {
            background: #667eea;
            color: white;
            border: none;
            padding: 12px 30px;
            border-radius: 8px;
            font-size: 1em;
            cursor: pointer;
            transition: all 0.3s;
            width: 100%;
            margin-top: 10px;
        }
        .btn:hover {
            background: #5568d3;
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
        }
        .btn:active {
            transform: translateY(0);
        }
        .control-group {
            margin-bottom: 25px;
        }
        .control-label {
            font-size: 1.1em;
            color: #333;
            margin-bottom: 10px;
            display: block;
        }
        .control-row {
            display: flex;
            align-items: center;
            gap: 15px;
            margin-bottom: 15px;
        }
        .control-value {
            font-size: 1.3em;
            font-weight: bold;
            color: #667eea;
            min-width: 80px;
            text-align: center;
        }
        .btn-group {
            display: flex;
            gap: 10px;
            flex: 1;
        }
        .btn-small {
            padding: 10px 20px;
            flex: 1;
        }
        .status-info {
            background: #e8f5e9;
            padding: 15px;
            border-radius: 8px;
            margin-top: 15px;
            color: #2e7d32;
            text-align: center;
        }
        .range-display {
            font-size: 0.85em;
            color: #999;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🦾 登山外骨骼</h1>
            <div class="server-time" id="server-time">正在获取时间...</div>
        </div>

        <!-- 电机位置显示 -->
        <div class="card">
            <div class="card-title">📊 双腿位置</div>
            <div class="motor-display">
                <div class="motor-item">
                    <div class="motor-label">左腿</div>
                    <div class="motor-value" id="motor1-angle">0.0</div>
                    <div class="motor-unit">度 (°)</div>
                </div>
                <div class="motor-item">
                    <div class="motor-label">右腿</div>
                    <div class="motor-value" id="motor2-angle">0.0</div>
                    <div class="motor-unit">度 (°)</div>
                </div>
            </div>
            <button class="btn" onclick="setZeroPosition()">🎯 设置当前位置为0度</button>
            <div class="status-info" id="zero-status" style="display:none;">
                ✓ 零点已设置
            </div>
            <div class="status-info" id="zero-error" style="display:none;background:#ffebee;color:#c62828;">
                ✗ 设置失败
            </div>
        </div>

        <!-- 参数调整 -->
        <div class="card">
            <div class="card-title">⚙️ 参数调整</div>

            <!-- 抬腿力矩调整 -->
            <div class="control-group">
                <label class="control-label">抬腿力矩 (Phase1)</label>
                <div class="control-row">
                    <div class="control-value" id="torque-value">1.5</div>
                    <div class="btn-group">
                        <button class="btn btn-small" onclick="adjustTorque(false)">➖ 减小</button>
                        <button class="btn btn-small" onclick="adjustTorque(true)">➕ 增加</button>
                    </div>
                </div>
            </div>

            <!-- 压腿力矩调整 -->
            <div class="control-group">
                <label class="control-label">压腿力矩 (Phase2)</label>
                <div class="control-row">
                    <div class="control-value" id="phase2-value">0.7</div>
                    <div class="btn-group">
                        <button class="btn btn-small" onclick="adjustPhase2Torque(false)">➖ 减小</button>
                        <button class="btn btn-small" onclick="adjustPhase2Torque(true)">➕ 增加</button>
                    </div>
                </div>
            </div>
        </div>

    </div>

    <script>
        // 全局变量
        let motor1ZeroOffset = 0;
        let motor2ZeroOffset = 0;
        const GEAR_RATIO = 6.33;

        // 设置零点位置
        function setZeroPosition() {
            fetch('/api/get_motor_params')
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        motor1ZeroOffset = data.motor1_pos;
                        motor2ZeroOffset = data.motor2_pos;

                        // 保存到localStorage
                        localStorage.setItem('motor1_zero', motor1ZeroOffset);
                        localStorage.setItem('motor2_zero', motor2ZeroOffset);

                        document.getElementById('zero-status').style.display = 'block';
                        document.getElementById('zero-error').style.display = 'none';
                        setTimeout(() => {
                            document.getElementById('zero-status').style.display = 'none';
                        }, 3000);
                    } else {
                        document.getElementById('zero-error').textContent = '✗ ' + (data.message || '设置失败');
                        document.getElementById('zero-error').style.display = 'block';
                        document.getElementById('zero-status').style.display = 'none';
                        setTimeout(() => {
                            document.getElementById('zero-error').style.display = 'none';
                        }, 5000);
                    }
                })
                .catch(error => {
                    document.getElementById('zero-error').textContent = '✗ 网络错误';
                    document.getElementById('zero-error').style.display = 'block';
                    document.getElementById('zero-status').style.display = 'none';
                    setTimeout(() => {
                        document.getElementById('zero-error').style.display = 'none';
                    }, 5000);
                });
        }

        // 调整力矩
        function adjustTorque(increase) {
            const action = increase ? 'increase' : 'decrease';
            fetch(`/api/adjust_torque?action=${action}`)
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        document.getElementById('torque-value').textContent = data.torque.toFixed(1);
                    }
                })
                .catch(error => console.error('Error:', error));
        }

        // 调整压腿力矩（Phase2）
        function adjustPhase2Torque(increase) {
            const action = increase ? 'increase' : 'decrease';
            fetch(`/api/adjust_phase2_torque?action=${action}`)
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        document.getElementById('phase2-value').textContent = data.phase2_torque.toFixed(1);
                    }
                })
                .catch(error => console.error('Error:', error));
        }

        // 更新电机位置显示
        function updateMotorPositions() {
            fetch('/api/get_motor_params')
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        // 计算相对于零点的角度，并转换为度数
                        const motor1Rad = data.motor1_pos - motor1ZeroOffset;
                        const motor2Rad = data.motor2_pos - motor2ZeroOffset;

                        // 弧度转角度，再除以减速比（内部位置/减速比=外部角度）
                        const motor1Deg = (motor1Rad * 180 / Math.PI) / GEAR_RATIO;
                        const motor2Deg = (motor2Rad * 180 / Math.PI) / GEAR_RATIO;

                        document.getElementById('motor1-angle').textContent = motor1Deg.toFixed(1);
                        document.getElementById('motor2-angle').textContent = motor2Deg.toFixed(1);
                    }
                })
                .catch(error => console.error('Error:', error));
        }

        // 初始化：加载当前参数
        function loadCurrentParams() {
            fetch('/api/get_current_params')
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        document.getElementById('torque-value').textContent = data.torque.toFixed(1);
                        document.getElementById('phase2-value').textContent = data.phase2_torque.toFixed(1);
                    }
                })
                .catch(error => console.error('Error:', error));
        }

        // 更新服务器时间显示
        function updateServerTime() {
            fetch('/api/get_server_time')
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        const timeElement = document.getElementById('server-time');
                        if (data.synced) {
                            timeElement.textContent = data.time;
                            timeElement.classList.remove('disconnected');
                        } else {
                            timeElement.textContent = data.time;
                            timeElement.classList.add('disconnected');
                        }
                    }
                })
                .catch(error => {
                    document.getElementById('server-time').textContent = '服务器未连接';
                    document.getElementById('server-time').classList.add('disconnected');
                });
        }

        // 页面加载时初始化
        window.onload = function() {
            // 从localStorage加载零点位置
            const savedMotor1Zero = localStorage.getItem('motor1_zero');
            const savedMotor2Zero = localStorage.getItem('motor2_zero');
            if (savedMotor1Zero !== null) {
                motor1ZeroOffset = parseFloat(savedMotor1Zero);
            }
            if (savedMotor2Zero !== null) {
                motor2ZeroOffset = parseFloat(savedMotor2Zero);
            }

            loadCurrentParams();
            updateMotorPositions();
            updateServerTime();
            // 每100ms更新一次电机位置
            setInterval(updateMotorPositions, 100);
            // 每1秒更新一次服务器时间
            setInterval(updateServerTime, 1000);
        };
    </script>
</body>
</html>
)rawliteral";

#endif // WEBPAGE_H
