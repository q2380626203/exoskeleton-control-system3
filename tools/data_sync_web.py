#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据同步Web服务

功能：
1. 在80端口提供Web界面
2. 点击按钮将data目录文件复制到百度网盘同步目录
3. 复制完成后跳转到百度网盘分享链接

配置：
- 源目录: C:/Users/Administrator/Desktop/esp32server/data
- 目标目录: C:/Users/Administrator/Documents/BaiduSyncdisk/data
- 百度网盘链接: https://pan.baidu.com/s/1ihDJ99CP_qZBYfuP3VegLA?pwd=data
"""

import os
import shutil
import http.server
import socketserver
from datetime import datetime
import json
import urllib.parse

# 配置
WEB_PORT = 80
SOURCE_DIR = r'C:\Users\Administrator\Desktop\esp32server\data'
DEST_DIR = r'C:\Users\Administrator\Documents\BaiduSyncdisk\data'
BAIDU_PAN_URL = 'https://pan.baidu.com/s/1ihDJ99CP_qZBYfuP3VegLA?pwd=data'

# HTML页面
HTML_PAGE = '''<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32数据同步</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        .container {
            background: white;
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            text-align: center;
            max-width: 500px;
            width: 90%;
        }
        h1 {
            color: #333;
            margin-bottom: 10px;
            font-size: 28px;
        }
        .subtitle {
            color: #666;
            margin-bottom: 30px;
            font-size: 14px;
        }
        .btn {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            padding: 18px 50px;
            font-size: 18px;
            border-radius: 50px;
            cursor: pointer;
            transition: transform 0.3s, box-shadow 0.3s;
            outline: none;
        }
        .btn:hover {
            transform: translateY(-3px);
            box-shadow: 0 10px 30px rgba(102, 126, 234, 0.4);
        }
        .btn:active {
            transform: translateY(0);
        }
        .btn:disabled {
            background: #ccc;
            cursor: not-allowed;
            transform: none;
            box-shadow: none;
        }
        .status {
            margin-top: 30px;
            padding: 15px;
            border-radius: 10px;
            display: none;
        }
        .status.success {
            background: #d4edda;
            color: #155724;
            display: block;
        }
        .status.error {
            background: #f8d7da;
            color: #721c24;
            display: block;
        }
        .status.loading {
            background: #e7f1ff;
            color: #004085;
            display: block;
        }
        .file-list {
            text-align: left;
            margin-top: 15px;
            max-height: 200px;
            overflow-y: auto;
            font-size: 12px;
            background: #f8f9fa;
            padding: 10px;
            border-radius: 5px;
        }
        .file-item {
            padding: 3px 0;
            border-bottom: 1px solid #eee;
        }
        .info {
            margin-top: 20px;
            font-size: 12px;
            color: #888;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>📊 ESP32 数据同步</h1>
        <p class="subtitle">将采集数据同步到百度网盘</p>

        <button class="btn" id="syncBtn" onclick="syncData()">
            ☁️ 上传数据
        </button>

        <div class="status" id="status"></div>

        <div class="info">
            <p>源: C:\\Users\\Administrator\\Desktop\\esp32server\\data</p>
            <p>目标: 百度网盘同步目录</p>
        </div>
    </div>

    <script>
        function syncData() {
            const btn = document.getElementById('syncBtn');
            const status = document.getElementById('status');

            btn.disabled = true;
            btn.textContent = '⏳ 同步中...';
            status.className = 'status loading';
            status.innerHTML = '正在复制文件，请稍候...';

            fetch('/sync', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        status.className = 'status success';
                        let html = '✅ 同步完成！共复制 ' + data.count + ' 个文件<br>';
                        html += '<small>即将跳转到百度网盘...</small>';
                        if (data.files && data.files.length > 0) {
                            html += '<div class="file-list">';
                            data.files.forEach(f => {
                                html += '<div class="file-item">' + f + '</div>';
                            });
                            html += '</div>';
                        }
                        status.innerHTML = html;

                        // 2秒后跳转
                        setTimeout(() => {
                            window.open('BAIDU_URL', '_blank');
                        }, 2000);
                    } else {
                        status.className = 'status error';
                        status.innerHTML = '❌ 同步失败: ' + data.error;
                    }
                })
                .catch(err => {
                    status.className = 'status error';
                    status.innerHTML = '❌ 请求失败: ' + err;
                })
                .finally(() => {
                    btn.disabled = false;
                    btn.textContent = '☁️ 上传数据';
                });
        }
    </script>
</body>
</html>
'''.replace('BAIDU_URL', BAIDU_PAN_URL)


def sync_files():
    """复制data目录文件到百度网盘同步目录"""
    result = {
        'success': False,
        'count': 0,
        'files': [],
        'error': None
    }

    try:
        # 确保源目录存在
        if not os.path.exists(SOURCE_DIR):
            result['error'] = f'源目录不存在: {SOURCE_DIR}'
            return result

        # 确保目标目录存在
        if not os.path.exists(DEST_DIR):
            os.makedirs(DEST_DIR)
            print(f'创建目标目录: {DEST_DIR}')

        # 遍历源目录
        copied_files = []
        for filename in os.listdir(SOURCE_DIR):
            src_path = os.path.join(SOURCE_DIR, filename)
            dst_path = os.path.join(DEST_DIR, filename)

            # 只复制文件
            if os.path.isfile(src_path):
                shutil.copy2(src_path, dst_path)
                copied_files.append(filename)
                print(f'复制: {filename}')

        result['success'] = True
        result['count'] = len(copied_files)
        result['files'] = copied_files
        print(f'同步完成，共复制 {len(copied_files)} 个文件')

    except Exception as e:
        result['error'] = str(e)
        print(f'同步失败: {e}')

    return result


class SyncHandler(http.server.BaseHTTPRequestHandler):
    """HTTP请求处理器"""

    def log_message(self, format, *args):
        """自定义日志格式"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        print(f'[{timestamp}] {args[0]}')

    def do_GET(self):
        """处理GET请求 - 返回网页"""
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        """处理POST请求 - 执行同步"""
        if self.path == '/sync':
            # 执行文件同步
            result = sync_files()

            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.end_headers()
            self.wfile.write(json.dumps(result, ensure_ascii=False).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()


def main():
    print('=' * 60)
    print(' ESP32 数据同步 Web 服务')
    print('=' * 60)
    print(f' 监听端口: {WEB_PORT}')
    print(f' 源目录:   {SOURCE_DIR}')
    print(f' 目标目录: {DEST_DIR}')
    print('=' * 60)
    print(f' 访问地址: http://localhost:{WEB_PORT}')
    print(' 按 Ctrl+C 停止服务')
    print('=' * 60)

    # 允许端口复用
    socketserver.TCPServer.allow_reuse_address = True

    with socketserver.TCPServer(('0.0.0.0', WEB_PORT), SyncHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print('\n服务已停止')


if __name__ == '__main__':
    main()
