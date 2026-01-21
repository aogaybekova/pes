#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Web Application - Веб-приложение для управления роботом SpotMicro
Позволяет управлять роботом через веб-интерфейс и работать с камерой
"""

import os
import sys
import json
import time
import threading
from datetime import datetime
from flask import Flask, render_template, jsonify, request, send_from_directory

# Импорт SmartController
try:
    from smart_control import SmartRobotController
except ImportError:
    print("[WARN] SmartRobotController не найден")
    SmartRobotController = None

# Камера Raspberry Pi
try:
    from picamera2 import Picamera2
    CAMERA_AVAILABLE = True
except ImportError:
    print("[WARN] Picamera2 не доступна")
    CAMERA_AVAILABLE = False


# Создаем Flask приложение
app = Flask(__name__, template_folder='templates', static_folder='static')

# Глобальные переменные
robot_controller = None
camera = None
camera_active = False
photos_dir = "photos"

# Создаем директории
if not os.path.exists(photos_dir):
    os.makedirs(photos_dir)

if not os.path.exists('templates'):
    os.makedirs('templates')

if not os.path.exists('static'):
    os.makedirs('static')


class CameraManager:
    """Менеджер камеры Raspberry Pi"""
    
    def __init__(self):
        self.camera = None
        self.is_active = False
        self.photos_dir = photos_dir
        
        if not os.path.exists(self.photos_dir):
            os.makedirs(self.photos_dir)
    
    def activate(self) -> bool:
        """Активирует камеру"""
        if not CAMERA_AVAILABLE:
            print("[Camera] Picamera2 недоступна")
            return False
        
        if self.is_active:
            return True
        
        try:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
            self.camera.configure(config)
            self.camera.start()
            self.is_active = True
            print("[Camera] Камера активирована")
            return True
        except Exception as e:
            print(f"[Camera] Ошибка активации: {e}")
            return False
    
    def deactivate(self):
        """Деактивирует камеру"""
        if self.camera and self.is_active:
            try:
                self.camera.stop()
                self.camera.close()
                self.camera = None
                self.is_active = False
                print("[Camera] Камера деактивирована")
            except Exception as e:
                print(f"[Camera] Ошибка деактивации: {e}")
    
    def take_photo(self) -> str:
        """
        Делает фото и сохраняет в директорию
        
        Returns:
            Имя файла или None при ошибке
        """
        if not self.is_active:
            if not self.activate():
                return None
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"photo_{timestamp}.jpg"
            filepath = os.path.join(self.photos_dir, filename)
            
            self.camera.capture_file(filepath)
            print(f"[Camera] Фото сохранено: {filepath}")
            return filename
        except Exception as e:
            print(f"[Camera] Ошибка съемки: {e}")
            return None
    
    def get_photos_list(self) -> list:
        """Возвращает список сохраненных фото"""
        try:
            photos = []
            for f in os.listdir(self.photos_dir):
                if f.lower().endswith(('.jpg', '.jpeg', '.png')):
                    filepath = os.path.join(self.photos_dir, f)
                    photos.append({
                        "filename": f,
                        "timestamp": os.path.getmtime(filepath),
                        "size": os.path.getsize(filepath)
                    })
            return sorted(photos, key=lambda x: x['timestamp'], reverse=True)
        except Exception as e:
            print(f"[Camera] Ошибка получения списка: {e}")
            return []


# Инициализация менеджера камеры
camera_manager = CameraManager()


# ==================== РОУТЫ ====================

@app.route('/')
def index():
    """Главная страница"""
    return render_template('index.html')


@app.route('/api/status')
def get_status():
    """Получение статуса робота и датчиков"""
    global robot_controller
    
    status = {
        "robot_connected": robot_controller is not None,
        "camera_active": camera_manager.is_active,
        "timestamp": time.time()
    }
    
    if robot_controller:
        sensor_data = robot_controller.get_sensor_data()
        status.update(sensor_data)
    else:
        status.update({
            "ultrasonic": {"left": -1, "right": -1},
            "touch": False,
            "mpu6050": {"acceleration": (0, 0, 0), "gyroscope": (0, 0, 0)},
            "robot_state": "disconnected",
            "current_action": "none",
            "is_moving": False
        })
    
    return jsonify(status)


@app.route('/api/command', methods=['POST'])
def send_command():
    """Отправка команды роботу"""
    global robot_controller
    
    data = request.get_json()
    command = data.get('command', '')
    
    if not command:
        return jsonify({"success": False, "error": "Команда не указана"})
    
    if robot_controller:
        robot_controller.add_command(command)
        return jsonify({
            "success": True, 
            "message": f"Команда '{command}' отправлена"
        })
    else:
        return jsonify({
            "success": False, 
            "error": "Робот не подключен"
        })


@app.route('/api/camera/activate', methods=['POST'])
def activate_camera():
    """Активация камеры"""
    success = camera_manager.activate()
    return jsonify({
        "success": success,
        "message": "Камера активирована" if success else "Ошибка активации камеры"
    })


@app.route('/api/camera/deactivate', methods=['POST'])
def deactivate_camera():
    """Деактивация камеры"""
    camera_manager.deactivate()
    return jsonify({
        "success": True,
        "message": "Камера деактивирована"
    })


@app.route('/api/camera/photo', methods=['POST'])
def take_photo():
    """Сделать фото"""
    filename = camera_manager.take_photo()
    if filename:
        return jsonify({
            "success": True,
            "filename": filename,
            "message": "Фото сохранено"
        })
    else:
        return jsonify({
            "success": False,
            "error": "Не удалось сделать фото"
        })


@app.route('/api/camera/photos')
def get_photos():
    """Получить список фото"""
    photos = camera_manager.get_photos_list()
    return jsonify({
        "success": True,
        "photos": photos,
        "count": len(photos)
    })


@app.route('/photos/<filename>')
def serve_photo(filename):
    """Отдача файла фото"""
    return send_from_directory(photos_dir, filename)


@app.route('/api/rl/stats')
def get_rl_stats():
    """Получение статистики обучения"""
    global robot_controller
    
    if robot_controller and robot_controller.rl_logger:
        stats = robot_controller.rl_logger.get_statistics()
        return jsonify({"success": True, "stats": stats})
    else:
        return jsonify({"success": False, "error": "RL логгер недоступен"})


@app.route('/api/rl/new_episode', methods=['POST'])
def new_rl_episode():
    """Начать новый эпизод обучения"""
    global robot_controller
    
    if robot_controller and robot_controller.rl_logger:
        robot_controller.rl_logger.new_episode()
        return jsonify({"success": True, "message": "Новый эпизод начат"})
    else:
        return jsonify({"success": False, "error": "RL логгер недоступен"})


# ==================== HTML ШАБЛОН ====================

HTML_TEMPLATE = '''<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SpotMicro Control Panel</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #eee;
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        
        h1 {
            text-align: center;
            margin-bottom: 30px;
            color: #00d4ff;
            text-shadow: 0 0 10px rgba(0, 212, 255, 0.5);
        }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
        }
        
        .card {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 15px;
            padding: 20px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }
        
        .card h2 {
            color: #00d4ff;
            margin-bottom: 15px;
            font-size: 1.2em;
            border-bottom: 1px solid rgba(0, 212, 255, 0.3);
            padding-bottom: 10px;
        }
        
        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 200px;
            margin: 0 auto;
        }
        
        .btn {
            padding: 15px 20px;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            font-size: 14px;
            font-weight: bold;
            transition: all 0.3s ease;
            text-transform: uppercase;
        }
        
        .btn-primary {
            background: linear-gradient(45deg, #00d4ff, #0099cc);
            color: white;
        }
        
        .btn-danger {
            background: linear-gradient(45deg, #ff4757, #cc3344);
            color: white;
        }
        
        .btn-success {
            background: linear-gradient(45deg, #2ed573, #1e9e5e);
            color: white;
        }
        
        .btn-warning {
            background: linear-gradient(45deg, #ffa502, #cc8402);
            color: white;
        }
        
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.3);
        }
        
        .btn:active {
            transform: translateY(0);
        }
        
        .btn-direction {
            width: 50px;
            height: 50px;
            font-size: 20px;
        }
        
        .sensor-data {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }
        
        .sensor-item {
            background: rgba(0, 0, 0, 0.3);
            padding: 10px;
            border-radius: 8px;
            text-align: center;
        }
        
        .sensor-value {
            font-size: 1.5em;
            font-weight: bold;
            color: #00d4ff;
        }
        
        .sensor-label {
            font-size: 0.8em;
            color: #aaa;
            margin-top: 5px;
        }
        
        .status-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }
        
        .status-online { background: #2ed573; }
        .status-offline { background: #ff4757; }
        
        .command-buttons {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
        }
        
        .camera-controls {
            display: flex;
            gap: 10px;
            margin-bottom: 15px;
        }
        
        .photo-gallery {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(100px, 1fr));
            gap: 10px;
            max-height: 200px;
            overflow-y: auto;
        }
        
        .photo-item {
            aspect-ratio: 1;
            border-radius: 8px;
            overflow: hidden;
        }
        
        .photo-item img {
            width: 100%;
            height: 100%;
            object-fit: cover;
        }
        
        .log-output {
            background: rgba(0, 0, 0, 0.5);
            border-radius: 8px;
            padding: 10px;
            height: 150px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 12px;
        }
        
        .log-entry {
            margin-bottom: 5px;
            padding: 3px 5px;
            border-radius: 3px;
        }
        
        .log-info { background: rgba(0, 212, 255, 0.2); }
        .log-success { background: rgba(46, 213, 115, 0.2); }
        .log-error { background: rgba(255, 71, 87, 0.2); }
        
        .empty-cell {
            visibility: hidden;
        }
        
        @media (max-width: 600px) {
            body { padding: 10px; }
            .card { padding: 15px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🐕 SpotMicro Control Panel</h1>
        
        <div class="grid">
            <!-- Статус -->
            <div class="card">
                <h2>📊 Статус системы</h2>
                <p>
                    <span class="status-indicator" id="robot-status"></span>
                    Робот: <span id="robot-state">--</span>
                </p>
                <p>
                    <span class="status-indicator" id="camera-status"></span>
                    Камера: <span id="camera-state">--</span>
                </p>
                <p style="margin-top: 10px;">
                    Действие: <span id="current-action" style="color: #00d4ff;">--</span>
                </p>
            </div>
            
            <!-- Управление движением -->
            <div class="card">
                <h2>🎮 Управление движением</h2>
                <div class="control-grid">
                    <div class="empty-cell"></div>
                    <button class="btn btn-primary btn-direction" onclick="sendCommand('forward')">↑</button>
                    <div class="empty-cell"></div>
                    
                    <button class="btn btn-primary btn-direction" onclick="sendCommand('turn_left')">↺</button>
                    <button class="btn btn-danger btn-direction" onclick="sendCommand('stop_walk')">⬤</button>
                    <button class="btn btn-primary btn-direction" onclick="sendCommand('turn_right')">↻</button>
                    
                    <div class="empty-cell"></div>
                    <button class="btn btn-primary btn-direction" onclick="sendCommand('backward')">↓</button>
                    <div class="empty-cell"></div>
                </div>
            </div>
            
            <!-- Команды -->
            <div class="card">
                <h2>📝 Команды</h2>
                <div class="command-buttons">
                    <button class="btn btn-success" onclick="sendCommand('stand')">Встать</button>
                    <button class="btn btn-warning" onclick="sendCommand('sit')">Сесть</button>
                    <button class="btn btn-primary" onclick="sendCommand('lie')">Лечь</button>
                    <button class="btn btn-primary" onclick="sendCommand('twist')">Покачаться</button>
                    <button class="btn btn-success" onclick="sendCommand('paw_right')">Правая лапа</button>
                    <button class="btn btn-success" onclick="sendCommand('paw_left')">Левая лапа</button>
                    <button class="btn btn-warning" onclick="sendCommand('paw_down')">Опустить лапы</button>
                    <button class="btn btn-danger" onclick="sendCommand('stop')">СТОП</button>
                </div>
            </div>
            
            <!-- Датчики -->
            <div class="card">
                <h2>📡 Датчики</h2>
                <div class="sensor-data">
                    <div class="sensor-item">
                        <div class="sensor-value" id="dist-left">--</div>
                        <div class="sensor-label">Левый датчик (см)</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="dist-right">--</div>
                        <div class="sensor-label">Правый датчик (см)</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="touch-sensor">--</div>
                        <div class="sensor-label">Датчик касания</div>
                    </div>
                    <div class="sensor-item">
                        <div class="sensor-value" id="mpu-data">--</div>
                        <div class="sensor-label">MPU6050</div>
                    </div>
                </div>
            </div>
            
            <!-- Камера -->
            <div class="card">
                <h2>📷 Камера</h2>
                <div class="camera-controls">
                    <button class="btn btn-success" onclick="activateCamera()">Вкл</button>
                    <button class="btn btn-danger" onclick="deactivateCamera()">Выкл</button>
                    <button class="btn btn-primary" onclick="takePhoto()">Фото</button>
                    <button class="btn btn-warning" onclick="loadPhotos()">Обновить</button>
                </div>
                <div class="photo-gallery" id="photo-gallery">
                    <p style="color: #aaa; grid-column: 1/-1; text-align: center;">Нет фото</p>
                </div>
            </div>
            
            <!-- Лог -->
            <div class="card">
                <h2>📋 Журнал</h2>
                <div class="log-output" id="log-output">
                    <div class="log-entry log-info">Система запущена</div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        // API запросы
        async function sendCommand(command) {
            try {
                const response = await fetch('/api/command', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({command: command})
                });
                const data = await response.json();
                addLog(data.success ? 'success' : 'error', 
                       data.message || data.error);
            } catch (e) {
                addLog('error', 'Ошибка отправки команды');
            }
        }
        
        async function activateCamera() {
            try {
                const response = await fetch('/api/camera/activate', {method: 'POST'});
                const data = await response.json();
                addLog(data.success ? 'success' : 'error', data.message);
            } catch (e) {
                addLog('error', 'Ошибка активации камеры');
            }
        }
        
        async function deactivateCamera() {
            try {
                const response = await fetch('/api/camera/deactivate', {method: 'POST'});
                const data = await response.json();
                addLog(data.success ? 'success' : 'error', data.message);
            } catch (e) {
                addLog('error', 'Ошибка деактивации камеры');
            }
        }
        
        async function takePhoto() {
            try {
                const response = await fetch('/api/camera/photo', {method: 'POST'});
                const data = await response.json();
                addLog(data.success ? 'success' : 'error', 
                       data.success ? 'Фото: ' + data.filename : data.error);
                if (data.success) loadPhotos();
            } catch (e) {
                addLog('error', 'Ошибка съемки');
            }
        }
        
        async function loadPhotos() {
            try {
                const response = await fetch('/api/camera/photos');
                const data = await response.json();
                const gallery = document.getElementById('photo-gallery');
                
                if (data.photos && data.photos.length > 0) {
                    gallery.innerHTML = data.photos.slice(0, 12).map(p => 
                        `<div class="photo-item">
                            <img src="/photos/${p.filename}" alt="${p.filename}">
                        </div>`
                    ).join('');
                } else {
                    gallery.innerHTML = '<p style="color:#aaa;grid-column:1/-1;text-align:center;">Нет фото</p>';
                }
            } catch (e) {
                addLog('error', 'Ошибка загрузки фото');
            }
        }
        
        async function updateStatus() {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                
                // Обновляем статусы
                const robotStatus = document.getElementById('robot-status');
                const cameraStatus = document.getElementById('camera-status');
                
                robotStatus.className = 'status-indicator ' + 
                    (data.robot_connected ? 'status-online' : 'status-offline');
                cameraStatus.className = 'status-indicator ' + 
                    (data.camera_active ? 'status-online' : 'status-offline');
                
                document.getElementById('robot-state').textContent = data.robot_state || '--';
                document.getElementById('camera-state').textContent = 
                    data.camera_active ? 'Активна' : 'Неактивна';
                document.getElementById('current-action').textContent = 
                    data.current_action || '--';
                
                // Датчики
                if (data.ultrasonic) {
                    document.getElementById('dist-left').textContent = 
                        data.ultrasonic.left > 0 ? data.ultrasonic.left.toFixed(1) : '--';
                    document.getElementById('dist-right').textContent = 
                        data.ultrasonic.right > 0 ? data.ultrasonic.right.toFixed(1) : '--';
                }
                
                document.getElementById('touch-sensor').textContent = 
                    data.touch ? '✓' : '✗';
                
                if (data.mpu6050 && data.mpu6050.acceleration) {
                    const acc = data.mpu6050.acceleration;
                    document.getElementById('mpu-data').textContent = 
                        `${acc[0]?.toFixed(1) || 0}, ${acc[1]?.toFixed(1) || 0}`;
                }
                
            } catch (e) {
                // Тихая ошибка
            }
        }
        
        function addLog(type, message) {
            const log = document.getElementById('log-output');
            const time = new Date().toLocaleTimeString();
            const entry = document.createElement('div');
            entry.className = 'log-entry log-' + type;
            entry.textContent = '[' + time + '] ' + message;
            log.insertBefore(entry, log.firstChild);
            
            // Ограничиваем количество записей
            while (log.children.length > 50) {
                log.removeChild(log.lastChild);
            }
        }
        
        // Клавиатурное управление
        document.addEventListener('keydown', (e) => {
            switch(e.key) {
                case 'ArrowUp': case 'w': case 'W':
                    sendCommand('forward'); break;
                case 'ArrowDown': case 's': case 'S':
                    sendCommand('backward'); break;
                case 'ArrowLeft': case 'a': case 'A':
                    sendCommand('turn_left'); break;
                case 'ArrowRight': case 'd': case 'D':
                    sendCommand('turn_right'); break;
                case ' ':
                    sendCommand('stop_walk'); break;
                case 'Escape':
                    sendCommand('stop'); break;
            }
        });
        
        // Автообновление статуса
        setInterval(updateStatus, 1000);
        updateStatus();
        loadPhotos();
        
        addLog('info', 'Панель управления готова');
        addLog('info', 'Клавиши: WASD/стрелки - движение, Space - стоп');
    </script>
</body>
</html>'''


def create_template():
    """Создает HTML шаблон только если он не существует"""
    template_path = os.path.join('templates', 'index.html')
    if not os.path.exists(template_path):
        with open(template_path, 'w', encoding='utf-8') as f:
            f.write(HTML_TEMPLATE)
        print(f"[WebApp] Шаблон создан: {template_path}")
    else:
        print(f"[WebApp] Шаблон существует: {template_path}")


def start_robot_controller():
    """Запускает контроллер робота в отдельном потоке"""
    global robot_controller
    
    if SmartRobotController is None:
        print("[WebApp] SmartRobotController недоступен")
        return
    
    try:
        # Создаем контроллер без запуска основного цикла
        robot_controller = SmartRobotController(
            use_robot=True,
            use_voice=True,
            log_rl_data=True
        )
        
        # Запускаем потоки датчиков и команд
        threading.Thread(
            target=robot_controller.sensor_monitoring_thread,
            daemon=True
        ).start()
        
        threading.Thread(
            target=robot_controller.command_processing_thread,
            daemon=True
        ).start()
        
        if robot_controller.use_voice:
            threading.Thread(
                target=robot_controller.voice_recognition_thread,
                daemon=True
            ).start()
        
        print("[WebApp] Контроллер робота запущен")
        
    except Exception as e:
        print(f"[WebApp] Ошибка запуска контроллера: {e}")
        robot_controller = None


def main():
    """Основная функция запуска веб-приложения"""
    print("=" * 50)
    print("   SpotMicro Web Control Panel")
    print("=" * 50)
    
    # Создаем HTML шаблон
    create_template()
    
    # Определяем режим работы
    use_robot = "--no-robot" not in sys.argv
    
    if use_robot:
        # Запускаем контроллер робота
        start_robot_controller()
    else:
        print("[WebApp] Запуск без робота (только веб-интерфейс)")
    
    # Запускаем Flask
    host = os.environ.get('HOST', '0.0.0.0')
    port = int(os.environ.get('PORT', 5000))
    
    print(f"[WebApp] Запуск на http://{host}:{port}")
    print("[WebApp] Для остановки нажмите Ctrl+C")
    
    try:
        app.run(host=host, port=port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n[WebApp] Остановка...")
    finally:
        if robot_controller:
            robot_controller.stop()
        camera_manager.deactivate()
        print("[WebApp] Завершено")


if __name__ == "__main__":
    main()
