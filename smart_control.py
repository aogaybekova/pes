#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smart Control - Умное управление роботом-собакой SpotMicro
Интегрирует голосовое управление, датчики и ориентацию в пространстве
"""

import os
import sys
import json
import time
import threading
from queue import Queue

# Настройки датчиков и GPIO
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("[WARN] RPi.GPIO не доступен - работа в режиме симуляции")
    GPIO_AVAILABLE = False

# Голосовое управление
try:
    import pyaudio
    from vosk import Model, KaldiRecognizer
    VOICE_AVAILABLE = True
except ImportError:
    print("[WARN] Vosk/PyAudio не доступны - голосовое управление отключено")
    VOICE_AVAILABLE = False

# Импорт логгера для RL
from rl_data_logger import RLDataLogger


class SensorManager:
    """Класс для управления всеми датчиками"""
    
    # Конфигурация пинов датчиков расстояния
    TRIG_LEFT = 14   # BCM14
    ECHO_LEFT = 15   # BCM15
    TRIG_RIGHT = 23  # BCM23
    ECHO_RIGHT = 24  # BCM24
    
    # Пин датчика касания
    TOUCH_PIN = 17   # BCM17
    
    # Пороговое расстояние для обнаружения препятствия (см)
    OBSTACLE_THRESHOLD = 30
    
    def __init__(self):
        self.initialized = False
        self.last_dist_left = -1
        self.last_dist_right = -1
        self.last_touch = False
        
        if GPIO_AVAILABLE:
            self._init_gpio()
    
    def _init_gpio(self):
        """Инициализация GPIO для датчиков"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Датчики расстояния
            GPIO.setup(self.TRIG_LEFT, GPIO.OUT)
            GPIO.setup(self.ECHO_LEFT, GPIO.IN)
            GPIO.setup(self.TRIG_RIGHT, GPIO.OUT)
            GPIO.setup(self.ECHO_RIGHT, GPIO.IN)
            
            # Датчик касания
            GPIO.setup(self.TOUCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Инициализация триггеров в низком состоянии
            GPIO.output(self.TRIG_LEFT, False)
            GPIO.output(self.TRIG_RIGHT, False)
            time.sleep(0.1)
            
            self.initialized = True
            print("[SensorManager] GPIO инициализированы успешно")
        except Exception as e:
            print(f"[SensorManager] Ошибка инициализации GPIO: {e}")
            self.initialized = False
    
    def measure_distance(self, trig_pin, echo_pin) -> float:
        """
        Измеряет расстояние с ультразвукового датчика
        
        Returns:
            Расстояние в сантиметрах или -1 при ошибке
        """
        if not GPIO_AVAILABLE or not self.initialized:
            return -1
        
        try:
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(trig_pin, False)
            
            start_time = time.time()
            timeout = start_time + 0.04
            
            while GPIO.input(echo_pin) == 0:
                start_time = time.time()
                if start_time > timeout:
                    return -1
            
            stop_time = time.time()
            timeout = stop_time + 0.04
            
            while GPIO.input(echo_pin) == 1:
                stop_time = time.time()
                if stop_time > timeout:
                    return -1
            
            elapsed = stop_time - start_time
            distance = elapsed * 17150
            
            return distance
        except Exception as e:
            return -1
    
    def measure_filtered(self, trig_pin, echo_pin, samples=3) -> float:
        """Измерение с фильтрацией"""
        readings = []
        for _ in range(samples):
            dist = self.measure_distance(trig_pin, echo_pin)
            if 2.0 <= dist <= 400.0:
                readings.append(dist)
            time.sleep(0.01)
        
        if readings:
            return sum(readings) / len(readings)
        return -1
    
    def get_distances(self) -> tuple:
        """
        Получает расстояния с обоих датчиков
        
        Returns:
            (dist_left, dist_right) в см
        """
        self.last_dist_left = self.measure_filtered(
            self.TRIG_LEFT, self.ECHO_LEFT)
        self.last_dist_right = self.measure_filtered(
            self.TRIG_RIGHT, self.ECHO_RIGHT)
        return (self.last_dist_left, self.last_dist_right)
    
    def get_touch(self) -> bool:
        """Получает состояние датчика касания"""
        if not GPIO_AVAILABLE or not self.initialized:
            return False
        
        try:
            self.last_touch = GPIO.input(self.TOUCH_PIN) == 1
            return self.last_touch
        except Exception:
            return False
    
    def check_obstacle(self) -> tuple:
        """
        Проверяет наличие препятствий
        
        Returns:
            (obstacle_left, obstacle_right) - булевы флаги
        """
        dist_left, dist_right = self.get_distances()
        
        obstacle_left = 0 < dist_left < self.OBSTACLE_THRESHOLD
        obstacle_right = 0 < dist_right < self.OBSTACLE_THRESHOLD
        
        return (obstacle_left, obstacle_right)
    
    def cleanup(self):
        """Освобождает ресурсы GPIO"""
        if GPIO_AVAILABLE and self.initialized:
            try:
                GPIO.cleanup()
                print("[SensorManager] GPIO освобождены")
            except Exception:
                pass


class SmartRobotController:
    """Умный контроллер робота с интеграцией датчиков и голосового управления"""
    
    # Маппинг голосовых команд
    VOICE_COMMANDS = {
        "вперёд": "forward",
        "перёд": "forward",
        "перед": "forward",
        "иди": "forward",
        "иди вперед": "forward",
        "назад": "backward",
        "влево": "turn_left",
        "лево": "turn_left",
        "вправо": "turn_right",
        "право": "turn_right",
        "стоп": "stop_walk",
        "стой": "stop_walk",
        "остановка": "stop_walk",
        "остановись": "stop_walk",
        "дай лапу": "paw_right",
        "правую лапу": "paw_right",
        "левую лапу": "paw_left",
        "опусти лапы": "paw_down",
        "сесть": "sit",
        "сидеть": "sit",
        "лечь": "lie",
        "встать": "stand",
        "стать": "stand",
        "поворот влево": "turn_left",
        "поворот вправо": "turn_right",
        "выход": "quit",
    }
    
    # Настройки голосового распознавания (можно переопределить через переменные окружения)
    MODEL_PATH = os.environ.get("VOSK_MODEL_PATH", "/home/rpi/sr_model/vosk-model-small-ru-0.22/")
    SAMPLE_RATE = 16000
    CHUNK_SIZE = 4096
    
    # Настройки таймингов (в секундах)
    SIT_COMMAND_DELAY = float(os.environ.get("SIT_COMMAND_DELAY", "2.0"))
    TOUCH_DEBOUNCE_TIME = float(os.environ.get("TOUCH_DEBOUNCE_TIME", "1.0"))
    
    def __init__(self, use_robot=True, use_voice=True, log_rl_data=True):
        """
        Инициализация контроллера
        
        Args:
            use_robot: Использовать реального робота
            use_voice: Использовать голосовое управление
            log_rl_data: Логировать данные для RL
        """
        self.running = True
        self.use_robot = use_robot
        self.use_voice = use_voice and VOICE_AVAILABLE
        self.log_rl_data = log_rl_data
        
        # Очередь команд
        self.command_queue = Queue()
        
        # Состояние робота
        self.robot_state = "standing"  # standing, walking, sitting, lying
        self.current_action = "idle"
        self.is_moving = False
        
        # Инициализация датчиков
        self.sensors = SensorManager()
        
        # Инициализация логгера RL
        if self.log_rl_data:
            self.rl_logger = RLDataLogger()
        else:
            self.rl_logger = None
        
        # Инициализация контроллера робота
        self.controller = None
        if self.use_robot:
            try:
                from oop import SpotMicroController
                self.controller = SpotMicroController()
                print("[SmartController] SpotMicroController инициализирован")
            except Exception as e:
                print(f"[SmartController] Не удалось инициализировать робота: {e}")
                self.controller = None
        
        # MPU6050 данные (получаем из контроллера)
        self.mpu_accel = (0, 0, 0)
        self.mpu_gyro = (0, 0, 0)
        
        # Флаги обхода препятствий
        self.obstacle_avoidance_active = False
        self.avoiding_direction = None
        
        print("[SmartController] Инициализация завершена")
    
    def get_mpu_data(self) -> tuple:
        """Получает данные MPU6050"""
        if self.controller and hasattr(self.controller, 'mpu'):
            try:
                accel = self.controller.mpu.acceleration
                gyro = self.controller.mpu.gyro
                self.mpu_accel = accel
                self.mpu_gyro = gyro
                return (accel, gyro)
            except Exception:
                pass
        return ((0, 0, 0), (0, 0, 0))
    
    def send_command(self, command: str):
        """Отправляет команду роботу"""
        if self.controller:
            self.controller.accept_command(command)
            print(f"[SmartController] Команда отправлена: {command}")
        else:
            print(f"[SmartController] (Симуляция) Команда: {command}")
        
        self.current_action = command
        self._update_robot_state(command)
    
    def _update_robot_state(self, command: str):
        """Обновляет состояние робота на основе команды"""
        if command in ["forward", "backward", "left", "right", "turn_left", "turn_right"]:
            self.robot_state = "walking"
            self.is_moving = True
        elif command == "sit":
            self.robot_state = "sitting"
            self.is_moving = False
        elif command == "lie":
            self.robot_state = "lying"
            self.is_moving = False
        elif command in ["stand", "stop", "stop_walk"]:
            self.robot_state = "standing"
            self.is_moving = False
    
    def handle_touch_sensor(self):
        """
        Обрабатывает нажатие датчика касания
        
        Логика:
        - Если робот в движении -> остановить
        - Если стоит -> сесть и дать лапу
        - Если сидит -> просто дать лапу
        """
        touch = self.sensors.get_touch()
        
        if not touch:
            return
        
        print("[SmartController] Датчик касания активирован!")
        
        if self.is_moving:
            print("[SmartController] Робот в движении -> Остановка")
            self.send_command("stop_walk")
        elif self.robot_state == "standing":
            print("[SmartController] Робот стоит -> Сесть и дать лапу")
            self.send_command("sit")
            time.sleep(self.SIT_COMMAND_DELAY)  # Ждем пока сядет
            self.send_command("paw_right")
        elif self.robot_state == "sitting":
            print("[SmartController] Робот сидит -> Дать лапу")
            self.send_command("paw_right")
        else:
            print(f"[SmartController] Касание в состоянии: {self.robot_state}")
    
    def handle_obstacle_avoidance(self, command: str) -> bool:
        """
        Обрабатывает избежание препятствий при движении вперед
        
        Args:
            command: Запрошенная команда движения
        
        Returns:
            True если команда была обработана (с обходом), False если можно выполнять напрямую
        """
        if command != "forward":
            self.obstacle_avoidance_active = False
            return False
        
        obstacle_left, obstacle_right = self.sensors.check_obstacle()
        
        if not obstacle_left and not obstacle_right:
            # Путь свободен
            if self.obstacle_avoidance_active:
                print("[SmartController] Путь свободен! Продолжаем вперед")
                self.obstacle_avoidance_active = False
            return False
        
        # Есть препятствие
        print(f"[SmartController] Препятствие! Лево: {obstacle_left}, Право: {obstacle_right}")
        self.obstacle_avoidance_active = True
        
        dist_left, dist_right = self.sensors.last_dist_left, self.sensors.last_dist_right
        
        # Выбираем направление поворота
        # Поворачиваем в сторону, где расстояние больше
        if dist_left > dist_right:
            turn_direction = "turn_left"
            print(f"[SmartController] Поворот влево (L:{dist_left:.1f}см > R:{dist_right:.1f}см)")
        else:
            turn_direction = "turn_right"
            print(f"[SmartController] Поворот вправо (R:{dist_right:.1f}см > L:{dist_left:.1f}см)")
        
        self.avoiding_direction = turn_direction
        self.send_command(turn_direction)
        
        return True
    
    def process_voice_command(self, text: str):
        """Обрабатывает распознанную голосовую команду"""
        text = text.strip().lower()
        print(f"[SmartController] Распознано: '{text}'")
        
        for phrase, cmd in self.VOICE_COMMANDS.items():
            if phrase in text:
                # Добавляем команду в очередь
                self.command_queue.put(cmd)
                print(f"[SmartController] Голосовая команда: {cmd}")
                return
        
        print(f"[SmartController] Неизвестная команда: '{text}'")
    
    def voice_recognition_thread(self):
        """Поток голосового распознавания"""
        if not VOICE_AVAILABLE:
            print("[SmartController] Голосовое управление недоступно")
            return
        
        if not os.path.exists(self.MODEL_PATH):
            print(f"[SmartController] Модель Vosk не найдена: {self.MODEL_PATH}")
            return
        
        try:
            print("[SmartController] Загрузка модели Vosk...")
            model = Model(self.MODEL_PATH)
            recognizer = KaldiRecognizer(model, self.SAMPLE_RATE)
            
            p = pyaudio.PyAudio()
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.SAMPLE_RATE,
                input=True,
                frames_per_buffer=self.CHUNK_SIZE
            )
            
            print("[SmartController] Голосовое управление активно!")
            
            while self.running:
                try:
                    data = stream.read(self.CHUNK_SIZE, exception_on_overflow=False)
                    if recognizer.AcceptWaveform(data):
                        result = json.loads(recognizer.Result())
                        text = result.get('text', '').strip()
                        if text:
                            self.process_voice_command(text)
                except Exception as e:
                    print(f"[SmartController] Ошибка распознавания: {e}")
                    time.sleep(0.1)
            
            stream.stop_stream()
            stream.close()
            p.terminate()
            
        except Exception as e:
            print(f"[SmartController] Ошибка голосового потока: {e}")
    
    def sensor_monitoring_thread(self):
        """Поток мониторинга датчиков"""
        print("[SmartController] Мониторинг датчиков запущен")
        
        last_touch_time = 0
        
        while self.running:
            try:
                # Проверяем датчик касания
                current_time = time.time()
                if current_time - last_touch_time > self.TOUCH_DEBOUNCE_TIME:
                    if self.sensors.get_touch():
                        self.handle_touch_sensor()
                        last_touch_time = current_time
                
                # Если активен обход препятствий, продолжаем проверять
                if self.obstacle_avoidance_active:
                    obstacle_left, obstacle_right = self.sensors.check_obstacle()
                    if not obstacle_left and not obstacle_right:
                        print("[SmartController] Препятствие пройдено!")
                        self.obstacle_avoidance_active = False
                        # Продолжаем движение вперед
                        self.send_command("forward")
                
                # Логируем данные для RL
                if self.rl_logger:
                    self._log_rl_state()
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"[SmartController] Ошибка мониторинга: {e}")
                time.sleep(0.5)
    
    def _log_rl_state(self):
        """Логирует текущее состояние для обучения"""
        if not self.rl_logger:
            return
        
        dist_left, dist_right = self.sensors.last_dist_left, self.sensors.last_dist_right
        touch = self.sensors.last_touch
        
        accel, gyro = self.get_mpu_data()
        
        state = self.rl_logger.create_state_dict(
            dist_left=dist_left,
            dist_right=dist_right,
            touch_sensor=touch,
            mpu_accel=accel,
            mpu_gyro=gyro,
            robot_state=self.robot_state,
            walking=self.robot_state == "walking",
            sitting=self.robot_state == "sitting",
            lying=self.robot_state == "lying"
        )
        
        # Вычисляем награду
        obstacle_detected = self.obstacle_avoidance_active
        collision = touch and self.is_moving
        reward = self.rl_logger.calculate_reward(
            self.current_action,
            obstacle_detected,
            collision
        )
        
        self.rl_logger.log_state(
            state=state,
            action=self.current_action,
            reward=reward,
            done=False
        )
    
    def command_processing_thread(self):
        """Поток обработки команд"""
        print("[SmartController] Обработка команд запущена")
        
        while self.running:
            try:
                # Получаем команду из очереди с таймаутом
                try:
                    command = self.command_queue.get(timeout=0.5)
                except Exception:
                    continue
                
                if command == "quit":
                    self.running = False
                    break
                
                # Проверяем препятствия для команды движения вперед
                if command == "forward":
                    if self.handle_obstacle_avoidance(command):
                        continue  # Команда обработана обходом препятствия
                
                # Отправляем команду роботу
                self.send_command(command)
                
            except Exception as e:
                print(f"[SmartController] Ошибка обработки команды: {e}")
    
    def add_command(self, command: str):
        """Добавляет команду в очередь (для внешнего использования)"""
        self.command_queue.put(command)
    
    def get_sensor_data(self) -> dict:
        """Возвращает текущие данные датчиков"""
        dist_left, dist_right = self.sensors.get_distances()
        accel, gyro = self.get_mpu_data()
        
        return {
            "ultrasonic": {
                "left": dist_left,
                "right": dist_right
            },
            "touch": self.sensors.last_touch,
            "mpu6050": {
                "acceleration": accel,
                "gyroscope": gyro
            },
            "robot_state": self.robot_state,
            "current_action": self.current_action,
            "is_moving": self.is_moving,
            "obstacle_avoidance": self.obstacle_avoidance_active
        }
    
    def start(self):
        """Запускает все потоки контроллера"""
        print("[SmartController] Запуск системы...")
        
        threads = []
        
        # Поток голосового управления
        if self.use_voice:
            voice_thread = threading.Thread(
                target=self.voice_recognition_thread,
                daemon=True
            )
            voice_thread.start()
            threads.append(voice_thread)
        
        # Поток мониторинга датчиков
        sensor_thread = threading.Thread(
            target=self.sensor_monitoring_thread,
            daemon=True
        )
        sensor_thread.start()
        threads.append(sensor_thread)
        
        # Поток обработки команд
        command_thread = threading.Thread(
            target=self.command_processing_thread,
            daemon=True
        )
        command_thread.start()
        threads.append(command_thread)
        
        # Запускаем основной цикл робота
        if self.controller:
            print("[SmartController] Запуск основного контроллера робота...")
            try:
                self.controller.start()
            except KeyboardInterrupt:
                print("\n[SmartController] Прерывание пользователем")
            except Exception as e:
                print(f"[SmartController] Ошибка контроллера: {e}")
        else:
            # Режим симуляции - просто ждем
            print("[SmartController] Работа в режиме симуляции")
            try:
                while self.running:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n[SmartController] Прерывание пользователем")
        
        self.stop()
    
    def stop(self):
        """Останавливает контроллер"""
        print("[SmartController] Остановка...")
        self.running = False
        
        # Завершаем эпизод RL
        if self.rl_logger:
            self.rl_logger.new_episode()
            stats = self.rl_logger.get_statistics()
            print(f"[SmartController] RL статистика: {stats}")
        
        # Освобождаем GPIO
        self.sensors.cleanup()
        
        print("[SmartController] Остановлен")


def main():
    """Основная функция запуска"""
    print("=" * 50)
    print("   SpotMicro Smart Control System")
    print("=" * 50)
    
    # Параметры запуска
    use_robot = True
    use_voice = True
    log_rl_data = True
    
    # Проверяем аргументы командной строки
    if "--no-robot" in sys.argv:
        use_robot = False
        print("Режим без робота")
    if "--no-voice" in sys.argv:
        use_voice = False
        print("Режим без голосового управления")
    if "--no-log" in sys.argv:
        log_rl_data = False
        print("Логирование RL отключено")
    
    # Создаем и запускаем контроллер
    controller = SmartRobotController(
        use_robot=use_robot,
        use_voice=use_voice,
        log_rl_data=log_rl_data
    )
    
    controller.start()


if __name__ == "__main__":
    main()
