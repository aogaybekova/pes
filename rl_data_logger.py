#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RL Data Logger - Модуль сохранения данных среды для обучения с подкреплением
Собирает данные с датчиков и состояния робота для последующего обучения
"""

import os
import json
import time
from datetime import datetime
from threading import Lock


class RLDataLogger:
    """Класс для сбора и сохранения данных окружения для обучения с подкреплением"""
    
    def __init__(self, log_dir="rl_data"):
        """
        Инициализация логгера
        
        Args:
            log_dir: Директория для сохранения данных
        """
        self.log_dir = log_dir
        self.lock = Lock()
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.episode_count = 0
        self.step_count = 0
        self.current_episode_data = []
        
        # Создаем директорию для логов
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # Файл сессии
        self.session_file = os.path.join(self.log_dir, f"session_{self.session_id}.jsonl")
        print(f"[RLDataLogger] Сессия создана: {self.session_id}")
        print(f"[RLDataLogger] Файл данных: {self.session_file}")
    
    def log_state(self, state: dict, action: str = None, reward: float = None, 
                  done: bool = False, info: dict = None):
        """
        Записывает состояние окружения
        
        Args:
            state: Словарь с данными состояния (датчики, позиция, etc.)
            action: Выполненное действие
            reward: Награда за действие
            done: Флаг завершения эпизода
            info: Дополнительная информация
        """
        with self.lock:
            timestamp = time.time()
            
            record = {
                "timestamp": timestamp,
                "datetime": datetime.fromtimestamp(timestamp).isoformat(),
                "episode": self.episode_count,
                "step": self.step_count,
                "state": state,
                "action": action,
                "reward": reward,
                "done": done,
                "info": info or {}
            }
            
            self.current_episode_data.append(record)
            self.step_count += 1
            
            # Записываем в файл
            with open(self.session_file, 'a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
            
            if done:
                self._end_episode()
    
    def _end_episode(self):
        """Завершает текущий эпизод"""
        if self.current_episode_data:
            # Сохраняем эпизод отдельным файлом
            episode_file = os.path.join(
                self.log_dir, 
                f"episode_{self.session_id}_{self.episode_count:04d}.json"
            )
            with open(episode_file, 'w', encoding='utf-8') as f:
                json.dump(self.current_episode_data, f, ensure_ascii=False, indent=2)
            
            print(f"[RLDataLogger] Эпизод {self.episode_count} завершен ({len(self.current_episode_data)} шагов)")
        
        self.episode_count += 1
        self.step_count = 0
        self.current_episode_data = []
    
    def new_episode(self):
        """Начинает новый эпизод"""
        if self.current_episode_data:
            self._end_episode()
        print(f"[RLDataLogger] Начат новый эпизод {self.episode_count}")
    
    def create_state_dict(self, 
                          dist_left: float = -1, 
                          dist_right: float = -1,
                          touch_sensor: bool = False,
                          mpu_accel: tuple = (0, 0, 0),
                          mpu_gyro: tuple = (0, 0, 0),
                          robot_state: str = "unknown",
                          walking: bool = False,
                          sitting: bool = False,
                          lying: bool = False,
                          position: dict = None) -> dict:
        """
        Создает словарь состояния для логирования
        
        Args:
            dist_left: Расстояние с левого ультразвукового датчика (см)
            dist_right: Расстояние с правого ультразвукового датчика (см)
            touch_sensor: Состояние датчика касания
            mpu_accel: Данные акселерометра (x, y, z)
            mpu_gyro: Данные гироскопа (x, y, z)
            robot_state: Текущее состояние робота
            walking: Флаг ходьбы
            sitting: Флаг сидения
            lying: Флаг лежания
            position: Позиция робота
        
        Returns:
            Словарь состояния
        """
        return {
            "ultrasonic": {
                "left_cm": dist_left,
                "right_cm": dist_right
            },
            "touch": touch_sensor,
            "mpu6050": {
                "acceleration": {
                    "x": mpu_accel[0],
                    "y": mpu_accel[1],
                    "z": mpu_accel[2]
                },
                "gyroscope": {
                    "x": mpu_gyro[0],
                    "y": mpu_gyro[1],
                    "z": mpu_gyro[2]
                }
            },
            "robot_state": robot_state,
            "flags": {
                "walking": walking,
                "sitting": sitting,
                "lying": lying
            },
            "position": position or {}
        }
    
    def calculate_reward(self, 
                         action: str,
                         obstacle_detected: bool,
                         collision: bool,
                         goal_reached: bool = False) -> float:
        """
        Вычисляет награду для обучения с подкреплением
        
        Args:
            action: Выполненное действие
            obstacle_detected: Было ли обнаружено препятствие
            collision: Произошло ли столкновение
            goal_reached: Достигнута ли цель
        
        Returns:
            Значение награды
        """
        reward = 0.0
        
        # Штраф за столкновение
        if collision:
            reward -= 10.0
        
        # Награда за успешное избежание препятствия
        if obstacle_detected and action in ["turn_left", "turn_right"]:
            reward += 1.0
        
        # Награда за движение вперед без препятствий
        if action == "forward" and not obstacle_detected:
            reward += 0.5
        
        # Большая награда за достижение цели
        if goal_reached:
            reward += 100.0
        
        # Небольшой штраф за стояние на месте
        if action in ["stop", "stop_walk"]:
            reward -= 0.1
        
        return reward
    
    def get_statistics(self) -> dict:
        """Возвращает статистику сессии"""
        return {
            "session_id": self.session_id,
            "episodes": self.episode_count,
            "current_episode_steps": self.step_count,
            "log_file": self.session_file
        }
