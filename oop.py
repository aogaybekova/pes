#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import queue
import numpy as np
import copy
from math import pi, sin, cos, atan, sqrt
from time import sleep, time

# External dependencies
import pygame
import board
import busio
import adafruit_pca9685
import adafruit_mpu6050
#import mpu6050 #import mpu6050
from adafruit_servokit import ServoKit

# SpotMicro-specific imports
import Spotmicro_lib_020
import Spotmicro_Gravity_Center_lib_007
import Spotmicro_Animate_lib_009

print("=== Script started ===")
# ==================== CLASS CONTROLLER ====================

class SpotMicroController:
    def __init__(self):
        print("=== Controller __init__ started ===")

        # == Core robot modules and helpers
        self.Spot = Spotmicro_lib_020.Spot()
        self.SpotCG = Spotmicro_Gravity_Center_lib_007.SpotCG()
        self.anim = Spotmicro_Animate_lib_009.SpotAnim()

        # == PWM & hardware
        self.i2c= busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.mpu = adafruit_mpu6050.MPU6050(self.i2c)
        self.pca.frequency = 50
        self.kit0 = ServoKit(address=0x40, channels=16)
        self.kit1 = ServoKit(address=0x41, channels=16)
        for i in range(0, 6):
            self.kit0.servo[self.Spot.servo_table[i]].set_pulse_width_range(500, 2500)
        for i in range(6, 12):
            self.kit1.servo[self.Spot.servo_table[i]].set_pulse_width_range(500, 2500)
        #self.mpu = mpu6050.MPU6050(0x68, bus=1)

        # == Display
        # Отключаем аппаратное ускорение для избежания проблем с OpenGL
        os.environ['SDL_VIDEO_X11_NOWINDOWMOVE'] = '1'
        pygame.init()
        print("=== Controller __init__ END ===")

        # Исправляем инициализацию дисплея
        self.screen = pygame.display.set_mode((600, 600))
        pygame.display.set_caption("SPOTMICRO CONSOLE CONTROL")

        # Цвета для интерфейса
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)

        self.smallfont = pygame.font.SysFont('Corbel', 20)
        self.text_animon = self.smallfont.render('Anim On', True, self.BLACK)
        self.text_animoff = self.smallfont.render('Anim Off', True, self.WHITE)
        self.text_moveon = self.smallfont.render('Move On', True, self.BLACK)
        self.text_moveoff = self.smallfont.render('Move Off', True, self.WHITE)

        # == Command/state management
        self.command_queue = []
        self.console_lock = threading.Lock()
        self.current_command_in_progress = None  # Track currently executing command
        self.command_queue_set = set()  # Prevent duplicate commands in queue

        # == Main state
        self.anim_flag = False
        self.move_flag = True
        self.walking = False
        self.trot = False
        self.sitting = False
        self.lying = False
        self.twisting = False
        self.shifting = False
        self.pawing = False
        self.recovering = False
        self.peeing = False
        self.stop = False
        self.Free = True
        self.lock = False
        self.move = True
        self.lockmouse = False
        self.mouseclick = False
        self.IMU_Comp = False
        
        # Animation state tracking
        self.in_turn_animation = False  # Flag to track if currently executing turn
        self.turn_start_time = 0  # Track when turn started


        # == Walking params
        self.b_height = 220
        self.x_offset = 0
        self.track2, self.track4 = 58, 58  # Ширина постановки ног (Y-координата)
        self.h_amp2, self.h_amp4 = 100, 60 #80  # Горизонтальная амплитуда движения ног (X-направление)
        self.v_amp2, self.v_amp4 = 20, 45 #25 # Вертикальная амплитуда подъема ног (Z-направление)
        #длинa шага
        self.stepl2, self.stepl4 = 0.16, 0.125 #was 0.2#08#0.125 # СКОРОСТЬ ПЕРЕМЕЩЕНИЯ ТЕЛА
        self.tstep2, self.tstep4 = self.stepl2 / 8, 0.015 #0.8 #0.012 #6666666666 # время шага
        self.track = self.track4
        self.h_amp = self.h_amp4
        self.v_amp = self.v_amp4
        self.stepl = self.stepl4
        self.tstep = self.tstep4
        self.height = self.b_height
        self.prev_pos = None
        self.animation_smoothing = 0.5  # Коэффициент сглаживания (0-1)
        self.target_speed = 0
        self.t = 0
        self.transtep = 0.0125
        self.trans = 0
        self.tstop = 1000
        self.current_movement_command = "stop"
        self.current_servo_angles = [0.0] * 12
        self.speed_smoothing = 0.3
        # Параметры стабилизации
        self.cg_stabilization_enabled = True
        self.imu_stabilization_enabled = True  # Добавляем отдельный флаг для IMU

        # Более консервативные параметры стабилизации
        self.max_cg_offset_x = 10  # Уменьшено с 15
        self.max_cg_offset_y = 8   # Уменьшено с 10
        self.max_leg_adjustment = 20  # Максимальная корректировка ноги (мм)

        # Плавная фильтрация
        self.body_filter_alpha = 0.2  # Более сильное сглаживание
        self.leg_filter_alpha = 0.3   # Фильтр для ног

        # Инициализация фильтров
        self.filtered_body_x = 0
        self.filtered_body_y = 0
        self.filtered_body_z = self.b_height
        self.filtered_leg_positions = [0.0] * 12


        # == Kinematics/pose
        self.xtlf = self.xtrf = self.xtrr = self.xtlr = 14
        self.ytlf = self.ztlf = self.ytrf = self.ytrr = self.ytlr = self.ztlr = 0
        self.ztrf = 3
        self.ztrr = 0
        self.stance = [True] * 4
        self.cw = 1
        self.walking_speed = 0
        self.walking_direction = 0
        self.steering = 1e6
        self.temp_start_pos = [0, 0, 0, 0, 0, 0]
        self.pos_sit_init = None
        self.joypal = -1
        self.joypar = -1
        self.Bat = 0

        # == filters
        self.anglex_buff = np.zeros(10)
        self.angley_buff = np.zeros(10)
        self.zeroangle_x = 0.0338
        self.zeroangle_y = -0.0594
        self.iangle = 0
        self.angle_count = 1
        self.Tcomp = 0.02
        self.angle = np.zeros(2)
        self.Angle = np.zeros(2)
        self.Angle_old = np.zeros(2)
        self.Integral_Angle = [0, 0]

        # == Positions (body, legs)
        self.pos_init = [-self.x_offset, self.track4, -self.b_height, -self.x_offset, -self.track4, -self.b_height,
                         -self.x_offset, -self.track4, -self.b_height, -self.x_offset, self.track4, -self.b_height]
        self.x_spot = [0, self.x_offset, self.Spot.xlf, self.Spot.xrf, self.Spot.xrr, self.Spot.xlr, 0, 0, 0, 0]
        self.y_spot = [0, 0, self.Spot.ylf + self.track4, self.Spot.yrf - self.track4, self.Spot.yrr - self.track4,
                       self.Spot.ylr + self.track4, 0, 0, 0, 0]
        self.z_spot = [0, self.b_height, 0, 0, 0, 0, 0, 0, 0, 0]
        self.theta_spot = [0, 0, 0, 0, 0, 0]
        self.pos = self.pos_init + [self.theta_spot, self.x_spot, self.y_spot, self.z_spot]
        self.tstart = 1

        # == GUI and control loop
        self.continuer = True
        self.clock = pygame.time.Clock()
        self.current_action = "Ready"

    # ==================== PRIMARY INTERFACE ====================

    def accept_command(self, command):
        """Accept a command with duplicate prevention"""
        with self.console_lock:
            # Prevent duplicate commands in queue
            if command not in self.command_queue_set:
                self.command_queue.append(command)
                self.command_queue_set.add(command)
                print(f"[QUEUE] Command added: {command} (queue size: {len(self.command_queue)})")

    def start_console_thread(self):
        print('=== Console thread started ===')

        th = threading.Thread(target=self.console_input_thread, daemon=True)
        th.start()

    def start(self):
        self.start_console_thread()
        self.main_loop()

    # ==================== CONSOLE HANDLER ====================

    def console_input_thread(self):
        print("\n" + "=" * 50)
        print("         SpotMicro Console Control")
        print("=" * 50)
        print("BASIC COMMANDS: walk, sit, lie, twist, pee, stop, stand, stop_walk")
        print("MOVEMENT: forward, backward, left, right, turn_left, turn_right")
        print("PAWING: paw_left, paw_right, paw_down (when sitting)")
        print("SETTINGS: move, anim, trot, imu, quit")
        print("=" * 50)
        print("Enter commands below:")
        while self.continuer:
            try:
                command = input("> ").strip().lower()
                if command:
                    with self.console_lock:
                        self.command_queue.append(command)
                        print(f"Command queued: {command}")
            except (EOFError, KeyboardInterrupt):
                with self.console_lock:
                    self.command_queue.append("quit")
                break
            except Exception as e:
                print(f"Console input error: {e}")
                sleep(0.1)

    def process_console_commands(self):
        def ensure_walking_mode():
            if not self.walking and self.Free:
                self.walking = True
                self.Free = False
                self.stop = False
                self.t = 0
                self.lock = True
                self.walking_speed = 0
                self.current_action = "Walking mode started"
                print("[STATE] === WALKING STARTED ===")

        with self.console_lock:
            while self.command_queue:
                command = self.command_queue.pop(0)
                # Remove from set to allow re-queuing after execution
                self.command_queue_set.discard(command)
                
                print(f"[EXEC] Executing: {command}")
                self.current_command_in_progress = command

                if command == "quit":
                    self.continuer = False
                    self.current_action = "Shutting down"

                elif command == "forward":
                    # Transition from turn to forward - complete turn animation first
                    if self.in_turn_animation:
                        print("[TRANSITION] Completing turn before moving forward...")
                        self.in_turn_animation = False
                    ensure_walking_mode()
                    self.current_movement_command = "forward"
                    self.target_speed = 100
                    self.current_action = "Moving forward"
                    print("[MOVEMENT] Moving forward")

                elif command == "backward":
                    if self.in_turn_animation:
                        print("[TRANSITION] Completing turn before moving backward...")
                        self.in_turn_animation = False
                    ensure_walking_mode()
                    self.current_movement_command = "backward"
                    self.target_speed = 100
                    self.current_action = "Moving backward"
                    print("[MOVEMENT] Moving backward")

                elif command == "left":
                    if self.in_turn_animation:
                        print("[TRANSITION] Completing turn before strafing left...")
                        self.in_turn_animation = False
                    ensure_walking_mode()
                    self.current_movement_command = "left"
                    self.current_action = "Moving left"
                    print("[MOVEMENT] Moving left")

                elif command == "right":
                    if self.in_turn_animation:
                        print("[TRANSITION] Completing turn before strafing right...")
                        self.in_turn_animation = False
                    ensure_walking_mode()
                    self.current_movement_command = "right"
                    self.current_action = "Moving right"
                    print("[MOVEMENT] Moving right")

                elif command == "turn_left":
                    ensure_walking_mode()
                    self.current_movement_command = "turn_left"
                    self.current_action = "Turning left"
                    self.in_turn_animation = True
                    self.turn_start_time = self.t
                    print("[TURN] === TURN LEFT STARTED ===")

                elif command == "turn_right":
                    ensure_walking_mode()
                    self.current_movement_command = "turn_right"
                    self.current_action = "Turning right"
                    self.in_turn_animation = True
                    self.turn_start_time = self.t
                    print("[TURN] === TURN RIGHT STARTED ===")

                elif command == "stop_walk":
                    if self.walking:
                        # Check if we're in the middle of a turn animation
                        if self.in_turn_animation:
                            print("[STOP_WALK] Waiting for turn to complete before stopping...")
                            # Re-queue the stop_walk command to execute after turn completes
                            if "stop_walk" not in self.command_queue_set:
                                self.command_queue.append("stop_walk")
                                self.command_queue_set.add("stop_walk")
                            continue
                        
                        self.stop = True
                        self.lock = True
                        self.tstop = int(self.t)
                        self.current_movement_command = "stop"
                        self.current_action = "Stopping walk..."
                        self.in_turn_animation = False
                        print("[STOP_WALK] === STOPPING WALK SEQUENCE INITIATED ===")
                        print(f"[STOP_WALK] Stop triggered at t={self.t}, tstop={self.tstop}")
                    else:
                        print("[STOP_WALK] Not in walking mode.")

                elif command == "walk":
                    if self.walking and not self.stop and not self.lock:
                        self.stop = True
                        self.lock = True
                        self.current_movement_command = "stop"
                        self.current_action = "Stopping walk and exiting mode"
                    elif not self.walking and self.Free:
                        # ????? ?????????????????, ???? ?????? "????????" ????? ??????:
                        # ensure_walking_mode()
                        self.current_movement_command = "stop"

                elif command == "stab_test":
                    # Тестовый режим стабилизации
                    print("=== РЕЖИМ ТЕСТИРОВАНИЯ СТАБИЛИЗАЦИИ ===")
                    print(f"CG стабилизация: {'ВКЛ' if self.cg_stabilization_enabled else 'ВЫКЛ'}")
                    print(f"IMU стабилизация: {'ВКЛ' if self.imu_stabilization_enabled else 'ВЫКЛ'}")
                    print(f"Смещение ЦТ: X={self.CGabs[0]-self.x_spot[1]:.1f}, Y={self.CGabs[1]-self.y_spot[1]:.1f}")

                elif command == "adjust_params":
                    # Настройка параметров стабилизации
                    print("Настройка параметров стабилизации:")
                    print("1. Увеличить жесткость")
                    print("2. Уменьшить жесткость")
                    print("3. Включить/выключить фильтрацию")
                    choice = input("Выберите опцию: ")

                    if choice == "1":
                        self.body_filter_alpha = min(self.body_filter_alpha + 0.1, 0.5)
                        print(f"Жесткость увеличена: alpha={self.body_filter_alpha}")
                    elif choice == "2":
                        self.body_filter_alpha = max(self.body_filter_alpha - 0.1, 0.05)
                        print(f"Жесткость уменьшена: alpha={self.body_filter_alpha}")
                    elif choice == "3":
                        self.cg_stabilization_enabled = not self.cg_stabilization_enabled
                        print(f"CG стабилизация: {'ВКЛ' if self.cg_stabilization_enabled else 'ВЫКЛ'}")

                elif command == "stop":
                    # ????? ? ??????????? ????
                    self.pos_init = [-self.x_offset, self.track, -self.b_height, -self.x_offset, -self.track,
                                     -self.b_height,
                                     -self.x_offset, -self.track, -self.b_height, -self.x_offset, self.track,
                                     -self.b_height]
                    self.pos[0:12] = self.pos_init
                    self.recovering = True
                    self.walking = False
                    self.sitting = False
                    self.lying = False
                    self.twisting = False
                    self.shifting = False
                    self.pawing = False
                    self.stop = False
                    self.Free = True
                    self.walking_speed = 0.0
                    self.current_movement_command = "stop"
                    self.joypal = -1
                    self.joypar = -1
                    self.current_action = "EMERGENCY STOP - All motions stopped"
                    print("*** EMERGENCY STOP ***")

                elif command == "sit":
                    # Check if animation is in progress
                    if self.walking and not self.Free:
                        print("[SIT] Waiting for walking animation to complete before transition")
                        # Re-queue sit command to execute after walking completes
                        if "sit" not in self.command_queue_set:
                            self.command_queue.append("sit")
                            self.command_queue_set.add("sit")
                        continue
                    
                    if not self.sitting and self.Free:
                        self.sitting = True
                        self.stop = False
                        self.Free = False
                        self.t = 0
                        self.lock = True
                        self.pawing = False
                        self.joypal = -1
                        self.joypar = -1
                        self.current_action = "Sitting down"
                        print("[STATE] === SITTING DOWN ===")
                    elif self.sitting and not self.stop and not self.lock:
                        self.stop = True
                        self.lock = True
                        self.pawing = False
                        self.current_action = "Standing up"
                        print("[STATE] === STANDING UP FROM SIT ===")

                elif command == "stand":
                    if (self.sitting or self.lying) and not self.stop and not self.lock:
                        self.stop = True
                        self.lock = True
                        self.pawing = False
                        self.current_action = "Standing up"
                        print("[STATE] === STANDING UP ===")
                    elif self.Free:
                        print("[STATE] Already standing.")


                elif command == "lie":
                    if not self.lying and self.Free:
                        self.lying = True
                        self.stop = False
                        self.Free = False
                        self.t = 0
                        self.lock = True
                        self.current_action = "Lying down"
                    elif self.lying and not self.stop and not self.lock:
                        self.stop = True
                        self.lock = True
                        self.current_action = "Standing up"

                elif command == "twist":
                    if not self.twisting and self.Free:
                        self.twisting = True
                        self.Free = False
                        self.t = 0
                        self.lock = True
                        self.current_action = "Twisting"

                elif command == "stab_off":
                    self.cg_stabilization_enabled = False
                    self.imu_stabilization_enabled = False
                    print("Стабилизация полностью отключена")

                elif command == "stab_on":
                    self.cg_stabilization_enabled = True
                    self.imu_stabilization_enabled = True
                    print("Стабилизация включена")

                elif command == "paw_left":
                    if self.sitting and self.t >= 0.99:
                        self.joypal = min(1.0, self.joypal + 0.2)
                        self.current_action = f"Left paw raised to {self.joypal:.1f}"
                        print(f"Left paw position: {self.joypal:.1f}")

                elif command == "paw_right":
                    if self.sitting and self.t >= 0.99:
                        self.joypar = min(1.0, self.joypar + 0.2)
                        self.current_action = f"Right paw raised to {self.joypar:.1f}"
                        print(f"Right paw position: {self.joypar:.1f}")

                elif command == "paw_down":
                    if self.sitting and self.t >= 0.99:
                        self.joypal = max(-1.0, self.joypal - 0.2)
                        self.joypar = max(-1.0, self.joypar - 0.2)
                        self.current_action = "Paws lowered"
                        print(f"Paw positions - Left: {self.joypal:.1f}, Right: {self.joypar:.1f}")

                elif command == "pee":
                    if not self.shifting and self.Free:
                        self.shifting = True
                        self.stop = False
                        self.Free = False
                        self.t = 0
                        self.lock = True
                        self.current_action = "Peeing started"
                    elif self.shifting and not self.stop and not self.lock:
                        self.stop = True
                        self.lock = True
                        self.current_action = "Stopping pee"

                elif command == "move":
                    self.move_flag = not self.move_flag
                    state = "ON" if self.move_flag else "OFF"
                    print(f"Servo movement: {state}")
                    self.current_action = f"Servo movement {state}"

                elif command == "anim":
                    self.anim_flag = not self.anim_flag
                    state = "ON" if self.anim_flag else "OFF"
                    print(f"Animation: {state}")
                    self.current_action = f"Animation {state}"

                elif command == "trot":
                    if not self.trot:
                        self.trot = True
                        self.stepl = self.stepl2
                        self.h_amp = self.h_amp2
                        self.v_amp = self.v_amp2
                        self.track = self.track2
                        self.tstep = self.tstep2
                        self.trans = 1
                    else:
                        self.trot = False
                        self.stepl = self.stepl4
                        self.h_amp = self.h_amp4
                        self.v_amp = self.v_amp4
                        self.track = self.track4
                        self.tstep = self.tstep4
                        self.trans = 0
                    state = "ON" if self.trot else "OFF"
                    print(f"Trot mode: {state}")
                    self.current_action = f"Trot mode {state}"

                elif command == "imu":
                    self.IMU_Comp = not self.IMU_Comp
                    self.Integral_Angle = [0, 0]
                    state = "ON" if self.IMU_Comp else "OFF"
                    print(f"IMU compensation: {state}")
                    self.current_action = f"IMU compensation {state}"

                else:
                    print(f"Unknown command: {command}")
                    print(
                        "Available commands: walk, sit, lie, twist, pee, stop, forward, backward, left, right, turn_left, turn_right, paw_left, paw_right, paw_down, move, anim, trot, imu, quit")


    # ==================== MAIN ROBOTIC CYCLE ====================

    def main_loop(self):
        print("Starting main loop... Use console to control the robot.")
        while self.continuer:
            self.clock.tick(60) #self.clock.tick(30)

            if not hasattr(self, 'frame_counter'):
                self.frame_counter = 0
            self.frame_counter += 1

            #if self.frame_counter % 100 == 0 and self.walking:
                #print(f"=== Отладочная информация (кадр {self.frame_counter}) ===")
                #print(f"Позиции ног (Z): LF={self.pos[2]:.1f}, RF={self.pos[5]:.1f}, RR={self.pos[8]:.1f}, LR={self.pos[11]:.1f}")
                #print(f"Углы IMU: roll={self.Angle[0]*180/pi:.1f}°, pitch={self.Angle[1]*180/pi:.1f}°")
                #if hasattr(self, 'CGabs'):
                #    print(f"Центр тяжести: X={self.CGabs[0]:.1f}, Y={self.CGabs[1]:.1f}")

            # Очищаем экран в начале каждого кадра
            self.screen.fill(self.BLACK)

            # Отображаем информацию о состоянии
            status_font = pygame.font.SysFont('Corbel', 24)
            status_text = status_font.render(f"State: {self.current_action}", True, self.WHITE)
            self.screen.blit(status_text, (10, 10))

            # Отображаем информацию о режимах
            anim_text = status_font.render(f"Animation: {'ON' if self.anim_flag else 'OFF'}", True, self.WHITE)
            move_text = status_font.render(f"Movement: {'ON' if self.move_flag else 'OFF'}", True, self.WHITE)
            self.screen.blit(anim_text, (10, 40))
            self.screen.blit(move_text, (10, 70))

            self.process_console_commands()

            #плавное изменение скорости
            if self.walking and hasattr(self, 'target_speed'):
                # Плавное достижение целевой скорости
                speed_diff = self.target_speed - self.walking_speed
                if abs(speed_diff) > 1:
                    self.walking_speed += speed_diff * self.speed_smoothing #0.1  # Плавное изменение

            # ---- IMU processing ----
            self.angle = self.comp_filter(self.angle, self.tstep, self.Tcomp)
            self.anglex_buff[self.iangle] = self.angle[0] + self.zeroangle_x
            self.angley_buff[self.iangle] = self.angle[1] + self.zeroangle_y
            self.Angle_old = self.Angle
            self.Angle = [np.mean(self.anglex_buff), np.mean(self.angley_buff)]
            self.iangle = (self.iangle + 1) % self.angle_count

            # ---- pygame event ----
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.continuer = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.mouseclick = True
                else:
                    self.mouseclick = False

            if (not self.walking and not self.sitting and not self.lying and not self.twisting
                and not self.shifting and not self.pawing) and self.lock:
                self.lock = False

            # ---- Movement logic for different states ----
            if self.walking:
                self.t = self.t + self.tstep
                self.trec = int(self.t) + 1

                # Process movement commands
                self.DIR_FORWARD = pi / 2
                self.DIR_BACKWARD = 3 * pi / 2
                self.DIR_LEFT = pi
                self.DIR_RIGHT = 0

                if self.current_movement_command == "forward":
                    #self.walking_speed = 100  # 0.5
                    self.walking_direction = self.DIR_FORWARD
                    self.steering = 1e6
                    self.cw = 1

                elif self.current_movement_command == "backward":
                    self.walking_speed = 100
                    self.walking_direction = self.DIR_BACKWARD
                    self.steering = 1e6
                    self.cw = 1

                elif self.current_movement_command == "left":
                    self.walking_speed = 5  # 50
                    self.walking_direction = self.DIR_LEFT
                    self.steering = 1e6
                    self.cw = 1

                elif self.current_movement_command == "right":
                    self.walking_speed = 5  # 50
                    self.walking_direction = self.DIR_RIGHT
                    self.steering = 1e6
                    self.cw = 1

                elif self.current_movement_command == "turn_left":
                    self.walking_speed = 100
                    self.walking_direction = 0
                    self.steering = 80  # 1000
                    self.cw = 1
                    
                    # Check if turn animation has completed a full cycle
                    if self.in_turn_animation and (self.t - self.turn_start_time) >= 1.0:
                        self.in_turn_animation = False
                        print("[TURN] === TURN LEFT COMPLETED ===")

                elif self.current_movement_command == "turn_right":
                    self.walking_speed = 100
                    self.walking_direction = 0
                    self.steering = 80  # 1000
                    self.cw = -1
                    
                    # Check if turn animation has completed a full cycle
                    if self.in_turn_animation and (self.t - self.turn_start_time) >= 1.0:
                        self.in_turn_animation = False
                        print("[TURN] === TURN RIGHT COMPLETED ===")

                elif self.current_movement_command == "stop":
                    self.walking_speed = 0.0
                    self.walking_direction = 0
                    self.steering = 1e6
                    self.cw = 1

                # Execute walking command
                self.pos = self.Spot.start_walk_stop(self.track, self.x_offset, self.steering, self.walking_direction, self.cw,
                                           self.walking_speed, self.v_amp, self.height, self.stepl, self.t, self.tstep,
                                           self.theta_spot, self.x_spot, self.y_spot, self.z_spot, 3 + self.trans)
                # В main_loop после вычисления self.pos:
                if self.prev_pos is None:
                    self.prev_pos = copy.deepcopy(self.pos)
                else:
                    # Плавная интерполяция между кадрами
                    for i in range(len(self.pos)):
                        if isinstance(self.pos[i], list):
                            for j in range(len(self.pos[i])):
                                self.pos[i][j] = (self.prev_pos[i][j] * (1 - self.animation_smoothing) +
                                                 self.pos[i][j] * self.animation_smoothing)
                        else:
                            self.pos[i] = (self.prev_pos[i] * (1 - self.animation_smoothing) +
                                          self.pos[i] * self.animation_smoothing)
                    self.prev_pos = copy.deepcopy(self.pos)
                self.theta_spot = self.pos[12]
                self.x_spot = self.pos[13]
                self.y_spot = self.pos[14]
                self.z_spot = self.pos[15]

                # ФИЛЬТРАЦИЯ ПОЗИЦИИ КОРПУСА (только при ходьбе)
                if self.walking:
                    if not hasattr(self, 'filtered_body_x'):
                        # Инициализируем фильтры текущими значениями
                        self.filtered_body_x = self.pos[13][1]
                        self.filtered_body_y = self.pos[14][1]
                        self.filtered_body_z = self.pos[15][1]

                    self.filtered_body_x = (1 - self.body_filter_alpha) * self.filtered_body_x + self.body_filter_alpha * self.pos[13][1]
                    self.filtered_body_y = (1 - self.body_filter_alpha) * self.filtered_body_y + self.body_filter_alpha * self.pos[14][1]
                    self.filtered_body_z = (1 - self.body_filter_alpha) * self.filtered_body_z + self.body_filter_alpha * self.pos[15][1]

                    self.pos[13][1] = self.filtered_body_x
                    self.pos[14][1] = self.filtered_body_y
                    self.pos[15][1] = self.filtered_body_z
                    try:
                        self.CG = self.SpotCG.CG_calculation(self.thetalf, self.thetarf, self.thetarr, self.thetalr)
                        self.M = self.Spot.xyz_rotation_matrix(self.theta_spot[0], self.theta_spot[1], self.theta_spot[2], False)
                        self.CGabs = self.Spot.new_coordinates(self.M, self.CG[0], self.CG[1], self.CG[2],
                                                               self.x_spot[1], self.y_spot[1], self.z_spot[1])

                        # ПРИМЕНЯЕМ стабилизацию только если расчет ЦТ успешен
                        self.pos = self.stabilize_body_cg_imu(self.pos, self.CGabs, self.Angle)

                        if hasattr(self, 'check_leg_positions_reachable'):
                            try:
                                self.pos = self.check_leg_positions_reachable(self.pos)
                            except Exception as e:
                                print(f"Ошибка при проверке достижимости позиций в ходьбе: {e}")

                        # Обновляем переменные после стабилизации
                        self.theta_spot = self.pos[12]
                        self.x_spot = self.pos[13]
                        self.y_spot = self.pos[14]
                        self.z_spot = self.pos[15]

                    except Exception as e:
                        print(f"Ошибка расчета центра тяжести: {e}")
                # Обновляем переменные после всех модификаций
                self.theta_spot = self.pos[12]
                self.x_spot = self.pos[13]
                self.y_spot = self.pos[14]
                self.z_spot = self.pos[15]

                if self.walking and self.frame_counter % 50 == 0:
                    norm_x, norm_y = self.normalize_cg_offset()
                #    print(f"CG норм: X={norm_x:.2f}, Y={norm_y:.2f} | "
                #          f"Смещение тела: X={getattr(self, 'body_shift_x', 0):.1f}, "
                #          f"Y={getattr(self, 'body_shift_y', 0):.1f}")

                # Check if walking should stop
                if self.stop and self.t >= (self.tstop + 1):  # ?????????? ???????
                    # t > (tstop + 1 - tstep)
                    # ?? ???????, ??? t ????????? tstop + 1 (????? ?????)
                    self.lock = False
                    self.stop = False
                    self.walking = False
                    self.in_turn_animation = False  # Clear turn flag
                    print("[STATE] ===> Switching to recovering!")
                    print(f"[STATE] Walking stopped at t={self.t}")
                    self.recovering = True  # Activate recovery state
                    self.t = 0

                    # Capture current pose to blend back to Stand
                    self.temp_start_pos = [self.pos[12][0], self.pos[12][1], self.pos[12][2], self.pos[13][1], self.pos[14][1], self.pos[15][1]]

                    self.current_action = "Realigning to Stand..."
                    print("[STATE] === WALKING DONE -> REALIGNING TO STAND ===")

            elif self.sitting:
                # Sitting/Standing logic
                alpha_sitting = -30 / 180 * pi
                x_end_sitting = self.Spot.xlr - self.Spot.L2 + self.Spot.L1 * cos(pi / 3) + self.Spot.Lb / 2 * cos(
                    -alpha_sitting) - self.Spot.d * sin(-alpha_sitting)
                z_end_sitting = self.Spot.L1 * sin(pi / 3) + self.Spot.Lb / 2 * sin(-alpha_sitting) + self.Spot.d * cos(-alpha_sitting)
                start_frame_pos = [0, 0, 0, self.x_offset, 0, self.b_height]
                end_frame_pos = [0, alpha_sitting, 0, x_end_sitting, 0, z_end_sitting]

                # Store initial sitting position for pawing reference
                if (self.t == 1) and (not self.pawing):
                    self.pos_sit_init = copy.deepcopy(self.pos)
                    # pawing
                    self.pawing = True
                    self.current_action = "Sitting completed - Pawing activated"
                    print("=== SITTING COMPLETED ===")
                    print("=== PAWING ACTIVATED ===")
                    print("Use 'paw_left', 'paw_right', 'paw_down' commands to control paws")

                if self.stop:
                    # Standing up - go from sitting position to standing
                    self.pos = self.Spot.moving(1 - self.t, end_frame_pos, start_frame_pos, self.pos)
                    self.t = self.t - 4 * self.tstep
                    if self.t <= 0:
                        self.t = 0
                        self.stop = False
                        self.sitting = False
                        self.Free = True
                        self.lock = False
                        self.pawing = False
                        self.joypal = -1
                        self.joypar = -1
                        self.current_action = "Standing completed"
                        print("[STATE] === STANDING COMPLETED ===")
                        print("[STATE] Robot is now Free and ready for new commands")
                else:
                    if self.t < 1:
                        # Sitting down - go from standing to sitting position
                        self.pos = self.Spot.moving(self.t, start_frame_pos, end_frame_pos, self.pos)
                        self.t = self.t + 4 * self.tstep
                        if self.t >= 1:
                            self.t = 1
                            self.Free = True
                            self.lock = False
                    elif self.t == 1 and self.pawing:
                        L_paw = 2000  #
                        alpha_pawing = -30/90*pi  #

                        # Right front leg pawing
                        self.pos[3] = self.pos_sit_init[3] + (L_paw*cos(alpha_pawing)-self.pos_sit_init[3])*(self.joypar+1)/2
                        self.pos[5] = self.pos_sit_init[5] + (-self.Spot.d-L_paw*sin(alpha_pawing)-self.pos_sit_init[5])*(self.joypar+1)/2

                        # Left front leg pawing -
                        self.pos[0] = self.pos_sit_init[0] + (L_paw*cos(alpha_pawing)-self.pos_sit_init[0])*(self.joypal+1)/2
                        self.pos[2] = self.pos_sit_init[2] + (-self.Spot.d-L_paw*sin(alpha_pawing)-self.pos_sit_init[2])*(self.joypal+1)/2


                        # ---- IK ??? ???????? ??? ----
                        self.thetarf = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                                    self.pos[3], self.pos[4], self.pos[5], -1)[0]
                        self.thetalf = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                                    self.pos[0], self.pos[1], self.pos[2], 1)[0]

                        # ---- FK для анимации ----
                        self.legrf = self.Spot.FK(self.thetarf, -1)
                        self.leglf = self.Spot.FK(self.thetalf, 1)

                        self.xlegrf = self.Spot.xrf + self.pos[3]
                        self.ylegrf = self.Spot.yrf + self.pos[4]
                        self.zlegrf = self.pos[5]

                        self.xleglf = self.Spot.xlf + self.pos[0]
                        self.yleglf = self.Spot.ylf + self.pos[1]
                        self.zleglf = self.pos[2]

                        # ---- Обновляем абсолютные позиции тела ----
                        self.theta_spot_sit = self.pos[12]
                        self.x_spot_sit = self.pos[13]
                        self.y_spot_sit = self.pos[14]
                        self.z_spot_sit = self.pos[15]

                        self.M = self.Spot.xyz_rotation_matrix(self.theta_spot_sit[3],
                                                               self.theta_spot_sit[4],
                                                               self.theta_spot_sit[2] + self.theta_spot_sit[5],
                                                               False)

                        self.paw_rf = self.Spot.new_coordinates(self.M, self.xlegrf, self.ylegrf, self.zlegrf,
                                                                self.x_spot_sit[1], self.y_spot_sit[1],
                                                                self.z_spot_sit[1])
                        self.paw_lf = self.Spot.new_coordinates(self.M, self.xleglf, self.yleglf, self.zleglf,
                                                                self.x_spot_sit[1], self.y_spot_sit[1],
                                                                self.z_spot_sit[1])

                        self.x_spot_sit[3] = self.paw_rf[0]
                        self.y_spot_sit[3] = self.paw_rf[1]
                        self.z_spot_sit[3] = self.paw_rf[2]

                        self.x_spot_sit[2] = self.paw_lf[0]
                        self.y_spot_sit[2] = self.paw_lf[1]
                        self.z_spot_sit[2] = self.paw_lf[2]

                        self.pos[13] = self.x_spot_sit
                        self.pos[14] = self.y_spot_sit
                        self.pos[15] = self.z_spot_sit


            elif self.lying:
                # Lying logic
                angle_lying = 40 / 180 * pi
                x_end_lying = self.Spot.xlr - self.Spot.L2 + self.Spot.L1 * cos(angle_lying) + self.Spot.Lb / 2
                z_end_lying = self.Spot.L1 * sin(angle_lying) + self.Spot.d
                start_frame_pos = [0, 0, 0, self.x_offset, 0, self.b_height]
                end_frame_pos = [0, 0, 0, x_end_lying, 0, z_end_lying]

                self.pos = self.Spot.moving(self.t, start_frame_pos, end_frame_pos, self.pos)

                if self.stop == False:
                    if self.t < 1:
                        self.t = self.t + 3 * self.tstep
                        if self.t >= 1:
                            self.t = 1
                            self.Free = True
                            self.lock = False
                else:
                    self.t = self.t - 3 * self.tstep
                    if self.t <= 0:
                        self.t = 0
                        self.stop = False
                        self.lying = False
                        self.Free = True
                        self.lock = False
                        self.current_action = "Standing up completed"
                        print("=== STANDING UP COMPLETED ===")

            elif self.twisting:
                # Twisting logic
                x_angle_twisting = 0 / 180 * pi
                y_angle_twisting = 0 / 180 * pi
                z_angle_twisting = 30 / 180 * pi
                start_frame_pos = [0, 0, 0, self.x_offset, 0, self.b_height]

                self.t = self.t + 4 * self.tstep
                if self.t >= 1:
                    self.t = 1
                    self.twisting = False
                    self.Free = True
                    self.current_action = "Twisting completed"
                    print("=== TWISTING COMPLETED ===")

                if self.t < 0.25:
                    end_frame_pos = [x_angle_twisting, y_angle_twisting, z_angle_twisting, self.x_offset, 0, self.b_height]
                    self.pos = self.Spot.moving(self.t * 4, start_frame_pos, end_frame_pos, self.pos)
                elif self.t >= 0.25 and self.t < 0.5:
                    end_frame_pos = [x_angle_twisting, y_angle_twisting, z_angle_twisting, self.x_offset, 0, self.b_height]
                    self.pos = self.Spot.moving((self.t - 0.25) * 4, end_frame_pos, start_frame_pos, self.pos)
                elif self.t >= 0.5 and self.t < 0.75:
                    end_frame_pos = [-x_angle_twisting, -y_angle_twisting, -z_angle_twisting, self.x_offset, 0, self.b_height]
                    self.pos = self.Spot.moving((self.t - 0.5) * 4, start_frame_pos, end_frame_pos, self.pos)
                elif self.t >= 0.75 and self.t <= 1:
                    end_frame_pos = [-x_angle_twisting, -y_angle_twisting, -z_angle_twisting, self.x_offset, 0, self.b_height]
                    self.pos = self.Spot.moving((self.t - 0.75) * 4, end_frame_pos, start_frame_pos, self.pos)

            elif self.shifting:
                # Shifting/Peeing logic
                self.x_end_shifting = self.ra_longi
                self.y_end_shifting = -self.ra_lat
                start_frame_pos = [0, 0, 0, self.x_offset, 0, self.b_height]
                end_frame_pos = [0, 0, 0, self.x_end_shifting + self.x_offset, self.y_end_shifting, self.b_height]

                self.pos = self.Spot.moving(self.t, start_frame_pos, end_frame_pos, self.pos)

                if self.stop == False:
                    if self.t < 1:
                        self.t = self.t + 4 * self.tstep
                        if self.t >= 1:
                            self.t = 1
                            self.Free = True
                            self.lock = False
                else:
                    self.t = self.t - 4 * self.tstep
                    if self.t <= 0:
                        self.t = 0
                        self.stop = False
                        self.shifting = False
                        self.Free = True
                        self.current_action = "Shifting completed"
                        print("=== SHIFTING COMPLETED ===")
            elif self.recovering:
                # Recovery state: smoothly transition from walking pose back to standing
                print(f"[RECOVERY] Recovering... t={self.t:.2f}")
                
                # (Stand/Initial position)
                self.transtep = 0.8 #0.5  #
                self.t = self.t + self.transtep  # transtep = 0.025,

                # Spot.moving (roll, pitch, yaw, x_body, y_body, z_body)
                start_frame_pos = [self.temp_start_pos[0], self.temp_start_pos[1], self.temp_start_pos[2], self.temp_start_pos[3],
                                   self.temp_start_pos[4], self.temp_start_pos[5]]
                end_frame_pos = [0, 0, 0, self.x_offset, 0, self.b_height]  # ??????? ??????

                self.pos = self.Spot.moving(self.t, start_frame_pos, end_frame_pos, self.pos)

                if self.t >= 1:
                    self.t = 0
                    self.recovering = False
                    self.Free = True
                    self.lock = False
                    self.walking = False
                    self.in_turn_animation = False  # Ensure turn flag is cleared
                    self.current_action = "Stand recovery complete"
                    print("[RECOVERY] === STAND RECOVERY COMPLETED ===")
                    print("[STATE] Robot is now Free and ready for new commands")

                    self.pos_init = [-self.x_offset, self.track, -self.b_height, -self.x_offset, -self.track, -self.b_height, -self.x_offset, -self.track, -self.b_height,
                                -self.x_offset, self.track, -self.b_height]
                    self.pos[0:12] = self.pos_init  #
                    self.pos[12] = [0, 0, 0, 0, 0, 0]  #
                    self.pos[13] = [0, self.x_offset, self.Spot.xlf, self.Spot.xrf, self.Spot.xrr, self.Spot.xlr, self.pos[13][6], self.pos[13][7], self.pos[13][8],
                               self.pos[13][9]]  # ????? X
                    self.pos[14] = [0, 0, self.Spot.ylf + self.track, self.Spot.yrf - self.track, self.Spot.yrr - self.track, self.Spot.ylr + self.track, self.pos[14][6],
                               self.pos[14][7], self.pos[14][8], self.pos[14][9]]  # ????? Y
                    self.pos[15] = [0, self.b_height, 0, 0, 0, 0, self.pos[15][6], self.pos[15][7], self.pos[15][8], self.pos[15][9]]  # ????? Z

                # Calculate center for animation
            self.xc = self.steering * cos(self.walking_direction)
            self.yc = self.steering * sin(self.walking_direction)
            self.center_x = self.x_spot[0] + (self.xc * cos(self.theta_spot[2]) - self.yc * sin(self.theta_spot[2]))
            self.center_y = self.y_spot[0] + (self.xc * sin(self.theta_spot[2]) + self.yc * cos(self.theta_spot[2]))

            # Calculate servo angles for animation
            self.thetalf = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, self.pos[0], self.pos[1], self.pos[2], 1)[0]
            self.thetarf = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, self.pos[3], self.pos[4], self.pos[5], -1)[0]
            self.thetarr = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, self.pos[6], self.pos[7], self.pos[8], -1)[0]
            self.thetalr = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, self.pos[9], self.pos[10], self.pos[11], 1)[0]

            # Update stance for animation
            self.stance = [False, False, False, False]
            if self.pos[15][2] < 10.01:
                self.stance[0] = True
            if self.pos[15][3] < 10.01:
                self.stance[1] = True
            if self.pos[15][4] < 10.01:
                self.stance[2] = True
            if self.pos[15][5] < 10.01:
                self.stance[3] = True

            # Display and animation
            if self.anim_flag:
                try:
                    self.anim.animate(self.pos, self.t, pi / 12, -135 / 180 * pi, self.Angle, self.center_x, self.center_y,
                                     self.thetalf, self.thetarf, self.thetarr, self.thetalr, self.walking_speed,
                                     self.walking_direction, self.steering, self.stance)
                except Exception as e:
                    print(f"Animation error: {e}")

            # Display buttons
            self.mouse = pygame.mouse.get_pos()

            # Animation toggle display
            if (self.mouse[0] >= 510) and (self.mouse[0] <= 590) and (self.mouse[1] >= 500) and (self.mouse[1] <= 540):
                pygame.draw.rect(self.screen, self.WHITE, [510, 500, 80, 40], 5)
                if self.mouseclick and not self.lockmouse:
                    self.lockmouse = True
                    self.anim_flag = not self.anim_flag
                    print(f"Animation toggled: {'ON' if self.anim_flag else 'OFF'}")
            else:
                pygame.draw.rect(self.screen, self.WHITE, [510, 500, 80, 40], 5)

            # После отображения кнопок добавьте:
            if self.anim_flag and hasattr(self, 'CGabs'):
                # Рисуем центр тяжести
                cg_screen_x = int(300 + self.CGabs[0] / 2)
                cg_screen_y = int(300 - self.CGabs[1] / 2)
                pygame.draw.circle(self.screen, self.RED, (cg_screen_x, cg_screen_y), 5)

                # Рисуем проекцию центра корпуса
                body_screen_x = int(300 + self.x_spot[1] / 2)
                body_screen_y = int(300 - self.y_spot[1] / 2)
                pygame.draw.circle(self.screen, self.BLUE, (body_screen_x, body_screen_y), 3)

                # Линия от центра корпуса к ЦТ
                pygame.draw.line(self.screen, self.GREEN,
                                (body_screen_x, body_screen_y),
                                (cg_screen_x, cg_screen_y), 2)

            # Move toggle display
            if (self.mouse[0] >= 510) and (self.mouse[0] <= 590) and (self.mouse[1] >= 550) and (self.mouse[1] <= 590):
                pygame.draw.rect(self.screen, self.WHITE, [510, 550, 80, 40], 5)
                if self.mouseclick and not self.lockmouse and self.Free:
                    self.lockmouse = True
                    self.move_flag = not self.move_flag
                    print(f"Servo movement toggled: {'ON' if self.move_flag else 'OFF'}")
            else:
                pygame.draw.rect(self.screen, self.WHITE, [510, 550, 80, 40], 5)

            if not self.mouseclick and self.lockmouse:
                self.lockmouse = False

            if self.anim_flag:
                pygame.draw.rect(self.screen, self.GREEN, [510, 500, 80, 40])
                self.screen.blit(self.text_animon, (520, 510))
            else:
                pygame.draw.rect(self.screen, self.RED, [510, 500, 80, 40])
                self.screen.blit(self.text_animoff, (520, 510))

            if self.move_flag:
                pygame.draw.rect(self.screen, self.GREEN, [510, 550, 80, 40])
                self.screen.blit(self.text_moveon, (520, 560))
            else:
                pygame.draw.rect(self.screen, self.RED, [510, 550, 80, 40])
                self.screen.blit(self.text_moveoff, (520, 560))

            pygame.display.flip()

            self.moving(self.pos, self.move_flag)

            # Update CG and other calculations
            self.thetalf = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, self.pos[0], self.pos[1], self.pos[2], 1)[0]
            self.thetarf = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, self.pos[3], self.pos[4], self.pos[5], -1)[0]
            self.thetarr = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, self.pos[6], self.pos[7], self.pos[8], -1)[0]
            self.thetalr = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d, self.pos[9], self.pos[10], self.pos[11], 1)[0]

            self.CG = self.SpotCG.CG_calculation(self.thetalf, self.thetarf, self.thetarr, self.thetalr)
            self.M = self.Spot.xyz_rotation_matrix(self.theta_spot[0], self.theta_spot[1], self.theta_spot[2], False)
            self.CGabs = self.Spot.new_coordinates(self.M, self.CG[0], self.CG[1], self.CG[2], self.x_spot[1], self.y_spot[1], self.z_spot[1])


            self.pos[13][6] = self.CG[0]
            self.pos[14][6] = self.CG[1]
            self.pos[15][6] = self.CG[2]

            self.pos[13][7] = self.CGabs[0]
            self.pos[14][7] = self.CGabs[1]
            self.pos[15][7] = self.CGabs[2]

        print("Shutting down...")
        pygame.quit()

    # ==================== HARDWARE HELPERS ====================
    """
    def comp_filter(self, angle, t, T):
        acc = self.mpu.get_accel_data()
        gyr = self.mpu.get_gyro_data()
        denb = sqrt(acc['y'] ** 2 + acc['z'] ** 2)
        dena = sqrt(acc['z'] ** 2 + acc['x'] ** 2)
        alpha = atan(acc['y'] / dena) if dena else 0
        beta = atan(acc['x'] / denb) if denb else 0
        A = T / (T + t)
        anglex = A * (angle[0] + t * gyr['x'] / 180 * pi) + (1 - A) * alpha
        angley = A * (angle[1] + t * gyr['y'] / 180 * pi) + (1 - A) * beta
        return [anglex, angley]
"""
    def comp_filter (self, angle,t,T):
        #Complementary filter calculates body angles around xt and y axis from IMU data
        acc = self.mpu.acceleration
        gyr = self.mpu.gyro
        denb = sqrt(acc[1]**2+acc[2]**2)
        dena = sqrt(acc[2]**2+acc[0]**2)

        if (dena == 0):
            alpha = 0
        else:
            alpha = atan (acc[1]/dena)

        if (denb == 0):
            beta = 0
        else:
            beta = atan (acc[0]/denb)

        A = T/(T+t)

        anglex = A*(angle[0]+t*gyr[0]/180*pi)+(1-A)*alpha
        angley = A*(angle[1]+t*gyr[1]/180*pi)+(1-A)*beta
        #print(acc, gyr)
        #print(anglex, angley)
        return [anglex, angley]

    def normalize_cg_offset(self):
        """Нормализует смещение ЦТ относительно корпуса"""
        if not hasattr(self, 'CGabs'):
            return 0, 0

        # Смещение ЦТ от центра корпуса
        offset_x = self.CGabs[0] - self.x_spot[1]
        offset_y = self.CGabs[1] - self.y_spot[1]

        # Нормализуем (приводим к диапазону -1..1)
        norm_x = offset_x / 100.0  # 100мм - масштабный коэффициент
        norm_y = offset_y / 100.0

        # Ограничиваем
        norm_x = max(min(norm_x, 1.0), -1.0)
        norm_y = max(min(norm_y, 1.0), -1.0)

        return norm_x, norm_y

    def stabilize_body_cg_imu(self, pos, cg_abs, imu_angles):
        """Стабилизация с корректировкой корпуса, а не ног"""

        if not self.walking or not self.cg_stabilization_enabled:
            return pos

        try:
            # 1. Сначала корректируем КОРПУС, а не ноги
            if hasattr(self, 'CGabs'):
                # Смещение ЦТ относительно центра корпуса
                cg_offset_x = self.CGabs[0] - self.x_spot[1]  # Смещение вперед/назад
                cg_offset_y = self.CGabs[1] - self.y_spot[1]  # Смещение влево/вправо

                # Целевое смещение корпуса для компенсации (мягкая коррекция)
                target_body_shift_x = -cg_offset_x * 0.3  # Компенсируем 30% смещения
                target_body_shift_y = -cg_offset_y * 0.3

                # Применяем плавную фильтрацию к смещению корпуса
                if not hasattr(self, 'body_shift_x'):
                    self.body_shift_x = 0
                    self.body_shift_y = 0

                self.body_shift_x = self.body_shift_x * 0.8 + target_body_shift_x * 0.2
                self.body_shift_y = self.body_shift_y * 0.8 + target_body_shift_y * 0.2

                # Ограничиваем смещение корпуса
                max_body_shift = 20  # Максимальное смещение корпуса (мм)
                self.body_shift_x = max(min(self.body_shift_x, max_body_shift), -max_body_shift)
                self.body_shift_y = max(min(self.body_shift_y, max_body_shift), -max_body_shift)

                # Корректируем позицию корпуса (в структуре pos)
                # Корпус находится в индексах 13-15, элемент [1] - это тело
                pos[13][1] = self.x_spot[1] + self.body_shift_x  # X корпуса
                pos[14][1] = self.y_spot[1] + self.body_shift_y  # Y корпуса

            # 2. Компенсация наклона по IMU (более мягкая)
            if self.IMU_Comp and self.imu_stabilization_enabled:
                roll = imu_angles[0] * 0.5  # Уменьшенный коэффициент
                pitch = imu_angles[1] * 0.5

                # Легкая коррекция высоты ног для компенсации наклона
                # Индексы Z: LF=2, RF=5, RR=8, LR=11
                z_corrections = [
                    -5 * pitch - 5 * roll,   # LF (перед-лев)
                    -5 * pitch + 5 * roll,   # RF (перед-прав)
                    5 * pitch + 5 * roll,    # RR (зад-прав)
                    5 * pitch - 5 * roll     # LR (зад-лев)
                ]

                # Применяем очень небольшие корректировки
                z_indices = [2, 5, 8, 11]
                for i, z_idx in enumerate(z_indices):
                    # Сохраняем оригинальную амплитуду движения
                    original_z = pos[z_idx]

                    # Добавляем очень небольшую корректировку
                    correction = z_corrections[i] * 0.1  # Всего 10% от расчета
                    pos[z_idx] = original_z + correction

                    # Ограничиваем, чтобы не мешать основной анимации ходьбы
                    max_z_adjustment = 10  # Максимальная корректировка 10мм
                    if abs(pos[z_idx] - original_z) > max_z_adjustment:
                        pos[z_idx] = original_z + (max_z_adjustment if correction > 0 else -max_z_adjustment)

            return pos

        except Exception as e:
            print(f"Ошибка стабилизации: {e}")
            return pos


    def check_leg_positions_reachable(self, pos):
        """Проверяет, что все позиции ног достижимы"""
        try:
            # Более мягкие ограничения для ходьбы
            if self.walking:
                min_z = -60  # Было -80
                max_z = 40   # Было 30
                max_xy = 120 # Было 100
            else:
                min_z = -80
                max_z = 30
                max_xy = 100

            for i in range(4):
                # Индексы: LF(0,1,2), RF(3,4,5), RR(6,7,8), LR(9,10,11)
                x_idx = i*3
                y_idx = i*3 + 1
                z_idx = i*3 + 2

                # Получаем конструктивные смещения для этой ноги
                if i == 0:  # LF
                    x_offset, y_offset, z_offset = self.xtlf, self.ytlf, self.ztlf
                elif i == 1:  # RF
                    x_offset, y_offset, z_offset = self.xtrf, self.ytrf, self.ztrf
                elif i == 2:  # RR
                    x_offset, y_offset, z_offset = self.xtrr, self.ytrr, self.ztrr
                else:  # LR
                    x_offset, y_offset, z_offset = self.xtlr, self.ytlr, self.ztlr

                # Абсолютные координаты
                x_abs = pos[x_idx] + x_offset
                y_abs = pos[y_idx] + y_offset
                z_abs = pos[z_idx] + z_offset

                # Корректируем только если сильно вышли за пределы
                if z_abs < min_z:
                    # Поднимаем ногу, но не слишком резко
                    correction = (min_z - z_abs) * 0.3  # Корректируем только 30%
                    pos[z_idx] += correction
                    #print(f"Коррекция ноги {i}: z={z_abs:.1f} -> {pos[z_idx]+z_offset:.1f}")
                elif z_abs > max_z:
                    correction = (max_z - z_abs) * 0.3
                    pos[z_idx] += correction
                    #print(f"Коррекция ноги {i}: z={z_abs:.1f} -> {pos[z_idx]+z_offset:.1f}")

                # Проверяем горизонтальные смещения (только предупреждение)
                if abs(x_abs) > max_xy or abs(y_abs) > max_xy:
                    print(f"Внимание: нога {i} близка к пределу: x={x_abs:.1f}, y={y_abs:.1f}")

            return pos

        except Exception as e:
            print(f"Ошибка в check_leg_positions_reachable: {e}")
            return pos

    # moving servos
    def moving(self, pos, move):
        # Инициализация текущих углов сервоприводов при первом вызове
        if not hasattr(self, 'current_servo_angles'):
            self.current_servo_angles = [0.0] * 12

        # Вычисляем целевые углы с помощью обратной кинематики
        thetalf_reply = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                    pos[0] + self.xtlf, pos[1] + self.ytlf, pos[2] + self.ztlf, 1)
        thetarf_reply = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                    pos[3] + self.xtrf, pos[4] + self.ytrf, pos[5] + self.ztrf, -1)
        thetarr_reply = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                    pos[6] + self.xtrr, pos[7] + self.ytrr, pos[8] + self.ztrr, -1)
        thetalr_reply = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                    pos[9] + self.xtlr, pos[10] + self.ytlr, pos[11] + self.ztlr, 1)

        if move:
            # АДАПТИВНОЕ сглаживание: меньше при остановке, больше при ходьбе
            if self.walking and not self.stop:
                servo_smoothing = 0.4  # Плавность при ходьбе
            else:
                servo_smoothing = 0.1  # Меньше плавности при остановке/переходах

            # Обрабатываем левую переднюю ногу
            if not thetalf_reply[1]:
                for i in range(3):
                    target_angle_rad = thetalf_reply[0][i]
                    target_angle_deg = target_angle_rad / pi * 180

                    if i == 0:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_lf1 * self.Spot.dir01 + self.Spot.zero01
                        current_idx = 0
                    elif i == 1:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_lf2 * self.Spot.dir02 + self.Spot.zero02
                        current_idx = 1
                    else:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_lf3 * self.Spot.dir03 + self.Spot.zero03
                        current_idx = 2

                    current_angle = self.current_servo_angles[current_idx]
                    smoothed_angle = current_angle * (1 - servo_smoothing) + target_angle_calib * servo_smoothing
                    self.current_servo_angles[current_idx] = smoothed_angle

                    try:
                        self.kit0.servo[self.Spot.servo_table[current_idx]].angle = smoothed_angle
                    except ValueError:
                        print(f'Angle out of Range for servo {current_idx}')

            # Обрабатываем правую переднюю ногу
            if not thetarf_reply[1]:
                for i in range(3):
                    target_angle_rad = thetarf_reply[0][i]
                    target_angle_deg = target_angle_rad / pi * 180

                    if i == 0:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_rf1 * self.Spot.dir04 + self.Spot.zero04
                        current_idx = 3
                    elif i == 1:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_rf2 * self.Spot.dir05 + self.Spot.zero05
                        current_idx = 4
                    else:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_rf3 * self.Spot.dir06 + self.Spot.zero06
                        current_idx = 5

                    current_angle = self.current_servo_angles[current_idx]
                    smoothed_angle = current_angle * (1 - servo_smoothing) + target_angle_calib * servo_smoothing
                    self.current_servo_angles[current_idx] = smoothed_angle

                    try:
                        self.kit0.servo[self.Spot.servo_table[current_idx]].angle = smoothed_angle
                    except ValueError:
                        print(f'Angle out of Range for servo {current_idx}')

            # Обрабатываем правую заднюю ногу
            if not thetarr_reply[1]:
                for i in range(3):
                    target_angle_rad = thetarr_reply[0][i]
                    target_angle_deg = target_angle_rad / pi * 180

                    if i == 0:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_rr1 * self.Spot.dir07 + self.Spot.zero07
                        current_idx = 6
                    elif i == 1:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_rr2 * self.Spot.dir08 + self.Spot.zero08
                        current_idx = 7
                    else:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_rr3 * self.Spot.dir09 + self.Spot.zero09
                        current_idx = 8

                    current_angle = self.current_servo_angles[current_idx]
                    smoothed_angle = current_angle * (1 - servo_smoothing) + target_angle_calib * servo_smoothing
                    self.current_servo_angles[current_idx] = smoothed_angle

                    try:
                        self.kit1.servo[self.Spot.servo_table[current_idx]].angle = smoothed_angle
                    except ValueError:
                        print(f'Angle out of Range for servo {current_idx}')

            # Обрабатываем левую заднюю ногу
            if not thetalr_reply[1]:
                for i in range(3):
                    target_angle_rad = thetalr_reply[0][i]
                    target_angle_deg = target_angle_rad / pi * 180

                    if i == 0:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_lr1 * self.Spot.dir10 + self.Spot.zero10
                        current_idx = 9
                    elif i == 1:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_lr2 * self.Spot.dir11 + self.Spot.zero11
                        current_idx = 10
                    else:
                        target_angle_calib = target_angle_deg * self.Spot.angle_scale_factor_lr3 * self.Spot.dir12 + self.Spot.zero12
                        current_idx = 11

                    current_angle = self.current_servo_angles[current_idx]
                    smoothed_angle = current_angle * (1 - servo_smoothing) + target_angle_calib * servo_smoothing
                    self.current_servo_angles[current_idx] = smoothed_angle

                    try:
                        self.kit1.servo[self.Spot.servo_table[current_idx]].angle = smoothed_angle
                    except ValueError:
                        print(f'Angle out of Range for servo {current_idx}')
# ======================

#if __name__ == "__main__":
#controller = SpotMicroController()
#    print("=== Created controller ===")
#controller.start()
