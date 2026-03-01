#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import queue
import numpy as np
import copy
from math import pi, sin, cos, atan, sqrt, radians
from time import sleep, time
import csv
import socket
from datetime import datetime

# External dependencies
import pygame
import board
import busio
import adafruit_pca9685
import adafruit_mpu6050
#import mpu6050 #import mpu6050
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO

# Camera imports
try:
    from picamera2 import Picamera2
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False
    print("Warning: Picamera2 or OpenCV not available. Camera functionality disabled.")

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
        # ÐÑÐºÐ»ÑÑÐ°ÐµÐ¼ Ð°Ð¿Ð¿Ð°ÑÐ°ÑÐ½Ð¾Ðµ ÑÑÐºÐ¾ÑÐµÐ½Ð¸Ðµ Ð´Ð»Ñ Ð¸Ð·Ð±ÐµÐ¶Ð°Ð½Ð¸Ñ Ð¿ÑÐ¾Ð±Ð»ÐµÐ¼ Ñ OpenGL
        os.environ['SDL_VIDEO_X11_NOWINDOWMOVE'] = '1'
        pygame.init()
        print("=== Controller __init__ END ===")

        # ÐÑÐ¿ÑÐ°Ð²Ð»ÑÐµÐ¼ Ð¸Ð½Ð¸ÑÐ¸Ð°Ð»Ð¸Ð·Ð°ÑÐ¸Ñ Ð´Ð¸ÑÐ¿Ð»ÐµÑ
        self.screen = pygame.display.set_mode((600, 600))
        pygame.display.set_caption("SPOTMICRO CONSOLE CONTROL")

        # Ð¦Ð²ÐµÑÐ° Ð´Ð»Ñ Ð¸Ð½ÑÐµÑÑÐµÐ¹ÑÐ°
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

        # == Main state
        self.anim_flag = False
        self.move_flag = True
        self.walking = False
        self.trot = False
        self._strafe_trot_applied = False
        self.sitting = False
        self.lying = False
        self.twisting = False
        self.twist_queue_count = 0
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


        # == Walking params
        self.b_height = 220
        self.x_offset = 0
        self.track2, self.track4 = 58, 58  # Ð¨Ð¸ÑÐ¸Ð½Ð° Ð¿Ð¾ÑÑÐ°Ð½Ð¾Ð²ÐºÐ¸ Ð½Ð¾Ð³ (Y-ÐºÐ¾Ð¾ÑÐ´Ð¸Ð½Ð°ÑÐ°)
        self.h_amp2, self.h_amp4 = 100, 60 #80  # ÐÐ¾ÑÐ¸Ð·Ð¾Ð½ÑÐ°Ð»ÑÐ½Ð°Ñ Ð°Ð¼Ð¿Ð»Ð¸ÑÑÐ´Ð° Ð´Ð²Ð¸Ð¶ÐµÐ½Ð¸Ñ Ð½Ð¾Ð³ (X-Ð½Ð°Ð¿ÑÐ°Ð²Ð»ÐµÐ½Ð¸Ðµ)
        self.v_amp2, self.v_amp4 = 20, 45 #25 # ÐÐµÑÑÐ¸ÐºÐ°Ð»ÑÐ½Ð°Ñ Ð°Ð¼Ð¿Ð»Ð¸ÑÑÐ´Ð° Ð¿Ð¾Ð´ÑÐµÐ¼Ð° Ð½Ð¾Ð³ (Z-Ð½Ð°Ð¿ÑÐ°Ð²Ð»ÐµÐ½Ð¸Ðµ)
        #Ð´Ð»Ð¸Ð½a ÑÐ°Ð³Ð°
        self.stepl2, self.stepl4 = 0.16, 0.125 #was 0.2#08#0.125 # Ð¡ÐÐÐ ÐÐ¡Ð¢Ð¬ ÐÐÐ ÐÐÐÐ©ÐÐÐÐ¯ Ð¢ÐÐÐ
        self.tstep2, self.tstep4 = self.stepl2 / 8, 0.015 #0.8 #0.012 #6666666666 # Ð²ÑÐµÐ¼Ñ ÑÐ°Ð³Ð°
        self.track = self.track4
        self.h_amp = self.h_amp4
        self.v_amp = self.v_amp4
        self.stepl = self.stepl4
        self.tstep = self.tstep4
        self.height = self.b_height
        self.prev_pos = None
        self.animation_smoothing = 0.5  # ÐÐ¾ÑÑÑÐ¸ÑÐ¸ÐµÐ½Ñ ÑÐ³Ð»Ð°Ð¶Ð¸Ð²Ð°Ð½Ð¸Ñ (0-1)
        self.target_speed = 0
        self.t = 0
        self.transtep = 0.0125
        self.trans = 0
        self.tstop = 1000
        self.current_movement_command = "stop"
        self.current_servo_angles = [0.0] * 12
        self.speed_smoothing = 0.3
        # ÐÐ°ÑÐ°Ð¼ÐµÑÑÑ ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ð¸
        self.cg_stabilization_enabled = True
        self.imu_stabilization_enabled = True  # ÐÐ¾Ð±Ð°Ð²Ð»ÑÐµÐ¼ Ð¾ÑÐ´ÐµÐ»ÑÐ½ÑÐ¹ ÑÐ»Ð°Ð³ Ð´Ð»Ñ IMU

        # ÐÐ¾Ð»ÐµÐµ ÐºÐ¾Ð½ÑÐµÑÐ²Ð°ÑÐ¸Ð²Ð½ÑÐµ Ð¿Ð°ÑÐ°Ð¼ÐµÑÑÑ ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ð¸
        self.max_cg_offset_x = 10  # Ð£Ð¼ÐµÐ½ÑÑÐµÐ½Ð¾ Ñ 15
        self.max_cg_offset_y = 8   # Ð£Ð¼ÐµÐ½ÑÑÐµÐ½Ð¾ Ñ 10
        self.max_leg_adjustment = 20  # ÐÐ°ÐºÑÐ¸Ð¼Ð°Ð»ÑÐ½Ð°Ñ ÐºÐ¾ÑÑÐµÐºÑÐ¸ÑÐ¾Ð²ÐºÐ° Ð½Ð¾Ð³Ð¸ (Ð¼Ð¼)

        # ÐÐ»Ð°Ð²Ð½Ð°Ñ ÑÐ¸Ð»ÑÑÑÐ°ÑÐ¸Ñ
        self.body_filter_alpha = 0.2  # ÐÐ¾Ð»ÐµÐµ ÑÐ¸Ð»ÑÐ½Ð¾Ðµ ÑÐ³Ð»Ð°Ð¶Ð¸Ð²Ð°Ð½Ð¸Ðµ
        self.leg_filter_alpha = 0.3   # Ð¤Ð¸Ð»ÑÑÑ Ð´Ð»Ñ Ð½Ð¾Ð³

        # ÐÐ½Ð¸ÑÐ¸Ð°Ð»Ð¸Ð·Ð°ÑÐ¸Ñ ÑÐ¸Ð»ÑÑÑÐ¾Ð²
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

        # == Sensor Configuration ==
        # HC-SR04 Ultrasonic sensors (left and right)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Left sensor
        self.TRIG_LEFT = 14  # BCM14
        self.ECHO_LEFT = 15  # BCM15

        # Right sensor
        self.TRIG_RIGHT = 23  # BCM23
        self.ECHO_RIGHT = 24  # BCM24

        # Touch sensor
        self.TOUCH_PIN = 17  # BCM17

        # Setup sensor pins
        GPIO.setup(self.TRIG_LEFT, GPIO.OUT)
        GPIO.setup(self.ECHO_LEFT, GPIO.IN)
        GPIO.setup(self.TRIG_RIGHT, GPIO.OUT)
        GPIO.setup(self.ECHO_RIGHT, GPIO.IN)
        GPIO.setup(self.TOUCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize ultrasonic triggers
        GPIO.output(self.TRIG_LEFT, False)
        GPIO.output(self.TRIG_RIGHT, False)

        # Sensor angle correction (sensors tilted ~40 degrees)
        # Measured 40cm shows actual 30cm distance, so correction factor is cos(40Â°)
        self.sensor_tilt_angle = 40  # degrees
        self.sensor_correction_factor = cos(radians(self.sensor_tilt_angle))  # ~0.766

        # Sensor state
        self.last_left_distance = -1
        self.last_right_distance = -1
        self.obstacle_detected = False
        self.touch_detected = False
        self.last_touch_time = 0
        self.touch_sequence_step = 0  # 0: none, 1: stopped, 2: sitting, 3: paw given
        self.paw_hold_start_time = 0  # Timer for holding paw
        self.paw_holding = False  # Flag for 10-second paw hold

        # == Camera Setup ==
        self.camera = None
        if CAMERA_AVAILABLE:
            try:
                self.camera = Picamera2()
                camera_config = self.camera.create_still_configuration()
                self.camera.configure(camera_config)
                print("Camera initialized successfully")
            except Exception as e:
                print(f"Warning: Could not initialize camera: {e}")
                self.camera = None

        # Create photos directory
        self.photos_dir = "/home//rpi//Desktop//ik//photos"
        os.makedirs(self.photos_dir, exist_ok=True)

        # == RL Environment Logging ==
        self.log_file = "/home//rpi//Desktop//ik//rl_environment_log.csv"
        self.init_logging()

        # == TCP Server for App Control ==
        self.tcp_server_socket = None
        self.tcp_port = 5000
        self.tcp_enabled = True
        self.start_tcp_server()

    # ==================== PRIMARY INTERFACE ====================

    def accept_command(self, command):
        with self.console_lock:
            self.command_queue.append(command)

    def start_console_thread(self):
        print('=== Console thread started ===')

        th = threading.Thread(target=self.console_input_thread, daemon=True)
        th.start()

    def start(self):
        self.start_console_thread()
        self.main_loop()

    # ==================== SENSOR METHODS ====================

    def measure_distance(self, trig_pin, echo_pin):
        """Measure distance using HC-SR04 ultrasonic sensor with angle correction"""
        try:
            GPIO.output(trig_pin, True)
            sleep(0.00001)
            GPIO.output(trig_pin, False)

            # Wait for echo to go high
            timeout_start = time() + 0.04
            pulse_start = time()

            while GPIO.input(echo_pin) == 0:
                pulse_start = time()
                if pulse_start > timeout_start:
                    return -1

            # Wait for echo to go low
            timeout_end = time() + 0.04
            pulse_end = time()

            while GPIO.input(echo_pin) == 1:
                pulse_end = time()
                if pulse_end > timeout_end:
                    return -1

            elapsed_time = pulse_end - pulse_start
            measured_distance_cm = elapsed_time * 17150

            # Apply angle correction for tilted sensors (~40 degrees)
            # actual_distance = measured_distance * cos(40Â°)
            actual_distance_cm = measured_distance_cm * self.sensor_correction_factor

            # Filter valid range
            if 2.0 <= actual_distance_cm <= 400.0:
                return actual_distance_cm
            return -1
        except Exception as e:
            print(f"Error measuring distance: {e}")
            return -1

    def read_sensors(self):
        """Read all sensors and update state"""
        # Read ultrasonic sensors
        self.last_left_distance = self.measure_distance(self.TRIG_LEFT, self.ECHO_LEFT)
        sleep(0.01)  # Small delay between sensors
        self.last_right_distance = self.measure_distance(self.TRIG_RIGHT, self.ECHO_RIGHT)

        # Read touch sensor (with pull-up: LOW=touched, HIGH=not touched)
        touch_state = GPIO.input(self.TOUCH_PIN)
        current_time = time()

        # Detect touch event (transition to touched - LOW with pull-up)
        if touch_state == 0 and not self.touch_detected:
            self.touch_detected = True
            self.last_touch_time = current_time
            self.handle_touch_event()
        elif touch_state == 1:
            self.touch_detected = False

    def handle_obstacle_avoidance(self):
        """Handle obstacle avoidance - keep turning until obstacle is clear"""
        # Only process if we were trying to move forward or are in obstacle avoidance mode
        if self.current_movement_command != "forward" and not hasattr(self, 'avoiding_obstacle'):
            return
    
        # Initialize obstacle avoidance state if not present
        if not hasattr(self, 'avoiding_obstacle'):
            self.avoiding_obstacle = False
            self.avoidance_turn_direction = None
            self.turn_start_time = 0
            self.obstacle_confirmed = False
    
        # Check if obstacle detected
        obstacle_threshold = 30  # cm (actual distance after correction)
        left_blocked = 0 < self.last_left_distance < obstacle_threshold
        right_blocked = 0 < self.last_right_distance < obstacle_threshold
    
        if left_blocked or right_blocked:
            self.obstacle_detected = True
    
            if not self.avoiding_obstacle:
                current_time = time()
                
                # Первое обнаружение - ждем подтверждения
                if not hasattr(self, 'first_obstacle_detection_time'):
                    self.first_obstacle_detection_time = current_time
                    print(" Obstacle detected, confirming...")
                    return
                
                # Проверяем подтверждение
                confirmation_time = 0.15
                if current_time - self.first_obstacle_detection_time < confirmation_time:
                    return
                
                # Препятствие подтверждено
                print("Obstacle CONFIRMED")
                self.avoiding_obstacle = True
                self.obstacle_confirmed = True
                self.turn_start_time = current_time
    
                # Определяем направление поворота
                if left_blocked and not right_blocked:
                    self.avoidance_turn_direction = "turn_right"
                    print(f"Turning RIGHT (left blocked: {self.last_left_distance:.1f}cm)")
                elif right_blocked and not left_blocked:
                    self.avoidance_turn_direction = "turn_left"
                    print(f" Turning LEFT (right blocked: {self.last_right_distance:.1f}cm)")
                else:
                    # Оба заблокированы - поворачиваем в сторону с большим расстоянием
                    if self.last_left_distance > self.last_right_distance:
                        self.avoidance_turn_direction = "turn_left"
                        print(f" Turning LEFT (less blocked: L={self.last_left_distance:.1f} > R={self.last_right_distance:.1f})")
                    else:
                        self.avoidance_turn_direction = "turn_right"
                        print(f" Turning RIGHT (less blocked: R={self.last_right_distance:.1f} > L={self.last_left_distance:.1f})")
    
                # Начинаем поворот БЕЗ остановки
                self.accept_command(self.avoidance_turn_direction)
            else:
                # Уже поворачиваем - продолжаем если команда поворота завершилась
                if self.current_movement_command != self.avoidance_turn_direction:
                    print(f" Continuing {self.avoidance_turn_direction} (obstacle still present)")
                    self.accept_command(self.avoidance_turn_direction)
        else:
            # Препятствие исчезло
            self.obstacle_detected = False
    
            # Сбрасываем таймер первого обнаружения
            if hasattr(self, 'first_obstacle_detection_time'):
                delattr(self, 'first_obstacle_detection_time')
    
            if self.avoiding_obstacle:
                # Проверяем: было ли препятствие подтверждено?
                if not self.obstacle_confirmed:
                    print("False alarm - ignoring (obstacle disappeared before confirmation)")
                    self.avoiding_obstacle = False
                    self.avoidance_turn_direction = None
                    self.turn_start_time = 0
                    return
                
                # Минимальное время поворота
                min_turn_duration = 0.5
                elapsed_turn_time = time() - self.turn_start_time
                
                if elapsed_turn_time >= min_turn_duration:
                    print(f" Obstacle cleared after {elapsed_turn_time:.2f}s turn")
                    print(" Stopping turn and transitioning to forward movement...")
                    
                    # Сбрасываем флаги избегания препятствий
                    self.avoiding_obstacle = False
                    self.avoidance_turn_direction = None
                    self.turn_start_time = 0
                    self.obstacle_confirmed = False
    
                    # ВАЖНО: Последовательность команд для плавного перехода
                    # 1. Останавливаем поворот
                    self.accept_command("stop_walk")
                    
                    # 2. НЕ нужно добавлять sleep здесь - это заблокирует main_loop
                    # 3. Команда forward будет обработана в следующем цикле
                    #    и автоматически вызовет transition_to_neutral() если нужно
                    self.accept_command("forward")
                    
                    print(" Commands queued: stop_walk → forward")
                else:
                    # Поворот начался недавно - продолжаем
                    remaining_time = min_turn_duration - elapsed_turn_time
                    print(f" Turn in progress ({elapsed_turn_time:.2f}s / {min_turn_duration}s), {remaining_time:.2f}s remaining...")
    
            elif self.current_movement_command == "forward":
                pass  # Both sensors clear, continue forward normally

    def handle_touch_event(self):
        """Handle touch sensor reaction: Stop -> Sit -> Give Paw (hold for 10 seconds)"""
        print(f"Touch detected! Sequence step: {self.touch_sequence_step}")
        current_time = time()

        if self.touch_sequence_step == 0:
            # First touch: Stop
            print("Touch sequence: STOP")
            self.accept_command("stop")
            self.touch_sequence_step = 1
        elif self.touch_sequence_step == 1 and self.Free:
            # Second touch: Sit
            print("Touch sequence: SIT")
            self.accept_command("sit")
            self.touch_sequence_step = 2
        elif self.touch_sequence_step == 2 and self.sitting and self.t >= 0.99:
            # Third touch and beyond: Give Paw Right and hold for 10 seconds
            if not self.paw_holding:
                print("Touch sequence: GIVE PAW RIGHT (holding for 10 seconds)")
                self.accept_command("paw_right")
                self.paw_holding = True
                self.paw_hold_start_time = current_time
                self.touch_sequence_step = 3
            else:
                print("Paw already being held, ignoring touch")
        elif self.touch_sequence_step >= 3 and self.sitting and self.t >= 0.99:
            # Already in paw holding state
            if not self.paw_holding:
                print("Touch sequence: GIVE PAW RIGHT (holding for 10 seconds)")
                self.accept_command("paw_right")
                self.paw_holding = True
                self.paw_hold_start_time = current_time
            else:
                print("Paw already being held, ignoring touch")

    def check_paw_hold_timer(self):
        """Check if 10 seconds have passed and lower the paw"""
        if self.paw_holding:
            current_time = time()
            elapsed = current_time - self.paw_hold_start_time
            if elapsed >= 10.0:
                print("10 seconds elapsed, lowering paw")
                self.accept_command("paw_down")
                self.paw_holding = False
                self.paw_hold_start_time = 0

    # ==================== CAMERA METHODS ====================

    def capture_photo(self):
        """Capture photo using Picamera2 and OpenCV"""
        if not CAMERA_AVAILABLE or self.camera is None:
            print("Camera not available")
            return False

        try:
            # Start camera if not already started
            if not self.camera.started:
                self.camera.start()
                sleep(0.5)  # Allow camera to warm up

            # Capture image
            image = self.camera.capture_array()

            # Convert to OpenCV format (RGB to BGR)
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"photo_{timestamp}.jpg"
            filepath = os.path.join(self.photos_dir, filename)

            # Save photo
            cv2.imwrite(filepath, image_bgr)
            print(f"Photo saved: {filepath}")

            return True
        except Exception as e:
            print(f"Error capturing photo: {e}")
            return False

    # ==================== LOGGING METHODS ====================

    def init_logging(self):
        """Initialize CSV logging for RL environment"""
        try:
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'command', 'action',
                    'left_distance', 'right_distance', 'touch_detected',
                    'imu_roll', 'imu_pitch', 'imu_accel_x', 'imu_accel_y', 'imu_accel_z',
                    'servo_0', 'servo_1', 'servo_2', 'servo_3', 'servo_4', 'servo_5',
                    'servo_6', 'servo_7', 'servo_8', 'servo_9', 'servo_10', 'servo_11'
                ])
            print(f"Logging initialized: {self.log_file}")
        except Exception as e:
            print(f"Error initializing logging: {e}")

    def log_state(self, command=""):
        """Log current robot state to CSV"""
        try:
            # Get IMU data
            try:
                accel = self.mpu.acceleration
                accel_x, accel_y, accel_z = accel
            except:
                accel_x = accel_y = accel_z = 0.0

            roll = self.Angle[0] if len(self.Angle) > 0 else 0.0
            pitch = self.Angle[1] if len(self.Angle) > 1 else 0.0

            # Log to CSV
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().isoformat(),
                    command,
                    self.current_action,
                    self.last_left_distance,
                    self.last_right_distance,
                    1 if self.touch_detected else 0,
                    roll,
                    pitch,
                    accel_x,
                    accel_y,
                    accel_z,
                    *self.current_servo_angles
                ])
        except Exception as e:
            print(f"Error logging state: {e}")

    # ==================== TCP SERVER METHODS ====================

    def start_tcp_server(self):
        """Start TCP server for app control

        Note: Binds to 0.0.0.0 to allow mobile app connections.
        WARNING: This exposes the server to all network interfaces.
        Use only on trusted networks or implement authentication if needed.
        """
        if not self.tcp_enabled:
            return

        try:
            self.tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # Bind to all interfaces for mobile app access
            # Change to '127.0.0.1' for local-only access if security is a concern
            self.tcp_server_socket.bind(('0.0.0.0', self.tcp_port))
            self.tcp_server_socket.listen(5)
            self.tcp_server_socket.settimeout(1.0)  # Non-blocking accept

            # Start TCP handler thread
            tcp_thread = threading.Thread(target=self.tcp_handler_thread, daemon=True)
            tcp_thread.start()

            print(f"TCP server started on port {self.tcp_port}")
        except Exception as e:
            print(f"Error starting TCP server: {e}")
            self.tcp_enabled = False

    def tcp_handler_thread(self):
        """Handle TCP connections from app"""
        print("TCP handler thread started")

        while self.continuer and self.tcp_enabled:
            try:
                # Accept connection with timeout
                try:
                    client_socket, address = self.tcp_server_socket.accept()
                    print(f"TCP client connected: {address}")

                    # Handle client in separate thread
                    client_thread = threading.Thread(
                        target=self.handle_tcp_client,
                        args=(client_socket, address),
                        daemon=True
                    )
                    client_thread.start()
                except socket.timeout:
                    continue
            except Exception as e:
                if self.continuer:
                    print(f"TCP accept error: {e}")
                sleep(0.1)

    def handle_tcp_client(self, client_socket, address):
        """Handle individual TCP client connection"""
        try:
            client_socket.settimeout(30.0)

            while self.continuer:
                try:
                    data = client_socket.recv(1024)
                    if not data:
                        break

                    command = data.decode('utf-8').strip().lower()
                    print(f"TCP command from {address}: {command}")

                    # Process command
                    if command == "photo":
                        success = self.capture_photo()
                        response = "OK" if success else "ERROR"
                        client_socket.send(response.encode('utf-8'))
                    else:
                        # Regular movement/action command
                        self.accept_command(command)
                        client_socket.send(b"OK")

                except socket.timeout:
                    break
                except Exception as e:
                    print(f"TCP client error: {e}")
                    break
        finally:
            client_socket.close()
            print(f"TCP client disconnected: {address}")

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
                print("=== WALKING STARTED ===")

        def transition_to_neutral():
            """Transition from any state to neutral (standing) position"""
            transitions_needed = []
            needs_wait = False

            # If paw is raised, lower it first
            if self.sitting and (self.joypal > -1.0 or self.joypar > -1.0):
                print("Transition: Lowering paw")
                transitions_needed.append("paw_down")
                self.paw_holding = False  # Cancel any ongoing paw hold

            # If sitting, stand up
            if self.sitting:
                if not self.stop and not self.lock:
                    print("Transition: Standing up from sitting")
                    transitions_needed.append("stand")
                else:
                    # If in middle of sitting animation, need to wait
                    print("Waiting for sitting animation to complete before transition")
                    needs_wait = True

            # If lying, stand up
            if self.lying:
                if not self.stop and not self.lock:
                    print("Transition: Standing up from lying")
                    transitions_needed.append("stand")
                else:
                    print("Waiting for lying animation to complete before transition")
                    needs_wait = True

            return transitions_needed, needs_wait

        with self.console_lock:
            while self.command_queue:
                command = self.command_queue.pop(0)
                print(f"Executing: {command}")

                # Check if we need transitions for movement commands
                movement_commands = ["forward", "backward", "left", "right", "turn_left", "turn_right", "walk"]
                needs_transition = command in movement_commands and not self.Free

                if needs_transition:
                    transitions, needs_wait = transition_to_neutral()
                    if needs_wait:
                        # Re-queue the command to try again later
                        print(f"Animation in progress, re-queuing '{command}'")
                        self.command_queue.append(command)
                        continue
                    if transitions:
                        print(f"Command '{command}' requires transition through neutral state")
                        # Add transitions to the front of the queue
                        for trans in reversed(transitions):
                            self.command_queue.insert(0, trans)
                        # Re-add the original command after transitions
                        self.command_queue.insert(len(transitions), command)
                        print(f"Transition sequence: {transitions} -> {command}")
                        continue  # Process transitions first

                if command == "quit":
                    self.continuer = False
                    self.current_action = "Shutting down"

                elif command == "forward":
                    ensure_walking_mode()
                    self.current_movement_command = "forward"
                    self.target_speed = 100
                    # сброс датчиков касания
                    self.touch_sequence_step = 0
                    self.paw_holding = False
                    self.paw_hold_start_time = 0
                    self.current_action = "Moving forward"

                elif command == "backward":
                    ensure_walking_mode()
                    self.current_movement_command = "backward"
                    self.target_speed = 100
                    self.current_action = "Moving backward"

                elif command == "left":
                    ensure_walking_mode()
                    self.current_movement_command = "left"
                    self.current_action = "Moving left"

                elif command == "right":
                    ensure_walking_mode()
                    self.current_movement_command = "right"
                    self.current_action = "Moving right"

                elif command == "turn_left":
                    ensure_walking_mode()
                    self.current_movement_command = "turn_left"
                    self.current_action = "Turning left"

                elif command == "turn_right":
                    ensure_walking_mode()
                    self.current_movement_command = "turn_right"
                    self.current_action = "Turning right"

                elif command == "stop_walk":
                    if self.walking:
                        self.stop = True
                        self.lock = True
                        self.tstop = int(self.t)
                        self.touch_sequence_step = 0
                        self.paw_holding = False
                        self.paw_hold_start_time = 0
                        self.current_movement_command = "stop"
                        self.current_action = "Stopping walk..."
                        print("=== STOPPING WALK SEQUENCE INITIATED ===")
                        self.current_action = "Movement stopped, remaining in Walk mode"
                    else:
                        print("Not in walking mode.")

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
                    # Ð¢ÐµÑÑÐ¾Ð²ÑÐ¹ ÑÐµÐ¶Ð¸Ð¼ ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ð¸
                    print("=== Ð ÐÐÐÐ Ð¢ÐÐ¡Ð¢ÐÐ ÐÐÐÐÐÐ¯ Ð¡Ð¢ÐÐÐÐÐÐÐÐ¦ÐÐ ===")
                    print(f"CG ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ñ: {'ÐÐÐ' if self.cg_stabilization_enabled else 'ÐÐ«ÐÐ'}")
                    print(f"IMU ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ñ: {'ÐÐÐ' if self.imu_stabilization_enabled else 'ÐÐ«ÐÐ'}")
                    print(f"Ð¡Ð¼ÐµÑÐµÐ½Ð¸Ðµ Ð¦Ð¢: X={self.CGabs[0]-self.x_spot[1]:.1f}, Y={self.CGabs[1]-self.y_spot[1]:.1f}")

                elif command == "adjust_params":
                    # ÐÐ°ÑÑÑÐ¾Ð¹ÐºÐ° Ð¿Ð°ÑÐ°Ð¼ÐµÑÑÐ¾Ð² ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ð¸
                    print("ÐÐ°ÑÑÑÐ¾Ð¹ÐºÐ° Ð¿Ð°ÑÐ°Ð¼ÐµÑÑÐ¾Ð² ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ð¸:")
                    print("1. Ð£Ð²ÐµÐ»Ð¸ÑÐ¸ÑÑ Ð¶ÐµÑÑÐºÐ¾ÑÑÑ")
                    print("2. Ð£Ð¼ÐµÐ½ÑÑÐ¸ÑÑ Ð¶ÐµÑÑÐºÐ¾ÑÑÑ")
                    print("3. ÐÐºÐ»ÑÑÐ¸ÑÑ/Ð²ÑÐºÐ»ÑÑÐ¸ÑÑ ÑÐ¸Ð»ÑÑÑÐ°ÑÐ¸Ñ")
                    choice = input("ÐÑÐ±ÐµÑÐ¸ÑÐµ Ð¾Ð¿ÑÐ¸Ñ: ")

                    if choice == "1":
                        self.body_filter_alpha = min(self.body_filter_alpha + 0.1, 0.5)
                        print(f"ÐÐµÑÑÐºÐ¾ÑÑÑ ÑÐ²ÐµÐ»Ð¸ÑÐµÐ½Ð°: alpha={self.body_filter_alpha}")
                    elif choice == "2":
                        self.body_filter_alpha = max(self.body_filter_alpha - 0.1, 0.05)
                        print(f"ÐÐµÑÑÐºÐ¾ÑÑÑ ÑÐ¼ÐµÐ½ÑÑÐµÐ½Ð°: alpha={self.body_filter_alpha}")
                    elif choice == "3":
                        self.cg_stabilization_enabled = not self.cg_stabilization_enabled
                        print(f"CG ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ñ: {'ÐÐÐ' if self.cg_stabilization_enabled else 'ÐÐ«ÐÐ'}")

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
                    self.twist_queue_count = 0
                    self.shifting = False
                    self.pawing = False
                    self.stop = False
                    self.Free = True
                    self.walking_speed = 0.0
                    self.current_movement_command = "stop"
                    self.joypal = -1
                    self.joypar = -1
                    self.touch_sequence_step = 0
                    self.paw_holding = False
                    self.paw_hold_start_time = 0
                    self.current_action = "EMERGENCY STOP - All motions stopped"
                    print("*** EMERGENCY STOP ***")

                elif command == "sit":
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
                        print("=== SITTING DOWN ===")
                    elif self.sitting and not self.stop and not self.lock:
                        self.stop = True
                        self.lock = True
                        self.pawing = False
                        self.current_action = "Standing up"
                        print("=== STANDING UP ===")

                elif command == "stand":
                    if (self.sitting or self.lying) and not self.stop and not self.lock:
                        self.stop = True
                        self.lock = True
                        self.pawing = False
                        self.touch_sequence_step = 0
                        self.paw_holding = False
                        self.paw_hold_start_time = 0
                        self.current_action = "Standing up"
                        print("=== STANDING UP ===")
                    elif self.Free:
                        print("Already standing.")


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

                elif command == "twist_3":
                    if not self.twisting and self.Free:
                        self.twisting = True
                        self.Free = False
                        self.t = 0
                        self.lock = True
                        self.twist_queue_count = 2
                        self.current_action = "Twisting (1/3)"

                elif command == "stab_off":
                    self.cg_stabilization_enabled = False
                    self.imu_stabilization_enabled = False
                    print("Ð¡ÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ñ Ð¿Ð¾Ð»Ð½Ð¾ÑÑÑÑ Ð¾ÑÐºÐ»ÑÑÐµÐ½Ð°")

                elif command == "stab_on":
                    self.cg_stabilization_enabled = True
                    self.imu_stabilization_enabled = True
                    print("Ð¡ÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ñ Ð²ÐºÐ»ÑÑÐµÐ½Ð°")

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

                elif command == "photo":
                    success = self.capture_photo()
                    if success:
                        self.current_action = "Photo captured"
                    else:
                        self.current_action = "Photo capture failed"

                else:
                    print(f"Unknown command: {command}")
                    print(
                        "Available commands: walk, sit, lie, twist, pee, stop, forward, backward, left, right, turn_left, turn_right, paw_left, paw_right, paw_down, move, anim, trot, imu, photo, quit")


    # ==================== MAIN ROBOTIC CYCLE ====================

    def main_loop(self):
        print("Starting main loop... Use console to control the robot.")
        while self.continuer:
            self.clock.tick(60) #self.clock.tick(30)

            if not hasattr(self, 'frame_counter'):
                self.frame_counter = 0
            self.frame_counter += 1

            #if self.frame_counter % 100 == 0 and self.walking:
                #print(f"=== ÐÑÐ»Ð°Ð´Ð¾ÑÐ½Ð°Ñ Ð¸Ð½ÑÐ¾ÑÐ¼Ð°ÑÐ¸Ñ (ÐºÐ°Ð´Ñ {self.frame_counter}) ===")
                #print(f"ÐÐ¾Ð·Ð¸ÑÐ¸Ð¸ Ð½Ð¾Ð³ (Z): LF={self.pos[2]:.1f}, RF={self.pos[5]:.1f}, RR={self.pos[8]:.1f}, LR={self.pos[11]:.1f}")
                #print(f"Ð£Ð³Ð»Ñ IMU: roll={self.Angle[0]*180/pi:.1f}Â°, pitch={self.Angle[1]*180/pi:.1f}Â°")
                #if hasattr(self, 'CGabs'):
                #    print(f"Ð¦ÐµÐ½ÑÑ ÑÑÐ¶ÐµÑÑÐ¸: X={self.CGabs[0]:.1f}, Y={self.CGabs[1]:.1f}")

            # ÐÑÐ¸ÑÐ°ÐµÐ¼ ÑÐºÑÐ°Ð½ Ð² Ð½Ð°ÑÐ°Ð»Ðµ ÐºÐ°Ð¶Ð´Ð¾Ð³Ð¾ ÐºÐ°Ð´ÑÐ°
            self.screen.fill(self.BLACK)

            # ÐÑÐ¾Ð±ÑÐ°Ð¶Ð°ÐµÐ¼ Ð¸Ð½ÑÐ¾ÑÐ¼Ð°ÑÐ¸Ñ Ð¾ ÑÐ¾ÑÑÐ¾ÑÐ½Ð¸Ð¸
            status_font = pygame.font.SysFont('Corbel', 24)
            status_text = status_font.render(f"State: {self.current_action}", True, self.WHITE)
            self.screen.blit(status_text, (10, 10))

            # ÐÑÐ¾Ð±ÑÐ°Ð¶Ð°ÐµÐ¼ Ð¸Ð½ÑÐ¾ÑÐ¼Ð°ÑÐ¸Ñ Ð¾ ÑÐµÐ¶Ð¸Ð¼Ð°Ñ
            anim_text = status_font.render(f"Animation: {'ON' if self.anim_flag else 'OFF'}", True, self.WHITE)
            move_text = status_font.render(f"Movement: {'ON' if self.move_flag else 'OFF'}", True, self.WHITE)
            self.screen.blit(anim_text, (10, 40))
            self.screen.blit(move_text, (10, 70))

            self.process_console_commands()

            # ---- Sensor Reading (every 10 frames to reduce overhead) ----
            if self.frame_counter % 10 == 0:
                self.read_sensors()

                # Handle obstacle avoidance
                if self.walking or hasattr(self, 'avoiding_obstacle') and self.avoiding_obstacle:
                    self.handle_obstacle_avoidance()

            # ---- Check paw hold timer ----
            self.check_paw_hold_timer()

            # ---- Logging (every 30 frames) ----
            if self.frame_counter % 30 == 0:
                self.log_state()

            #Ð¿Ð»Ð°Ð²Ð½Ð¾Ðµ Ð¸Ð·Ð¼ÐµÐ½ÐµÐ½Ð¸Ðµ ÑÐºÐ¾ÑÐ¾ÑÑÐ¸
            if self.walking and hasattr(self, 'target_speed'):
                # ÐÐ»Ð°Ð²Ð½Ð¾Ðµ Ð´Ð¾ÑÑÐ¸Ð¶ÐµÐ½Ð¸Ðµ ÑÐµÐ»ÐµÐ²Ð¾Ð¹ ÑÐºÐ¾ÑÐ¾ÑÑÐ¸
                speed_diff = self.target_speed - self.walking_speed
                if abs(speed_diff) > 1:
                    self.walking_speed += speed_diff * self.speed_smoothing #0.1  # ÐÐ»Ð°Ð²Ð½Ð¾Ðµ Ð¸Ð·Ð¼ÐµÐ½ÐµÐ½Ð¸Ðµ

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
                    if self._strafe_trot_applied and not self.trot:
                        self.stepl = self.stepl4
                        self.h_amp = self.h_amp4
                        self.v_amp = self.v_amp4
                        self.tstep = self.tstep4
                        self._strafe_trot_applied = False

                elif self.current_movement_command == "backward":
                    self.walking_speed = 100
                    self.walking_direction = self.DIR_BACKWARD
                    self.steering = 1e6
                    self.cw = 1
                    if self._strafe_trot_applied and not self.trot:
                        self.stepl = self.stepl4
                        self.h_amp = self.h_amp4
                        self.v_amp = self.v_amp4
                        self.tstep = self.tstep4
                        self._strafe_trot_applied = False

                elif self.current_movement_command == "left":
                    self.walking_speed = 5  # 50
                    self.walking_direction = self.DIR_LEFT
                    self.steering = 1e6
                    self.cw = 1
                    if not self.trot:
                        self.stepl = self.stepl2
                        self.h_amp = self.h_amp2
                        self.v_amp = self.v_amp2
                        self.tstep = self.tstep2
                        self._strafe_trot_applied = True

                elif self.current_movement_command == "right":
                    self.walking_speed = 5  # 50
                    self.walking_direction = self.DIR_RIGHT
                    self.steering = 1e6
                    self.cw = 1
                    if not self.trot:
                        self.stepl = self.stepl2
                        self.h_amp = self.h_amp2
                        self.v_amp = self.v_amp2
                        self.tstep = self.tstep2
                        self._strafe_trot_applied = True

                elif self.current_movement_command == "turn_left":
                    self.walking_speed = 100
                    self.walking_direction = 0
                    self.steering = 80  # 1000
                    self.cw = 1
                    if self._strafe_trot_applied and not self.trot:
                        self.stepl = self.stepl4
                        self.h_amp = self.h_amp4
                        self.v_amp = self.v_amp4
                        self.tstep = self.tstep4
                    self._strafe_trot_applied = False

                elif self.current_movement_command == "turn_right":
                    self.walking_speed = 100
                    self.walking_direction = 0
                    self.steering = 80  # 1000
                    self.cw = -1
                    if self._strafe_trot_applied and not self.trot:
                        self.stepl = self.stepl4
                        self.h_amp = self.h_amp4
                        self.v_amp = self.v_amp4
                        self.tstep = self.tstep4
                    self._strafe_trot_applied = False

                elif self.current_movement_command == "stop":
                    self.walking_speed = 0.0
                    self.walking_direction = 0
                    self.steering = 1e6
                    self.cw = 1
                    if self._strafe_trot_applied and not self.trot:
                        self.stepl = self.stepl4
                        self.h_amp = self.h_amp4
                        self.v_amp = self.v_amp4
                        self.tstep = self.tstep4
                        self._strafe_trot_applied = False

                # Execute walking command
                self.pos = self.Spot.start_walk_stop(self.track, self.x_offset, self.steering, self.walking_direction, self.cw,
                                           self.walking_speed, self.v_amp, self.height, self.stepl, self.t, self.tstep,
                                           self.theta_spot, self.x_spot, self.y_spot, self.z_spot, 3 + self.trans)
                # Ð main_loop Ð¿Ð¾ÑÐ»Ðµ Ð²ÑÑÐ¸ÑÐ»ÐµÐ½Ð¸Ñ self.pos:
                if self.prev_pos is None:
                    self.prev_pos = copy.deepcopy(self.pos)
                else:
                    # ÐÐ»Ð°Ð²Ð½Ð°Ñ Ð¸Ð½ÑÐµÑÐ¿Ð¾Ð»ÑÑÐ¸Ñ Ð¼ÐµÐ¶Ð´Ñ ÐºÐ°Ð´ÑÐ°Ð¼Ð¸
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

                # Ð¤ÐÐÐ¬Ð¢Ð ÐÐ¦ÐÐ¯ ÐÐÐÐÐ¦ÐÐ ÐÐÐ ÐÐ£Ð¡Ð (ÑÐ¾Ð»ÑÐºÐ¾ Ð¿ÑÐ¸ ÑÐ¾Ð´ÑÐ±Ðµ)
                if self.walking:
                    if not hasattr(self, 'filtered_body_x'):
                        # ÐÐ½Ð¸ÑÐ¸Ð°Ð»Ð¸Ð·Ð¸ÑÑÐµÐ¼ ÑÐ¸Ð»ÑÑÑÑ ÑÐµÐºÑÑÐ¸Ð¼Ð¸ Ð·Ð½Ð°ÑÐµÐ½Ð¸ÑÐ¼Ð¸
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

                        # ÐÐ ÐÐÐÐÐ¯ÐÐ ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ñ ÑÐ¾Ð»ÑÐºÐ¾ ÐµÑÐ»Ð¸ ÑÐ°ÑÑÐµÑ Ð¦Ð¢ ÑÑÐ¿ÐµÑÐµÐ½
                        self.pos = self.stabilize_body_cg_imu(self.pos, self.CGabs, self.Angle)

                        if hasattr(self, 'check_leg_positions_reachable'):
                            try:
                                self.pos = self.check_leg_positions_reachable(self.pos)
                            except Exception as e:
                                print(f"ÐÑÐ¸Ð±ÐºÐ° Ð¿ÑÐ¸ Ð¿ÑÐ¾Ð²ÐµÑÐºÐµ Ð´Ð¾ÑÑÐ¸Ð¶Ð¸Ð¼Ð¾ÑÑÐ¸ Ð¿Ð¾Ð·Ð¸ÑÐ¸Ð¹ Ð² ÑÐ¾Ð´ÑÐ±Ðµ: {e}")

                        # ÐÐ±Ð½Ð¾Ð²Ð»ÑÐµÐ¼ Ð¿ÐµÑÐµÐ¼ÐµÐ½Ð½ÑÐµ Ð¿Ð¾ÑÐ»Ðµ ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ð¸
                        self.theta_spot = self.pos[12]
                        self.x_spot = self.pos[13]
                        self.y_spot = self.pos[14]
                        self.z_spot = self.pos[15]

                    except Exception as e:
                        print(f"ÐÑÐ¸Ð±ÐºÐ° ÑÐ°ÑÑÐµÑÐ° ÑÐµÐ½ÑÑÐ° ÑÑÐ¶ÐµÑÑÐ¸: {e}")
                # ÐÐ±Ð½Ð¾Ð²Ð»ÑÐµÐ¼ Ð¿ÐµÑÐµÐ¼ÐµÐ½Ð½ÑÐµ Ð¿Ð¾ÑÐ»Ðµ Ð²ÑÐµÑ Ð¼Ð¾Ð´Ð¸ÑÐ¸ÐºÐ°ÑÐ¸Ð¹
                self.theta_spot = self.pos[12]
                self.x_spot = self.pos[13]
                self.y_spot = self.pos[14]
                self.z_spot = self.pos[15]

                if self.walking and self.frame_counter % 50 == 0:
                    norm_x, norm_y = self.normalize_cg_offset()
                #    print(f"CG Ð½Ð¾ÑÐ¼: X={norm_x:.2f}, Y={norm_y:.2f} | "
                #          f"Ð¡Ð¼ÐµÑÐµÐ½Ð¸Ðµ ÑÐµÐ»Ð°: X={getattr(self, 'body_shift_x', 0):.1f}, "
                #          f"Y={getattr(self, 'body_shift_y', 0):.1f}")

                # Check if walking should stop
                if self.stop and self.t >= (self.tstop + 1):  # ?????????? ???????
                    # t > (tstop + 1 - tstep)
                    # ?? ???????, ??? t ????????? tstop + 1 (????? ?????)
                    self.lock = False
                    self.stop = False
                    self.walking = False
                    print("===> Switching to recovering!")
                    self.recovering = True  # Activate recovery state
                    self.t = 0

                    # Capture current pose to blend back to Stand
                    self.temp_start_pos = [self.pos[12][0], self.pos[12][1], self.pos[12][2], self.pos[13][1], self.pos[14][1], self.pos[15][1]]

                    self.current_action = "Realigning to Stand..."
                    print("=== WALKING DONE -> REALIGNING TO STAND ===")

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
                        print("=== STANDING COMPLETED ===")
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

                        # ---- FK Ð´Ð»Ñ Ð°Ð½Ð¸Ð¼Ð°ÑÐ¸Ð¸ ----
                        self.legrf = self.Spot.FK(self.thetarf, -1)
                        self.leglf = self.Spot.FK(self.thetalf, 1)

                        self.xlegrf = self.Spot.xrf + self.pos[3]
                        self.ylegrf = self.Spot.yrf + self.pos[4]
                        self.zlegrf = self.pos[5]

                        self.xleglf = self.Spot.xlf + self.pos[0]
                        self.yleglf = self.Spot.ylf + self.pos[1]
                        self.zleglf = self.pos[2]

                        # ---- ÐÐ±Ð½Ð¾Ð²Ð»ÑÐµÐ¼ Ð°Ð±ÑÐ¾Ð»ÑÑÐ½ÑÐµ Ð¿Ð¾Ð·Ð¸ÑÐ¸Ð¸ ÑÐµÐ»Ð° ----
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
                    if self.twist_queue_count > 0:
                        self.twist_queue_count -= 1
                        self.command_queue.insert(0, "twist")
                        self.current_action = f"Twisting ({3 - self.twist_queue_count}/3)"
                    else:
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
                # print(f"Recover t={t}, stance={stance}, transtep={transtep}")
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
                    self.current_action = "Stand recovery complete"
                    print("=== STAND RECOVERY COMPLETED ===")

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

            # ÐÐ¾ÑÐ»Ðµ Ð¾ÑÐ¾Ð±ÑÐ°Ð¶ÐµÐ½Ð¸Ñ ÐºÐ½Ð¾Ð¿Ð¾Ðº Ð´Ð¾Ð±Ð°Ð²ÑÑÐµ:
            if self.anim_flag and hasattr(self, 'CGabs'):
                # Ð Ð¸ÑÑÐµÐ¼ ÑÐµÐ½ÑÑ ÑÑÐ¶ÐµÑÑÐ¸
                cg_screen_x = int(300 + self.CGabs[0] / 2)
                cg_screen_y = int(300 - self.CGabs[1] / 2)
                pygame.draw.circle(self.screen, self.RED, (cg_screen_x, cg_screen_y), 5)

                # Ð Ð¸ÑÑÐµÐ¼ Ð¿ÑÐ¾ÐµÐºÑÐ¸Ñ ÑÐµÐ½ÑÑÐ° ÐºÐ¾ÑÐ¿ÑÑÐ°
                body_screen_x = int(300 + self.x_spot[1] / 2)
                body_screen_y = int(300 - self.y_spot[1] / 2)
                pygame.draw.circle(self.screen, self.BLUE, (body_screen_x, body_screen_y), 3)

                # ÐÐ¸Ð½Ð¸Ñ Ð¾Ñ ÑÐµÐ½ÑÑÐ° ÐºÐ¾ÑÐ¿ÑÑÐ° Ðº Ð¦Ð¢
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
        self.cleanup()
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
        """ÐÐ¾ÑÐ¼Ð°Ð»Ð¸Ð·ÑÐµÑ ÑÐ¼ÐµÑÐµÐ½Ð¸Ðµ Ð¦Ð¢ Ð¾ÑÐ½Ð¾ÑÐ¸ÑÐµÐ»ÑÐ½Ð¾ ÐºÐ¾ÑÐ¿ÑÑÐ°"""
        if not hasattr(self, 'CGabs'):
            return 0, 0

        # Ð¡Ð¼ÐµÑÐµÐ½Ð¸Ðµ Ð¦Ð¢ Ð¾Ñ ÑÐµÐ½ÑÑÐ° ÐºÐ¾ÑÐ¿ÑÑÐ°
        offset_x = self.CGabs[0] - self.x_spot[1]
        offset_y = self.CGabs[1] - self.y_spot[1]

        # ÐÐ¾ÑÐ¼Ð°Ð»Ð¸Ð·ÑÐµÐ¼ (Ð¿ÑÐ¸Ð²Ð¾Ð´Ð¸Ð¼ Ðº Ð´Ð¸Ð°Ð¿Ð°Ð·Ð¾Ð½Ñ -1..1)
        norm_x = offset_x / 100.0  # 100Ð¼Ð¼ - Ð¼Ð°ÑÑÑÐ°Ð±Ð½ÑÐ¹ ÐºÐ¾ÑÑÑÐ¸ÑÐ¸ÐµÐ½Ñ
        norm_y = offset_y / 100.0

        # ÐÐ³ÑÐ°Ð½Ð¸ÑÐ¸Ð²Ð°ÐµÐ¼
        norm_x = max(min(norm_x, 1.0), -1.0)
        norm_y = max(min(norm_y, 1.0), -1.0)

        return norm_x, norm_y

    def stabilize_body_cg_imu(self, pos, cg_abs, imu_angles):
        """Ð¡ÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ñ Ñ ÐºÐ¾ÑÑÐµÐºÑÐ¸ÑÐ¾Ð²ÐºÐ¾Ð¹ ÐºÐ¾ÑÐ¿ÑÑÐ°, Ð° Ð½Ðµ Ð½Ð¾Ð³"""

        if not self.walking or not self.cg_stabilization_enabled:
            return pos

        try:
            # 1. Ð¡Ð½Ð°ÑÐ°Ð»Ð° ÐºÐ¾ÑÑÐµÐºÑÐ¸ÑÑÐµÐ¼ ÐÐÐ ÐÐ£Ð¡, Ð° Ð½Ðµ Ð½Ð¾Ð³Ð¸
            if hasattr(self, 'CGabs'):
                # Ð¡Ð¼ÐµÑÐµÐ½Ð¸Ðµ Ð¦Ð¢ Ð¾ÑÐ½Ð¾ÑÐ¸ÑÐµÐ»ÑÐ½Ð¾ ÑÐµÐ½ÑÑÐ° ÐºÐ¾ÑÐ¿ÑÑÐ°
                cg_offset_x = self.CGabs[0] - self.x_spot[1]  # Ð¡Ð¼ÐµÑÐµÐ½Ð¸Ðµ Ð²Ð¿ÐµÑÐµÐ´/Ð½Ð°Ð·Ð°Ð´
                cg_offset_y = self.CGabs[1] - self.y_spot[1]  # Ð¡Ð¼ÐµÑÐµÐ½Ð¸Ðµ Ð²Ð»ÐµÐ²Ð¾/Ð²Ð¿ÑÐ°Ð²Ð¾

                # Ð¦ÐµÐ»ÐµÐ²Ð¾Ðµ ÑÐ¼ÐµÑÐµÐ½Ð¸Ðµ ÐºÐ¾ÑÐ¿ÑÑÐ° Ð´Ð»Ñ ÐºÐ¾Ð¼Ð¿ÐµÐ½ÑÐ°ÑÐ¸Ð¸ (Ð¼ÑÐ³ÐºÐ°Ñ ÐºÐ¾ÑÑÐµÐºÑÐ¸Ñ)
                target_body_shift_x = -cg_offset_x * 0.3  # ÐÐ¾Ð¼Ð¿ÐµÐ½ÑÐ¸ÑÑÐµÐ¼ 30% ÑÐ¼ÐµÑÐµÐ½Ð¸Ñ
                target_body_shift_y = -cg_offset_y * 0.3

                # ÐÑÐ¸Ð¼ÐµÐ½ÑÐµÐ¼ Ð¿Ð»Ð°Ð²Ð½ÑÑ ÑÐ¸Ð»ÑÑÑÐ°ÑÐ¸Ñ Ðº ÑÐ¼ÐµÑÐµÐ½Ð¸Ñ ÐºÐ¾ÑÐ¿ÑÑÐ°
                if not hasattr(self, 'body_shift_x'):
                    self.body_shift_x = 0
                    self.body_shift_y = 0

                self.body_shift_x = self.body_shift_x * 0.8 + target_body_shift_x * 0.2
                self.body_shift_y = self.body_shift_y * 0.8 + target_body_shift_y * 0.2

                # ÐÐ³ÑÐ°Ð½Ð¸ÑÐ¸Ð²Ð°ÐµÐ¼ ÑÐ¼ÐµÑÐµÐ½Ð¸Ðµ ÐºÐ¾ÑÐ¿ÑÑÐ°
                max_body_shift = 20  # ÐÐ°ÐºÑÐ¸Ð¼Ð°Ð»ÑÐ½Ð¾Ðµ ÑÐ¼ÐµÑÐµÐ½Ð¸Ðµ ÐºÐ¾ÑÐ¿ÑÑÐ° (Ð¼Ð¼)
                self.body_shift_x = max(min(self.body_shift_x, max_body_shift), -max_body_shift)
                self.body_shift_y = max(min(self.body_shift_y, max_body_shift), -max_body_shift)

                # ÐÐ¾ÑÑÐµÐºÑÐ¸ÑÑÐµÐ¼ Ð¿Ð¾Ð·Ð¸ÑÐ¸Ñ ÐºÐ¾ÑÐ¿ÑÑÐ° (Ð² ÑÑÑÑÐºÑÑÑÐµ pos)
                # ÐÐ¾ÑÐ¿ÑÑ Ð½Ð°ÑÐ¾Ð´Ð¸ÑÑÑ Ð² Ð¸Ð½Ð´ÐµÐºÑÐ°Ñ 13-15, ÑÐ»ÐµÐ¼ÐµÐ½Ñ [1] - ÑÑÐ¾ ÑÐµÐ»Ð¾
                pos[13][1] = self.x_spot[1] + self.body_shift_x  # X ÐºÐ¾ÑÐ¿ÑÑÐ°
                pos[14][1] = self.y_spot[1] + self.body_shift_y  # Y ÐºÐ¾ÑÐ¿ÑÑÐ°

            # 2. ÐÐ¾Ð¼Ð¿ÐµÐ½ÑÐ°ÑÐ¸Ñ Ð½Ð°ÐºÐ»Ð¾Ð½Ð° Ð¿Ð¾ IMU (Ð±Ð¾Ð»ÐµÐµ Ð¼ÑÐ³ÐºÐ°Ñ)
            if self.IMU_Comp and self.imu_stabilization_enabled:
                roll = imu_angles[0] * 0.5  # Ð£Ð¼ÐµÐ½ÑÑÐµÐ½Ð½ÑÐ¹ ÐºÐ¾ÑÑÑÐ¸ÑÐ¸ÐµÐ½Ñ
                pitch = imu_angles[1] * 0.5

                # ÐÐµÐ³ÐºÐ°Ñ ÐºÐ¾ÑÑÐµÐºÑÐ¸Ñ Ð²ÑÑÐ¾ÑÑ Ð½Ð¾Ð³ Ð´Ð»Ñ ÐºÐ¾Ð¼Ð¿ÐµÐ½ÑÐ°ÑÐ¸Ð¸ Ð½Ð°ÐºÐ»Ð¾Ð½Ð°
                # ÐÐ½Ð´ÐµÐºÑÑ Z: LF=2, RF=5, RR=8, LR=11
                z_corrections = [
                    -5 * pitch - 5 * roll,   # LF (Ð¿ÐµÑÐµÐ´-Ð»ÐµÐ²)
                    -5 * pitch + 5 * roll,   # RF (Ð¿ÐµÑÐµÐ´-Ð¿ÑÐ°Ð²)
                    5 * pitch + 5 * roll,    # RR (Ð·Ð°Ð´-Ð¿ÑÐ°Ð²)
                    5 * pitch - 5 * roll     # LR (Ð·Ð°Ð´-Ð»ÐµÐ²)
                ]

                # ÐÑÐ¸Ð¼ÐµÐ½ÑÐµÐ¼ Ð¾ÑÐµÐ½Ñ Ð½ÐµÐ±Ð¾Ð»ÑÑÐ¸Ðµ ÐºÐ¾ÑÑÐµÐºÑÐ¸ÑÐ¾Ð²ÐºÐ¸
                z_indices = [2, 5, 8, 11]
                for i, z_idx in enumerate(z_indices):
                    # Ð¡Ð¾ÑÑÐ°Ð½ÑÐµÐ¼ Ð¾ÑÐ¸Ð³Ð¸Ð½Ð°Ð»ÑÐ½ÑÑ Ð°Ð¼Ð¿Ð»Ð¸ÑÑÐ´Ñ Ð´Ð²Ð¸Ð¶ÐµÐ½Ð¸Ñ
                    original_z = pos[z_idx]

                    # ÐÐ¾Ð±Ð°Ð²Ð»ÑÐµÐ¼ Ð¾ÑÐµÐ½Ñ Ð½ÐµÐ±Ð¾Ð»ÑÑÑÑ ÐºÐ¾ÑÑÐµÐºÑÐ¸ÑÐ¾Ð²ÐºÑ
                    correction = z_corrections[i] * 0.1  # ÐÑÐµÐ³Ð¾ 10% Ð¾Ñ ÑÐ°ÑÑÐµÑÐ°
                    pos[z_idx] = original_z + correction

                    # ÐÐ³ÑÐ°Ð½Ð¸ÑÐ¸Ð²Ð°ÐµÐ¼, ÑÑÐ¾Ð±Ñ Ð½Ðµ Ð¼ÐµÑÐ°ÑÑ Ð¾ÑÐ½Ð¾Ð²Ð½Ð¾Ð¹ Ð°Ð½Ð¸Ð¼Ð°ÑÐ¸Ð¸ ÑÐ¾Ð´ÑÐ±Ñ
                    max_z_adjustment = 10  # ÐÐ°ÐºÑÐ¸Ð¼Ð°Ð»ÑÐ½Ð°Ñ ÐºÐ¾ÑÑÐµÐºÑÐ¸ÑÐ¾Ð²ÐºÐ° 10Ð¼Ð¼
                    if abs(pos[z_idx] - original_z) > max_z_adjustment:
                        pos[z_idx] = original_z + (max_z_adjustment if correction > 0 else -max_z_adjustment)

            return pos

        except Exception as e:
            print(f"ÐÑÐ¸Ð±ÐºÐ° ÑÑÐ°Ð±Ð¸Ð»Ð¸Ð·Ð°ÑÐ¸Ð¸: {e}")
            return pos


    def check_leg_positions_reachable(self, pos):
        """ÐÑÐ¾Ð²ÐµÑÑÐµÑ, ÑÑÐ¾ Ð²ÑÐµ Ð¿Ð¾Ð·Ð¸ÑÐ¸Ð¸ Ð½Ð¾Ð³ Ð´Ð¾ÑÑÐ¸Ð¶Ð¸Ð¼Ñ"""
        try:
            # ÐÐ¾Ð»ÐµÐµ Ð¼ÑÐ³ÐºÐ¸Ðµ Ð¾Ð³ÑÐ°Ð½Ð¸ÑÐµÐ½Ð¸Ñ Ð´Ð»Ñ ÑÐ¾Ð´ÑÐ±Ñ
            if self.walking:
                min_z = -60  # ÐÑÐ»Ð¾ -80
                max_z = 40   # ÐÑÐ»Ð¾ 30
                max_xy = 120 # ÐÑÐ»Ð¾ 100
            else:
                min_z = -80
                max_z = 30
                max_xy = 100

            for i in range(4):
                # ÐÐ½Ð´ÐµÐºÑÑ: LF(0,1,2), RF(3,4,5), RR(6,7,8), LR(9,10,11)
                x_idx = i*3
                y_idx = i*3 + 1
                z_idx = i*3 + 2

                # ÐÐ¾Ð»ÑÑÐ°ÐµÐ¼ ÐºÐ¾Ð½ÑÑÑÑÐºÑÐ¸Ð²Ð½ÑÐµ ÑÐ¼ÐµÑÐµÐ½Ð¸Ñ Ð´Ð»Ñ ÑÑÐ¾Ð¹ Ð½Ð¾Ð³Ð¸
                if i == 0:  # LF
                    x_offset, y_offset, z_offset = self.xtlf, self.ytlf, self.ztlf
                elif i == 1:  # RF
                    x_offset, y_offset, z_offset = self.xtrf, self.ytrf, self.ztrf
                elif i == 2:  # RR
                    x_offset, y_offset, z_offset = self.xtrr, self.ytrr, self.ztrr
                else:  # LR
                    x_offset, y_offset, z_offset = self.xtlr, self.ytlr, self.ztlr

                # ÐÐ±ÑÐ¾Ð»ÑÑÐ½ÑÐµ ÐºÐ¾Ð¾ÑÐ´Ð¸Ð½Ð°ÑÑ
                x_abs = pos[x_idx] + x_offset
                y_abs = pos[y_idx] + y_offset
                z_abs = pos[z_idx] + z_offset

                # ÐÐ¾ÑÑÐµÐºÑÐ¸ÑÑÐµÐ¼ ÑÐ¾Ð»ÑÐºÐ¾ ÐµÑÐ»Ð¸ ÑÐ¸Ð»ÑÐ½Ð¾ Ð²ÑÑÐ»Ð¸ Ð·Ð° Ð¿ÑÐµÐ´ÐµÐ»Ñ
                if z_abs < min_z:
                    # ÐÐ¾Ð´Ð½Ð¸Ð¼Ð°ÐµÐ¼ Ð½Ð¾Ð³Ñ, Ð½Ð¾ Ð½Ðµ ÑÐ»Ð¸ÑÐºÐ¾Ð¼ ÑÐµÐ·ÐºÐ¾
                    correction = (min_z - z_abs) * 0.3  # ÐÐ¾ÑÑÐµÐºÑÐ¸ÑÑÐµÐ¼ ÑÐ¾Ð»ÑÐºÐ¾ 30%
                    pos[z_idx] += correction
                    #print(f"ÐÐ¾ÑÑÐµÐºÑÐ¸Ñ Ð½Ð¾Ð³Ð¸ {i}: z={z_abs:.1f} -> {pos[z_idx]+z_offset:.1f}")
                elif z_abs > max_z:
                    correction = (max_z - z_abs) * 0.3
                    pos[z_idx] += correction
                    #print(f"ÐÐ¾ÑÑÐµÐºÑÐ¸Ñ Ð½Ð¾Ð³Ð¸ {i}: z={z_abs:.1f} -> {pos[z_idx]+z_offset:.1f}")

                # ÐÑÐ¾Ð²ÐµÑÑÐµÐ¼ Ð³Ð¾ÑÐ¸Ð·Ð¾Ð½ÑÐ°Ð»ÑÐ½ÑÐµ ÑÐ¼ÐµÑÐµÐ½Ð¸Ñ (ÑÐ¾Ð»ÑÐºÐ¾ Ð¿ÑÐµÐ´ÑÐ¿ÑÐµÐ¶Ð´ÐµÐ½Ð¸Ðµ)
                if abs(x_abs) > max_xy or abs(y_abs) > max_xy:
                    print(f"ÐÐ½Ð¸Ð¼Ð°Ð½Ð¸Ðµ: Ð½Ð¾Ð³Ð° {i} Ð±Ð»Ð¸Ð·ÐºÐ° Ðº Ð¿ÑÐµÐ´ÐµÐ»Ñ: x={x_abs:.1f}, y={y_abs:.1f}")

            return pos

        except Exception as e:
            print(f"ÐÑÐ¸Ð±ÐºÐ° Ð² check_leg_positions_reachable: {e}")
            return pos

    # moving servos
    def moving(self, pos, move):
        # ÐÐ½Ð¸ÑÐ¸Ð°Ð»Ð¸Ð·Ð°ÑÐ¸Ñ ÑÐµÐºÑÑÐ¸Ñ ÑÐ³Ð»Ð¾Ð² ÑÐµÑÐ²Ð¾Ð¿ÑÐ¸Ð²Ð¾Ð´Ð¾Ð² Ð¿ÑÐ¸ Ð¿ÐµÑÐ²Ð¾Ð¼ Ð²ÑÐ·Ð¾Ð²Ðµ
        if not hasattr(self, 'current_servo_angles'):
            self.current_servo_angles = [0.0] * 12

        # ÐÑÑÐ¸ÑÐ»ÑÐµÐ¼ ÑÐµÐ»ÐµÐ²ÑÐµ ÑÐ³Ð»Ñ Ñ Ð¿Ð¾Ð¼Ð¾ÑÑÑ Ð¾Ð±ÑÐ°ÑÐ½Ð¾Ð¹ ÐºÐ¸Ð½ÐµÐ¼Ð°ÑÐ¸ÐºÐ¸
        thetalf_reply = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                    pos[0] + self.xtlf, pos[1] + self.ytlf, pos[2] + self.ztlf, 1)
        thetarf_reply = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                    pos[3] + self.xtrf, pos[4] + self.ytrf, pos[5] + self.ztrf, -1)
        thetarr_reply = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                    pos[6] + self.xtrr, pos[7] + self.ytrr, pos[8] + self.ztrr, -1)
        thetalr_reply = self.Spot.IK(self.Spot.L0, self.Spot.L1, self.Spot.L2, self.Spot.d,
                                    pos[9] + self.xtlr, pos[10] + self.ytlr, pos[11] + self.ztlr, 1)

        if move:
            # ÐÐÐÐÐ¢ÐÐÐÐÐ ÑÐ³Ð»Ð°Ð¶Ð¸Ð²Ð°Ð½Ð¸Ðµ: Ð¼ÐµÐ½ÑÑÐµ Ð¿ÑÐ¸ Ð¾ÑÑÐ°Ð½Ð¾Ð²ÐºÐµ, Ð±Ð¾Ð»ÑÑÐµ Ð¿ÑÐ¸ ÑÐ¾Ð´ÑÐ±Ðµ
            if self.walking and not self.stop:
                servo_smoothing = 0.4  # ÐÐ»Ð°Ð²Ð½Ð¾ÑÑÑ Ð¿ÑÐ¸ ÑÐ¾Ð´ÑÐ±Ðµ
            else:
                servo_smoothing = 0.1  # ÐÐµÐ½ÑÑÐµ Ð¿Ð»Ð°Ð²Ð½Ð¾ÑÑÐ¸ Ð¿ÑÐ¸ Ð¾ÑÑÐ°Ð½Ð¾Ð²ÐºÐµ/Ð¿ÐµÑÐµÑÐ¾Ð´Ð°Ñ

            # ÐÐ±ÑÐ°Ð±Ð°ÑÑÐ²Ð°ÐµÐ¼ Ð»ÐµÐ²ÑÑ Ð¿ÐµÑÐµÐ´Ð½ÑÑ Ð½Ð¾Ð³Ñ
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

            # ÐÐ±ÑÐ°Ð±Ð°ÑÑÐ²Ð°ÐµÐ¼ Ð¿ÑÐ°Ð²ÑÑ Ð¿ÐµÑÐµÐ´Ð½ÑÑ Ð½Ð¾Ð³Ñ
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

            # ÐÐ±ÑÐ°Ð±Ð°ÑÑÐ²Ð°ÐµÐ¼ Ð¿ÑÐ°Ð²ÑÑ Ð·Ð°Ð´Ð½ÑÑ Ð½Ð¾Ð³Ñ
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

            # ÐÐ±ÑÐ°Ð±Ð°ÑÑÐ²Ð°ÐµÐ¼ Ð»ÐµÐ²ÑÑ Ð·Ð°Ð´Ð½ÑÑ Ð½Ð¾Ð³Ñ
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

    # ==================== CLEANUP ====================

    def cleanup(self):
        """Clean up resources on exit"""
        print("Cleaning up resources...")

        # Close TCP server
        if self.tcp_server_socket:
            try:
                self.tcp_server_socket.close()
            except:
                pass

        # Stop camera
        if self.camera and hasattr(self.camera, 'stop'):
            try:
                self.camera.stop()
            except:
                pass

        # Clean up GPIO
        try:
            GPIO.cleanup()
        except:
            pass

        print("Cleanup complete")

# ======================

#if __name__ == "__main__":
#controller = SpotMicroController()
#    print("=== Created controller ===")
#controller.start()
