#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

# ============================================
# КОНФИГУРАЦИЯ ПИНОВ
# ============================================
# Важно: используем нумерацию BCM (по номеру GPIO)
GPIO.setmode(GPIO.BCM)

# Пины для первого датчика (GPIO14/15 - TXD0/RXD0)
TRIG_1 = 14  # BCM14 (физический пин 8)
ECHO_1 = 15  # BCM15 (физический пин 10)

# Пины для второго датчика
TRIG_2 = 23  # BCM23 (физический пин 16)
ECHO_2 = 24  # BCM24 (физический пин 18)

# Настройка пинов
GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIG_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)

# Отключаем предупреждения, если пины уже использовались
GPIO.setwarnings(False)

# Гарантируем, что Trig в низком состоянии
GPIO.output(TRIG_1, False)
GPIO.output(TRIG_2, False)
time.sleep(0.1)  # Датчикам нужно время на инициализацию

# ============================================
# ФУНКЦИЯ ИЗМЕРЕНИЯ ДЛЯ ОДНОГО ДАТЧИКА
# ============================================
def measure_distance(trig_pin, echo_pin):
    """
    Измеряет расстояние для одного датчика HC-SR04.
    Возвращает расстояние в сантиметрах.
    При ошибке измерения возвращает -1.
    """
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)  
    GPIO.output(trig_pin, False)
    
    # время начала и ожидаем начало импульса на Echo
    start_time = time.time()
    timeout = start_time + 0.04  
    
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        if start_time > timeout:
            #print("Таймаут при ожидании начала импульса")
            return -1
    
    # время окончания и ожидаем конец импульса
    stop_time = time.time()
    timeout = stop_time + 0.04
    
    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()
        if stop_time > timeout:
            #print("Таймаут при ожидании конца импульса")
            return -1
    

    elapsed_time = stop_time - start_time
    distance_cm=elapsed_time*17150

    return distance_cm

def measure_filtered(trig_pin, echo_pin, samples=5):
    """
    Делает несколько измерений и усредняет результат.
    Игнорирует явные выбросы (значения за пределами разумного диапазона).
    """
    readings = []
    
    for _ in range(samples):
        dist = measure_distance(trig_pin, echo_pin)
        
        # filter
        if 2.0 <= dist <= 400.0:  
            readings.append(dist)
        
        time.sleep(0.01)
    
    # возвращаем среднее
    if readings:
        avg_distance = sum(readings) / len(readings)
        return avg_distance
    else:
        return -1  # измерения были ошибочными

# ============================================
# ОСНОВНОЙ ЦИКЛ ИЗМЕРЕНИЙ
# ============================================
try:
    
    while True:
        dist_1 = measure_filtered(TRIG_1, ECHO_1, samples=3)
        dist_2 = measure_filtered(TRIG_2, ECHO_2, samples=3)
        
        if dist_1 >= 0:
            print(f"Датчик 1 (14/15): {dist_1:.1f} см")
        else:
            print(f"Датчик 1 (14/15): Ошибка измерения")
        
        if dist_2 >= 0:
            print(f"Датчик 2 (23/24): {dist_2:.1f} см")
        else:
            print(f"Датчик 2 (23/24): Ошибка измерения")
        
        print("-" * 20)
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nИзмерение остановлено")

finally:
    # корректно освобождаем пины
    GPIO.cleanup()
    print("GPIO очищены. Выход.")
