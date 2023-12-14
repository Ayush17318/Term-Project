# -*- coding: utf-8 -*-
"""
Created on Mon Nov 13 13:06:34 2023

@author: akank
"""

from pyb import Pin, Timer
# from time import sleep_ms
from array import array
from micropython import alloc_emergency_exception_buf


class L6206:
    def __init__ (self, PWM_tim, IN1_pin, IN2_pin, EN_pin):

        self.tim = PWM_tim
        self.PWM_1 = self.tim.channel(3,pin = IN1_pin, mode = Timer.PWM, pulse_width_percent=0)

        self.IN2 = Pin(IN2_pin, mode = Pin.OUT_PP)
        self.EN = Pin(EN_pin, mode = Pin.OUT_PP)
    def set_duty (self, duty):

        self.duty=duty

        if duty >= 0.0:
            self.IN2.low()
            self.PWM_1.pulse_width_percent(abs(duty))
        else:
            self.IN2.high()
            self.PWM_1.pulse_width_percent(abs(duty))
    def enable(self):
        self.EN.high()
    def disable(self):
        self.EN.low()