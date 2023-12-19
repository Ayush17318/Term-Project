"""!
@file motor_romi.py
This file contains the class to act as the controller for romi motors.

@author Akanksha Maddi and Ayush Kakkanat
@date   13 November 2023 Date of creation of file
@date   19 December 2023 Updated doxygen documentation
"""

from pyb import Pin, Timer
from array import array
from micropython import alloc_emergency_exception_buf


class L6206:
    """!
    Implements the romi motor controller.
    """
    
    def __init__ (self, PWM_tim, IN1_pin, IN2_pin, EN_pin):
        """!
        Initialize a controller object to control any romi motor. Define the
        PWM logic, and the enable pin.
        
        @param PWM_tim Define the PWM timer channel on the microcontroller
        @param IN1_pin Define the first direction pin for the DC motor
        @param IN2_pin Define the second direction pin for the DC motor
        @param EN_pin Define the enable control for the DC motor
        """
        
        self.tim = PWM_tim
        self.PWM_1 = self.tim.channel(3,pin = IN1_pin, mode = Timer.PWM, 
                                      pulse_width_percent = 0)

        self.IN2 = Pin(IN2_pin, mode = Pin.OUT_PP)
        self.EN = Pin(EN_pin, mode = Pin.OUT_PP)
        
    def set_duty (self, duty):
        """!
        Set the duty cycle between -100 and 100 for the motor in either 
        direction.
        
        @param duty Define the float value for the duty cycle.
        """

        self.duty = duty

        if duty >= 0.0:
            self.IN2.low()
            self.PWM_1.pulse_width_percent(abs(duty))
            
        else:
            self.IN2.high()
            self.PWM_1.pulse_width_percent(abs(duty))
            
    def enable(self):
        """!
        Enable the motor to run.
        """
        
        self.EN.high()
        
    def disable(self):
        """!
        Disable the motor from running.
        """
        
        self.EN.low()