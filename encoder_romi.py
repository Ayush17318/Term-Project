"""!
@file encoder_romi.py
This file contains the class to act as the controller for romi encoders.

@author Akanksha Maddi and Ayush Kakkanat
@date   13 November 2023 Date of creation of file
@date   19 December 2023 Updated doxygen documentation
"""

from pyb import Pin, Timer
from array import array
from micropython import alloc_emergency_exception_buf

class Encoder:
    """!
    Class interfacing with the romi encoders.
    """
    
    def __init__(self, ENC_tim, ENC_CH_A, ENC_CH_B, auto_r):
        """!
        Initializes a task object, saving copies of constructor parameters and 
        initializes counters.
        
        @param ENC_tim Define the encoder timer channel on the microcontroller.
        @param ENC_CH_A Define the encoder timer channel A on the romi
               power board.
        @param ENC_CH_B Define the encoder timer channel B on the romi
               power board.
        @param auro_r The autoreload value to handle overflow and underflow.
        """
        
        self.tim = ENC_tim
        self.ENC_CHA = self.tim.channel(1, pin=ENC_CH_A, mode=Timer.ENC_AB)
        self.ENC_CHB = self.tim.channel(2, pin=ENC_CH_B, mode=Timer.ENC_AB)
        self.idx = 0
        self.delta = 0
        self.count = 0
        self.old_count = 0
        self.auto_reload = auto_r
        self.limit = (self.auto_reload + 1)/2
        self.neg_limit = -1*(self.auto_reload + 1)/2
        self.AR1 = self.auto_reload + 1
        self.position = 0
    
    def update(self):
        """!
        Update the encoder position with a counter and an auto-reload to 
        handle overflow or underflow.
        """
        
        self.count = self.tim.counter()
        self.delta = self.count - self.old_count
        
        if self.delta > self.limit:
            self.delta -= (self.AR1)
            
        elif self.delta < self.neg_limit:
            self.delta += (self.AR1)
            
        self.position += self.delta
        self.old_count = self.count
       
    def get_position(self):
        """!
        Retrieve the current encoder position.
        """
        
        return self.position
        
    def get_delta(self):
        """!
        Retrieve the change in encoder position.
        """
        return self.delta
    
    def zero(self):
        """!
        Zero the encoders.
        """
        
        self.position = 0
        self.delta = 0
        self.count = 0
        self.old_count = 0