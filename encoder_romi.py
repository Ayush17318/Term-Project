# -*- coding: utf-8 -*-
"""
Created on Mon Nov 13 13:11:21 2023

@author: akank
"""

from pyb import Pin, Timer
# from time import sleep_ms
from array import array
from micropython import alloc_emergency_exception_buf

class Encoder:
    
    def __init__(self, ENC_tim, ENC_CH_A, ENC_CH_B, auto_r):
        self.tim = ENC_tim
        self.ENC_CHA= self.tim.channel(1, pin=ENC_CH_A, mode=Timer.ENC_AB)
        self.ENC_CHB= self.tim.channel(2, pin=ENC_CH_B, mode=Timer.ENC_AB)
        #self.tim.callback(self.tim_cb)
        self.idx =0
        self.delta= 0
        self.count=0
        self.old_count=0
        self.auto_reload = auto_r
        self.limit=(self.auto_reload+1)/2
        self.neg_limit=-1*(self.auto_reload+1)/2
        self.AR1= self.auto_reload+1
        self.position =0
    
    def update(self):
        
        self.count=self.tim.counter()
        self.delta= self.count-self.old_count
        
        if self.delta>self.limit:
            self.delta-=(self.AR1)
            
        elif self.delta<self.neg_limit:
            self.delta+=(self.AR1)
        #print(self.position)
        self.position += self.delta
        self.old_count= self.count
       
    def get_position(self):
        return self.position
        
    def get_delta(self):
        
        return self.delta
    
    def zero(self):
        self.position=0
        self.delta=0
        self.count=0
        self.old_count=0
        # sleep_ms(2)