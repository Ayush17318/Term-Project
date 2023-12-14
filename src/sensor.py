# -*- coding: utf-8 -*-
"""
Created on Mon Nov 13 13:45:05 2023

@author: akank
"""
from pyb import Pin, Timer
from time import sleep_ms
from array import array
from micropython import alloc_emergency_exception_buf
from motor_romi import L6206
from encoder_romi import Encoder
import pyb

L1= Pin(Pin.cpu.C0,mode= Pin.IN)
L2= Pin(Pin.cpu.C1,mode= Pin.IN)
L1_adc = pyb.ADC(L1)

idx=0 
num=0 
n=1000

while True:
    #print(L1_adc.read())
    if idx <n:
        idx+=1
        num+= L1_adc.read()
    elif idx ==n:
        if num/n > 700:
            print("black, ",num/n)
        if num/n < 600:
            print("white, ",num/n)
        idx =0
        num=0