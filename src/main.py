"""
@file               main.py (Term Project)

@brief              file to execute the term project code

@author             Ayush Kakkanat & Akanksha Maddi

@date               13 November 2023

@latest update      11 December 2023
"""

from pyb import Pin, Timer
from time import sleep_ms
from array import array
from micropython import alloc_emergency_exception_buf
from motor_romi import L6206
from encoder_romi import Encoder
import pyb
import time
import task_share
import cotask
import random
import pyb
from pyb import USB_VCP, UART
from pyb import ExtInt, Pin

import Queues
from ClosedLoop import ClosedLoop


alloc_emergency_exception_buf(100)
turn_q_r = task_share.Queue('H', 1000, name = 'turn right')
turn_q_l = task_share.Queue('H', 1000, name = 'turn left')

m_per = 5       # period of running the motor  [ms] - used for closed loop control and cotask
s_per = 15       # period of running the sensor [ms] - used for cotask
l_per= 15 # period of running the limit switches [ms] - used for cotask
class motor:
    """
    @brief      class takes sensor data and controls the motor duty cycle
    """
    
    def __init__(self, motor, encoder, me_set):
        self.mot = motor
        self.enc = encoder
        self.state = 0
        self.set = me_set
        self.v_ref = 15
        self.start = 0
        self.end = 0
        self.count = 0
        self.v_meas = 0
        self.v_set = 0
        self.Kp = 1.6
        self.Ki = 15
        self.mot.set_duty(0)
        self.adjust1 = 10
        self.adjust2 = 40
        self.adjust3 = 85
        self.adjust4 = 25
        self.adjust5 = 30
        self.period = m_per
        self.offset =3
        self.back =0
        self.turn=0
        self.forw =0

    def mot_gen(self):
        """
        @brief      motor generator function for round robin multi-tasking

        @details    obtain instructions from the sensor class and set the motor duty cycle to execute them
        """
        
        while True:
            #self.v_meas = self.get_vel()
            self.enc.update()
            self.end = time.ticks_ms()
            d = self.enc.get_delta()
            t = self.end - self.start
            self.v_meas = (d/t)*(1000/1440)
            self.start = self.end 
            #print(self.set, "     " , self.v_meas)
            
            # STATE 0 - WAITING STATE TO SEE WHAT TO DO 
            if self.state == 0: 
                if self.set == "R":
                    
                    if turn_q_r.any():
                        self.state = turn_q_r.get()
                        
                    else:
                        self.mot.set_duty(self.v_ref)
            
                elif self.set == "L": 
                    
                    if turn_q_l.any():
                        self.state = turn_q_l.get() 
                        
                    else:
                        self.mot.set_duty(self.v_ref + 0) # adding 5 here because the left motor is naturally slower than the right
                
                else: 
                    self.mot.set_duty(self.v_ref)
                
            # STATE 1 - LEFT 1
            elif self.state == 1: 
                #print('STATE 1')
                if self.set == "R":
                    self.mot.set_duty(self.v_ref + self.adjust1)
                    #calc = ClosedLoop(self.period , self.Kp, self.Ki, self.v_ref+self.adjust, self.v_meas)
                    #dc_new = calc.update()
                    #self.mot.set_duty(dc_new)
                   
                elif self.set == "L":
                    self.mot.set_duty(self.v_ref - self.adjust1 + 0)
                    # calc = ClosedLoop(self.period , self.Kp, self.Ki, self.v_ref-self.adjust, self.v_meas)
                    # dc_new=calc.update()
                    # self.mot.set_duty(dc_new)
                   
                self.state = 0
                
            # STATE 2 - RIGHT 1
            elif self.state == 2: 
                #print('STATE 2')
                if self.set == "R":
                    self.mot.set_duty(self.v_ref - self.adjust1)
                    # calc = ClosedLoop(self.period , self.Kp, self.Ki, self.v_ref-self.adjust, self.v_meas)
                    # dc_new=calc.update()
                    # self.mot.set_duty(dc_new)
                    
                elif self.set == "L":
                    self.mot.set_duty(self.v_ref + self.adjust1 + 0)
                    #calc = ClosedLoop(self.period , self.Kp, self.Ki, self.v_ref+self.adjust, self.v_meas)
                    #dc_new=calc.update()
                    #self.mot.set_duty(dc_new)
                    
                self.state = 0
                
            # STATE 3 - FORWARD
            elif self.state == 3:
                #print('STATE 3')
                self.mot.set_duty(self.v_ref)
                # calc = ClosedLoop(self.period, self.Kp, self.Ki, self.v_ref, self.v_meas)
                # dc_new=calc.update()
                # self.mot.set_duty(dc_new)
                self.state = 0 
            
            # STATE 4 - LEFT 2
            elif self.state == 4: 
                #print('STATE 4')
                if self.set == "R":
                   self.mot.set_duty(self.v_ref + self.adjust2)
                   
                elif self.set == "L":
                   self.mot.set_duty(self.v_ref - self.adjust2 + 0)
                  
                self.state = 0
            
            # STATE 5 - LEFT 3
            elif self.state == 5: 
                #print('STATE 5')
                if self.set == "R":
                   self.mot.set_duty(self.v_ref + self.adjust3)
                   
                elif self.set == "L":
                   self.mot.set_duty(self.v_ref - self.adjust3 + 0)
                  
                self.state = 0
            
            # STATE 6 - LEFT 4
            elif self.state == 6: 
                #print('STATE 6')
                if self.set == "R":
                   self.mot.set_duty(self.v_ref + self.adjust4)
                   
                elif self.set == "L":
                   self.mot.set_duty(self.v_ref - self.adjust4 + 0)
                  
                self.state = 0
            
            # STATE 7 - LEFT 5
            elif self.state == 7: 
                #print('STATE 7')
                if self.set == "R":
                   self.mot.set_duty(self.v_ref + self.adjust4)
                   
                elif self.set == "L":
                   self.mot.set_duty(self.v_ref - self.adjust4 + 0)
                  
                self.state = 0
            
            # STATE 8 - RIGHT 2
            elif self.state == 8: 
                #print('STATE 8')
                if self.set == "R":
                    self.mot.set_duty(self.v_ref - self.adjust2 +self.offset)
                    
                elif self.set == "L":
                    self.mot.set_duty(self.v_ref + self.adjust2 + 0)
                
                self.state = 0
            
            # STATE 9 - RIGHT 3
            elif self.state == 9: 
                #print('STATE 9')
                if self.set == "R":
                    self.mot.set_duty(self.v_ref - self.adjust3+self.offset)
                    
                elif self.set == "L":
                    self.mot.set_duty(self.v_ref + self.adjust3 + 0)
                
                self.state = 0
            
            # STATE 10 - RIGHT 4
            elif self.state == 10: 
                #print('STATE 10')
                if self.set == "R":
                    self.mot.set_duty(self.v_ref - self.adjust4+self.offset)
                    
                elif self.set == "L":
                    self.mot.set_duty(self.v_ref + self.adjust4 + 0)
                
                self.state = 0
            
            # STATE 11 - RIGHT 5
            elif self.state == 11: 
                #print('STATE 11')
                if self.set == "R":
                    self.mot.set_duty(self.v_ref - self.adjust5+self.offset)
                    
                elif self.set == "L":
                    self.mot.set_duty(self.v_ref + self.adjust5 + 0)
                
                self.state = 0
            
            # STATE 12 - STOP
            elif self.state == 12:
                #print('STATE 12')
                self.mot.set_duty(0)
                self.state = 0 
            # STATE 13 - TURN SEQUENCE
            elif self.state == 13:
                #print('STATE 12')
               # print("stop",self.set)
                if self.back ==0 and self.turn ==0 and self.forw ==0:
                    print("FIRST")
                    self.enc.zero()
                    self.back =1
                    self.enc.update()
                elif self.back ==1:
                    print("SECOND")
                    self.enc.zero()
                    self.enc.update()
                    if abs(self.enc.get_position()) < 1440: #number of ticks? ######
                        print(abs(self.enc.get_position()))
                        self.mot.set_duty(-3)
                    else:
                        self.turn =1 
                        self.back =0
                        self.mot.set_duty(0)
                        if self.count < 1000:
                            self.count+=1 
                            self.state = 13
                        else:
                            self.state =0
                   # print(self.count)
                
                else: 
                    #self.count =0
                    print("DONE")
                    self.state = 0     
            # #STATE 5 ROTATE IN PLACE CLOCKWISE
            # elif self.state ==5: 
            #     if self.set == "R":
            #         self.mot.set_duty(-1*self.v_ref)
            #     elif self.set == "L":
            #         self.mot.set_duty(self.v_ref+5)
            #     self.state =0  
            else:
                raise ValueError("Invalid State")
                self.state = 0
            
            yield(self.state)

class sensor:
    """
    @brief  class to read sensor data and decide what the motors should do
    """
    
    def __init__(self, S11, S9, S7, S5, S3, S1):
        self.S1  = S1
        self.S3  = S3
        self.S5  = S5
        self.S7  = S7
        self.S9  = S9
        self.S11 = S11
        
        self.state = 0
        self.black = 2000
        self.white = 2000
        
    def sen_gen(self):
        """
        @brief      sensor generator function for round robin multi-tasking
        
        @details    takes in sensor data and tells the motors to do one of the following:
                    go straight
                    turn soft left
                    turn hard left
                    turn soft right
                    turn hard right
        """
        
        while True:
            
            if(self.state == 0):
                
                # GO STRAIGHT
                if self.S9.read()<self.white and self.S7.read()>self.black and self.S5.read()>self.black and self.S3.read()<self.white:
                    turn_q_l.put(3) 
                    turn_q_r.put(3)
                    self.state = 0
                    #print("  3:   ", self.S3.read(),"  5:   ", self.S5.read(),"  7:   ", self.S7.read(),"  9:   ", self.S9.read())
                    #print("  1:   ", self.S1.read(),"  3:   ", self.S3.read(),"  5:   ", self.S5.read(),"  7:   ", self.S7.read(),"  9:   ", self.S9.read(),"  11:   ", self.S11.read())
                   # print("straight")
                    
                # LEFT 1
                elif self.S3.read()>self.black and self.S5.read()>self.black:
                    
                    if turn_q_r.full():
                      print("RIGHT QUEUE FULL")
                      
                    turn_q_l.put(1) 
                    turn_q_r.put(1)
                    self.state = 0
                   # print("left 1")
                   # print("  1:   ", self.S1.read(),"  3:   ", self.S3.read(),"  5:   ", self.S5.read(),"  7:   ", self.S7.read(),"  9:   ", self.S9.read(),"  11:   ", self.S11.read())
                
                # LEFT 2
                elif self.S1.read()>self.black and self.S3.read()>self.black:
                    
                    if turn_q_r.full():
                      print("RIGHT QUEUE FULL")
                      
                    turn_q_l.put(4) 
                    turn_q_r.put(4)
                    self.state = 0
                    #print("left 2")
                    
                # LEFT 3
                elif self.S1.read()>self.black and self.S3.read()<self.white and self.S5.read()<self.white and self.S7.read()<self.white and self.S9.read()<self.white and self.S11.read()<self.white:
                    
                    if turn_q_r.full():
                      print("RIGHT QUEUE FULL")
                      
                    turn_q_l.put(5) 
                    turn_q_r.put(5)
                    self.state = 0
                   # print("left3")
                    
                # LEFT 4
                elif self.S5.read()>self.black and self.S7.read()>self.black and self.S9.read()>self.black:
                    
                    if turn_q_r.full():
                      print("RIGHT QUEUE FULL")
                      
                    turn_q_l.put(6) 
                    turn_q_r.put(6)
                    self.state = 0
                   # print("left 4")
                
                # LEFT 5
                elif self.S5.read()>self.black and self.S7.read()>self.black and self.S9.read()>self.black and self.S11.read()>self.black:
                    
                    if turn_q_r.full():
                      print("RIGHT QUEUE FULL")
                      
                    turn_q_l.put(7) 
                    turn_q_r.put(7)
                    self.state = 0
                  #  print("left 5")
                
                # RIGHT 1 
                elif self.S7.read()>self.black and self.S9.read()>self.black:
                    
                    if turn_q_l.full():
                      print("LEFT QUEUE FULL")
                      
                    turn_q_l.put(2) 
                    turn_q_r.put(2)
                    self.state = 0
                  #  print("right 1")
                   # print("  1:   ", self.S1.read(),"  3:   ", self.S3.read(),"  5:   ", self.S5.read(),"  7:   ", self.S7.read(),"  9:   ", self.S9.read(),"  11:   ", self.S11.read())
                   # print("  3:   ", self.S3.read(),"  5:   ", self.S5.read(),"  7:   ", self.S7.read(),"  9:   ", self.S9.read())
                 
                # RIGHT 2 
                elif self.S9.read()>self.black and self.S11.read()>self.black:
                    
                    if turn_q_l.full():
                      print("LEFT QUEUE FULL")
                      
                    turn_q_l.put(8) 
                    turn_q_r.put(8)
                    self.state = 0
                   # print("right 2")
                
                # RIGHT 3 
                elif self.S11.read()>self.black and self.S3.read()<self.white and self.S5.read()<self.white and self.S7.read()<self.white and self.S9.read()<self.white and self.S1.read()<self.white:
                    
                    if turn_q_l.full():
                      print("LEFT QUEUE FULL")
                      
                    turn_q_l.put(9) 
                    turn_q_r.put(9)
                    self.state = 0
                    # print("right 3")
                    
                # RIGHT 4 
                elif self.S3.read()>self.black and self.S5.read()>self.black and self.S7.read()>self.black:
                    
                    if turn_q_l.full():
                      print("LEFT QUEUE FULL")
                      
                    turn_q_l.put(10) 
                    turn_q_r.put(10)
                    self.state = 0
                    # print("right 4")
                    
                # RIGHT 5 
                elif self.S1.read()>self.black and self.S3.read()>self.black and self.S5.read()>self.black and self.S7.read()>self.black:
                    
                    if turn_q_l.full():
                      print("LEFT QUEUE FULL")
                      
                    turn_q_l.put(11) 
                    turn_q_r.put(11)
                    self.state = 0
                   # print("right 5")
                
                # #DASHED LINES - if all white drive forward
                # elif  self.S9.read()<self.white and self.S7.read()<self.white and self.S5.read()<self.white and self.S3.read()<self.white:
                #     turn_q_l.put(3) 
                #     turn_q_r.put(3)
                #     self.state =0
                #     print("  3:   ", self.S3.read(),"  5:   ", self.S5.read(),"  7:   ", self.S7.read(),"  9:   ", self.S9.read())
                # #BLACK CROSSED LINES - if all black drive forward
                # elif  self.S9.read()>self.black and self.S7.read()>self.black and self.S5.read()>self.black and self.S3.read()>self.black:
                #     turn_q_l.put(3) 
                #     turn_q_r.put(3)
                #     self.state =0
                #     print("  3:   ", self.S3.read(),"  5:   ", self.S5.read(),"  7:   ", self.S7.read(),"  9:   ", self.S9.read())
                    
                else: 
                    #default to go straight
                  #  print("straight")
                    turn_q_l.put(3) 
                    turn_q_r.put(3)
                    self.state = 0
                    
            else:
                raise ValueError("Invalid State")
                self.state = 0
                
            yield(self.state)
            
class switch:
    """
    @brief  class to read sensor data and decide what the motors should do
    """
    
    def __init__(self, SWL, SWR):
        self.left = SWL
        self.right= SWR
        self.state =0
    def sw_gen(self):
        """
        @brief      
        
        @details    
        """
        while True: 
          if(self.state == 0): 
            
            if self.left.value() == 1 or self.right.value() == 1:
                self.state =1
            else: self.state = 0
          elif(self.state ==1):
              turn_q_l.put(13)
              print("triggered")
              turn_q_r.put(13)
              self.state = 2
          elif(self.state ==2):
              self.state =2
          else:
              raise ValueError("Invalid State")
              self.state = 0
              
          yield(self.state)
            
            
                    
            
            
#b8 is right switch
#b9 is left switch
SWR = Pin(Pin.cpu.B6, mode= Pin.IN)
SWL = Pin(Pin.cpu.B10, mode = Pin.IN)

# ENCODERS
AR = 62354
PS = 0
tim_A = Timer(1, period=AR, prescaler=PS)
ENC_A = Encoder(tim_A, Pin.cpu.A8, Pin.cpu.A9, AR)
tim_B = Timer(3, period=AR, prescaler=PS)
ENC_B = Encoder(tim_B, Pin.cpu.B4, Pin.cpu.B5, AR)



# MOTOR R (RIGHT)
tim_8 = Timer(8, freq=20000)
mot_R = L6206(tim_8, Pin.cpu.C8, Pin.cpu.C9, Pin.cpu.C7)
mot_R.enable()
mot_R.set_duty(0)



# MOTOR L (LEFT)
tim_4 = Timer(4, freq=20000)
mot_L = L6206(tim_4, Pin.cpu.B8, Pin.cpu.B9, Pin.cpu.B7)
mot_L.enable()
mot_L.set_duty(0)
global stat
stat= 0
def motor_toggle(mot_R, mot_L, status):
    #print(stat)
    if status == 1:
        mot_R.disable()
        mot_L.disable()
        status = 0
    else:
        mot_R.enable()
        mot_L.enable()
        status = 1
    stat = status
# class button: 
#    # def __init__(self, pin, r_mot, l_mot, )
#     def motor_toggle(mot_R, mot_L, status):
#         #print(stat)
#         if status == 1:
#             mot_R.disable()
#             mot_L.disable()
#             status = 0
#         else:
#             mot_R.enable()
#             mot_L.enable()
#             status = 1
#         stat = status
        


#SENSORS 
#DEFINED FROM THE FRONT OF THE ROBOT - Left to Right 11,9,7,5,3,1
#Pins used A6 A7 A1 A0 C2 C3       
S1  = Pin(Pin.cpu.C2, mode = Pin.IN)
S3  = Pin(Pin.cpu.A6, mode = Pin.IN)
S5  = Pin(Pin.cpu.A7, mode = Pin.IN)
S7  = Pin(Pin.cpu.A1, mode = Pin.IN)
S9  = Pin(Pin.cpu.A0, mode = Pin.IN)
S11 = Pin(Pin.cpu.C3, mode = Pin.IN)

S1_adc  = pyb.ADC(S1)
S3_adc  = pyb.ADC(S3)
S5_adc  = pyb.ADC(S5)
S7_adc  = pyb.ADC(S7)
S9_adc  = pyb.ADC(S9)
S11_adc = pyb.ADC(S11)



#BLUE BUTTON
#PA5 = Pin(Pin.cpu.A5, mode=Pin.OUT_PP)
button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING,Pin.PULL_NONE, lambda p: motor_toggle(mot_R,mot_L,(0 if stat==1 else 1)))


#COTASK NONSENSE
motor_R = motor(mot_R, ENC_A, "R")
motor_L = motor(mot_L, ENC_B, "L")    
Switches = switch(SWL, SWR) 
Sensor = sensor(S11_adc, S9_adc, S7_adc, S5_adc, S3_adc, S1_adc)
     
# tasks setup     
motorR = cotask.Task(motor_R.mot_gen, name = "Motor R Run Task", priority = 2, period = m_per, profile = True, trace = True)
motorL = cotask.Task(motor_L.mot_gen, name = "Motor L Run Task", priority = 2, period = m_per, profile = True, trace = True)
Sen    = cotask.Task(Sensor.sen_gen , name = "Sensor Run Task" , priority = 1, period = s_per, profile = True, trace = True)
swi = cotask.Task(Switches.sw_gen , name = "Limit Switches Run Task" , priority = 3, period = l_per, profile = True, trace = True)
# create the scheduler
cotask.task_list.append(Sen)
cotask.task_list.append(motorR)
cotask.task_list.append(motorL)
cotask.task_list.append(swi)



while True:
        cotask.task_list.pri_sched()