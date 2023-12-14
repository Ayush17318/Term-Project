"""
@file               ClosedLoop.py

@brief              A closed loop feedback class implementing P, PI, or PID control

@author             Ayush Kakkanat & Akanksha Maddi

@date               17 October 2023

@latest update      11 December 2023
"""

class ClosedLoop:
    """
    @brief      A closed loop feedback class implementing P, PI, or PID control
    """
    
    def __init__(self, period, Kp, Ki, ref, meas):
        """
        @brief          initialize the parameters
        
        @param Kp       proportional control gain
        
        @param Ki       integral control gain
        
        @param ref      desired output
        
        @param meas     measured output
        """
        
        # input rps velocities
        
        
        self.Kp = Kp
        self.Ki = Ki
        self.meas = meas
        self.ref = ref
        self.error = 0
        self.integral = 0
        self.period = period/1000
        
    def set_Kp(self, Kp):
        """
        @brief          set the Kp
        
        @param Kp       proportional control gain
        """
    
        self.Kp = Kp
        
    def set_Ki(self, Ki):
        """
        @brief          set the Ki
        
        @param Ki       integral control gain
        """
        
        self.Ki = Ki
        
    def setpoint(self, ref):
        """
        @brief          set the desired value
        
        @param ref      desired output
        """
        
        self.ref = ref
        
    def update(self):
        """
        @brief      math for obtaining the output after the controller
        """
        
        self.error =  self.ref - self.meas
        self.proportional = self.Kp*self.error
        self.integral += self.Ki*self.error*self.period
        self.L = self.proportional + self.integral
        
        # returning duty cycle
        return self.L


        
        
        
        
        
        
        
        