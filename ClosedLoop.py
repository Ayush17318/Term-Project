"""!
@file ClosedLoop.py
A closed loop feedback class implementing P or PI control.

@author Ayush Kakkanat & Akanksha Maddi
@date   17 October 2023 Date of file creation
@date   19 December 2023 Updated doxygen documentation
"""

class ClosedLoop:
    """!
    A closed loop feedback class implementing P or PI control.
    """
    
    def __init__(self, period, Kp, Ki, ref, meas):
        """
        @brief          initialize the parameters
        
        @param Kp       proportional control gain
        
        @param Ki       integral control gain
        
        @param ref      desired output
        
        @param meas     measured output
        """
        
        """!
        Initialize the parameters for the closed loop control.
        
        @param period The differential time used for integral or derivative
               control in seconds
        @param Kp Proportional control gain
        @param Ki Integral control gain
        @param ref Desired output
        @param meas Measured output
        """
        
        self.Kp = Kp
        self.Ki = Ki
        self.meas = meas
        self.ref = ref
        self.error = 0
        self.integral = 0
        self.period = period/1000
        
    def set_Kp(self, Kp):
        """!
        Set the Kp.
        
        @param Kp Proportional control gain
        """
    
        self.Kp = Kp
        
    def set_Ki(self, Ki):
        """
        Set the Ki.
        
        @param Ki Integral control gain
        """
        
        self.Ki = Ki
        
    def setpoint(self, ref):
        """!
        Set the desired value.
        
        @param ref Desired output
        """
        
        self.ref = ref
        
    def update(self):
        """!
        Math for obtaining the output after the controller.
        """
        
        self.error =  self.ref - self.meas
        self.proportional = self.Kp*self.error
        self.integral += self.Ki*self.error*self.period
        self.L = self.proportional + self.integral
        
        # returning output
        return self.L


        
        
        
        
        
        
        
        