"""
@file               Queues.py
    
@brief              Sets up the queues to be used in the main file and task
                    class files

@author             Ayush Kakkanat

@date created       5 November 2023

@latest update      5 November 2023
"""

import task_share

user_entry_A_q = task_share.Queue('H', 100, name = 'user entry')
duty_cycle_A_q = task_share.Queue('f', 100, name = 'OL duty cycle entry')
setpoint_velocity_A_q = task_share.Queue('H', 100, name = 'CL setpoint velocity')
gain_A_q = task_share.Queue('f', 100, name = 'CL setpoint velocity')
record_A_q = task_share.Queue('d', 100, name = 'record data')    # 1 is ol 2 is CL

new_duty_cycle = task_share.Queue('d', 100, name = 'collect data') 

user_entry_B_q = task_share.Queue('H', 100, name = 'user entry')
duty_cycle_B_q = task_share.Queue('f', 100, name = 'OL duty cycle entry')
setpoint_velocity_B_q = task_share.Queue('H', 100, name = 'CL setpoint velocity')
gain_B_q = task_share.Queue('f', 100, name = 'CL setpoint velocity')
record_B_q = task_share.Queue('d', 100, name = 'record data')