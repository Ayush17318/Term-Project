"""
@file               BNO055.py

@brief              class to interface BNO055 IMU sensor
            
@details            the class has methods that can obtain IMU euler angles and
                    angular velocity data along with methods to facilitate
                    calibration
                       
@author             Ayush Kakkanat

@date created       12 November 2023            

@latest update      15 December 2023
"""

import time
import struct  # structs unpack binary data into integers
from machine import Pin, I2C

class BNO055:
    """
    @brief      class to interface BNO055 IMU sensor
    """
    
    def __init__(self, i2c):
        """
        @brief      intialize the i2c, all the addresses, and set the 
                    operation mode
        
        #i2c        set the i2c for this class
        """
        
        self.i2c = i2c
        self.i2c_addr = 0x28    # Default I2C address for BNO055
        
        self.mag_radius_msb = 0x6A  # I2C address for magnetometer radius msb
        self.mag_radius_lsb = 0x69  # I2C address for magnetometer radius lsb
        self.acc_radius_msb = 0x68  # I2C address for accelerometer radius msb
        self.mag_radius_lsb = 0x67  # I2C address for accelerometer radius msb
        
        self.gyr_off_z_msb = 0x66   # I2C address for gyroscope offset z axis msb
        self.gyr_off_z_lsb = 0x65   # I2C address for gyroscope offset z axis lsb
        self.gyr_off_y_msb = 0x64   # I2C address for gyroscope offset y axis msb
        self.gyr_off_y_lsb = 0x63   # I2C address for gyroscope offset y axis lsb
        self.gyr_off_x_msb = 0x62   # I2C address for gyroscope offset x axis msb
        self.gyr_off_x_lsb = 0x61   # I2C address for gyroscope offset x axis lsb
        
        self.mag_off_z_msb = 0x60   # I2C address for magnetometer offset z axis msb
        self.mag_off_z_lsb = 0x5F   # I2C address for magnetometer offset z axis lsb
        self.mag_off_y_msb = 0x5E   # I2C address for magnetometer offset y axis msb
        self.mag_off_y_lsb = 0x5D   # I2C address for magnetometer offset y axis lsb
        self.mag_off_x_msb = 0x5C   # I2C address for magnetometer offset x axis msb
        self.mag_off_x_lsb = 0x5B   # I2C address for magnetometer offset x axis lsb
        
        self.acc_off_z_msb = 0x5A   # I2C address for accelerometer offset z axis msb
        self.acc_off_z_lsb = 0x59   # I2C address for accelerometer offset z axis lsb
        self.acc_off_y_msb = 0x58   # I2C address for accelerometer offset y axis msb
        self.acc_off_y_lsb = 0x57   # I2C address for accelerometer offset y axis lsb
        self.acc_off_x_msb = 0x56   # I2C address for accelerometer offset x axis msb
        self.acc_off_x_lsb = 0x55   # I2C address for accelerometer offset x axis lsb
        
        # axis remap constant for various BNO055 orientation configs
        self.AXIS_REMAP_P0 = 0x21   
        self.AXIS_REMAP_P1 = 0x24
        self.AXIS_REMAP_P2 = 0x24
        self.AXIS_REMAP_P3 = 0x21
        self.AXIS_REMAP_P4 = 0x24
        self.AXIS_REMAP_P5 = 0x21
        self.AXIS_REMAP_P6 = 0x21
        self.AXIS_REMAP_P7 = 0x24

        # axis sign constant for various BNO055 orientation configs
        self.AXIS_REMAP_SIGN_P0 = 0x04
        self.AXIS_REMAP_SIGN_P1 = 0x00
        self.AXIS_REMAP_SIGN_P2 = 0x06
        self.AXIS_REMAP_SIGN_P3 = 0x02
        self.AXIS_REMAP_SIGN_P4 = 0x03
        self.AXIS_REMAP_SIGN_P5 = 0x01
        self.AXIS_REMAP_SIGN_P6 = 0x07
        self.AXIS_REMAP_SIGN_P7 = 0x05
        
        self.set_operation_mode(0x00)  # Set to CONFIG mode

    def set_operation_mode(self, mode):
        """
        @brief      set the operation mode by writing to the IMU
        
        @mode       hex value indicating the operation mode
        """
        
        self.i2c.writeto_mem(self.i2c_addr, 0x3D, bytearray([mode]))      

    def reset(self):
        self.write_byte(0x3D, 0x20)  # Reset BNO055
        time.sleep_ms(650)

    def get_calibration_status(self):
        """
        @brief      obtain the calibration status for each of the sensor and
                    overall system
                    0 is not calibrated
                    3 is fully calibrated
        """
        
        calibration_byte = self.i2c.readfrom_mem(self.i2c_addr, 0x35, 1)
        sys, gyro, accel, mag = [(calibration_byte[0] >> i) & 0x03 for i in range(6, -1, -2)]
        return {"system": sys, "gyroscope": gyro, "accelerometer": accel, "magnetometer": mag}

    def read_calibration_coefficients(self):
        """
        @brief      get the 22 calibration coefficients
                    each coeff is 2 bytes across 2 registers by MSB and LSB
                    order is (2 bytes each) accel x
                                            accel y
                                            accel z
                                            mag x
                                            mag y
                                            mag z
                                            gyr x
                                            gyr y
                                            gyr z
                                            accel radius
                                            mag radius
                    struct contains 11 signed integers         
        """
        
        coefficients = self.i2c.readfrom_mem(self.i2c_addr, 0x55, 22)    # 22 bytes of data (6 from each sensor and 4 for radius)
        return struct.unpack("<22B", coefficients)     # values should be stored as: acc x, acc y, acc z, mag(x,y,z), gyr(x,y,z), acc radius, mag radius
    
    def write_calibration_coefficients(self, coefficients):
        """
        @brief      write the 22 calibration coefficients to the IMU
                    each coeff is 2 bytes across 2 registers by MSB and LSB
                    order is (2 bytes each) accel x
                                            accel y
                                            accel z
                                            mag x
                                            mag y
                                            mag z
                                            gyr x
                                            gyr y
                                            gyr z
                                            accel radius
                                            mag radius
                    struct contains 11 signed integers         
        """
        
        packed_data = struct.pack("<22B", *coefficients)
        self.i2c.writeto_mem(self.i2c_addr, 0x55, packed_data)

    def read_euler_angles(self): 
        """
        @brief      get the 6 euler angles
                    each angle is 2 bytes across 2 registers by MSB and LSB
                    order is (2 bytes each) roll, pitch, yaw        
                    struct contains 6 signed integers         
        """
        
        euler_data = self.i2c.readfrom_mem(self.i2c_addr, 0x1A, 6)
        yaw, roll, pitch = struct.unpack("<3h", euler_data)
        return {"roll": roll / 16.0, "pitch": pitch / 16.0, "yaw": yaw / 16.0}     # 1 deg = 16 LSB, default units are in deg

    def read_angular_velocity(self):
        """
        @brief      get the 6 ang velocities (rate of change in euler angles)
                    each ang vel is 2 bytes across 2 registers by MSB and LSB
                    order is (2 bytes each) rate of roll, pitch, yaw        
                    struct contains 6 signed integers         
        """
        
        angular_data = self.i2c.readfrom_mem(self.i2c_addr, 0x14, 6)
        roll_rate, pitch_rate, yaw_rate = struct.unpack("<3h", angular_data)
        return {"roll_rate": roll_rate / 16.0, "pitch_rate": pitch_rate / 16.0, "yaw_rate": yaw_rate / 16.0}     # 1 deg/sec = 16 LSB, default units are deg/sec

    def load_coefficients(self, filename = "IMU_cal_coeffs.txt"):
        """
        @brief      load calibration coefficients from a file and set them in the IMU

        @filename   name of the file to load calibration coefficients from
        """
        
        try:
            # with open(filename, "r") as file:
            #     coeff_data = file.readline().strip().split(",")
            #     coeff_data = [int(x, 16) for x in coeff_data]    # convert hexadecimal strings to integers
            #     self.write_calibration_coefficients(coeff_data)
            # print("Calibration data loaded from file.")
            
            with open(filename, "r") as file:
                coeff_data = file.readline().strip().split(",")
                print("Calibration Coefficients:", coeff_data)
                coeff_data_int = [int(x, 0) for x in coeff_data]
                self.write_calibration_coefficients(coeff_data_int)
            #print("Calibration data loaded from file.")
        except OSError:
            print("No calibration file found.")

    def save_coefficients(self, filename = "IMU_cal_coeffs.txt"):
        """
        @brief      save calibration coefficients to a file

        @filename   name of the file to save calibration coefficients to
        """
        
        coeff_data = self.read_calibration_coefficients()
        #coeff_data = [int(x, 0) if not x.startswith('0x-') else -int(x[3:], 0) for x in raw_data]
        coeff_data_str = ",".join(hex(x) for x in coeff_data)
        #coeff_data_str = ",".join(f"{x:08X}" if x >= 0 else f"{(1 << 32) + x:08X}" for x in coeff_data)
        #coeff_data_str = ",".join(f"{x if x >= 0 else (1 << 32) + x:08X}" for x in coeff_data)     # f"0x{x:02X}" is a format specification to write hex values as a string
        with open(filename, "w") as file:
            file.write(coeff_data_str)
        print("Calibration data saved to file.")
        
    def remap_axes(self, placement):
        """
        @brief          remap the axes of the BNO055
        
        @placement      orientation of the imu axes wrt to desired orientation
        """
        
        # Set the axis configuration and sign registers
        if placement == 0:
            remap = 0x21
            sign = 0x04
        elif placement == 1:
            remap = 0x24
            sign = 0x00
        elif placement == 2:
            remap = 0x24
            sign = 0x06
        elif placement == 3:
            remap = 0x21
            sign = 0x02
        elif placement == 4:
            remap = 0x24
            sign = 0x03
        elif placement == 5:
            remap = 0x21
            sign = 0x01
        elif placement == 6:
            remap = 0x21
            sign = 0x07
        elif placement == 7:
            remap = 0x24
            sign = 0x05
        else:
            raise ValueError("Invalid placement value")

        # Set the axis config and sign registers
        self.i2c.writeto_mem(self.i2c_addr, 0x41, bytes([remap]))
        self.i2c.writeto_mem(self.i2c_addr, 0x42, bytes([sign]))