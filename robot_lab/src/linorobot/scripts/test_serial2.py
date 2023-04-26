#!/usr/bin/env python
import serial 
import time

frame_start  = b'\x51'
frame_end = b'\xFF'


def create_msg(data, sign):
    if data == None:
        return None
    msg = frame_start + sign + data + frame_end 

    return msg

class PC_ROBOT_Communication:

    def __init__(self,b_rate = 9600, port='/dev/ttyUSB0'):
        
        self.ser = serial.Serial(port = port,timeout=1,baudrate= b_rate)
        self.velocity_data  = []
        

    def PC_to_ROBOT(self, data = None):
        self.data_bytes_send = None
        s_vx = 4 if data[0] <0 else 0 
        s_vy = 2 if data[1] < 0 else 0
        s_th = 1 if data[2] <0 else 0
        sign_bytes = (s_vx | s_vy | s_th).to_bytes(1,'big',signed=False)
     

        if len(data) == 3:     
            self.velocity_data = data  
            vx = (abs(data[0])).to_bytes(2,'big',signed = False)
            vy = (abs(data[1])).to_bytes(2,'big',signed = False)
            th = (abs(data[2])).to_bytes(2,'big',signed = False)
            self.data_bytes_send = vx+vy+th


        msg_to_send = create_msg(self.data_bytes_send, sign_bytes)

        if msg_to_send != None:
            self.ser.write(msg_to_send)

    def ROBOT_to_PC(self):

        data_bytes = self.ser.read(9)  
      
        self.data_receive = None 
        if data_bytes != b'':
            sign_bytes = data_bytes[1]
            sx = -1 if (sign_bytes & 4) ==4 else 1
            sy = -1 if (sign_bytes & 2) ==2 else 1 
            s_th = -1 if (sign_bytes & 1) ==1 else 1 

        
            vx = int.from_bytes(data_bytes[2:4],'big',signed= False) * sx
            vy = int.from_bytes(data_bytes[4:6],'big',signed= False) * sy
            th = int.from_bytes(data_bytes[6:8],'big',signed= False) * s_th
            self.data_receive = [vx,vy,th]

        return self.data_receive





