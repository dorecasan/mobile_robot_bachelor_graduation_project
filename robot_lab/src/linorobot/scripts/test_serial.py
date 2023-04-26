import serial 
import numpy as np
from common import *

class PC_ROBOT_Communication:

    def __init__(self,b_rate = 9600):
        
        self.ser = serial.Serial(port='/dev/ttyS0',timeout=1,baudrate= b_rate)
        self.velocity_data  = []
        

    def PC_to_ROBOT(self, msg_type, data = None):
        msg = PC_to_Robot_msg_type[msg_type]
        data_length = None
        frame_length = None
        self.data_bytes_send = None

        if len(data) == 3 and msg_type == 'RUN':     
            self.velocity_data = data  
            data_length = b'0x07'
            frame_length = b'0x11'
            vx = (data[0]).to_bytes(2,'big',signed = True)
            vy = (data[1]).to_bytes(2,'big',signed = True)
            th = (data[2]).to_bytes(2,'big',signed = True)
            self.data_bytes_send = vx+vy+th
            
        
        elif msg_type in  ['STOP', 'ENCODER', 'SENSOR'] :
            data_length = b'0x02'
            frame_length = b'0x0C'
            self.data_bytes_send = (68).to_bytes(1,'big')
           

        elif msg_type == 'RESEND':
            data_length = b'0x03'
            frame_length = b'0x0C'
            self.data_bytes_send = msg + msg_count


        msg_to_send = create_msg(frame_length,self.data_bytes_send , data_length,msg)

        if msg_to_send != None:
            print(msg_to_send)

    def ROBOT_to_PC(self):
        data_bytes = self.ser.read(15)
        data_length = int.from_bytes(data_bytes[8],'big')
        frame_length = int.from_bytes(data_bytes[2],'big')
        msg_type = data_bytes[7]
        self.data_receive = None

        if msg_type == Robot_to_PC_msg_type['ENCODER']:
            vx = int.from_bytes(data_bytes[9:11],'big',signed= True)
            vy = int.from_bytes(data_bytes[11:13],'big',signed= True)
            th = int.from_bytes(data_bytes[13:15],'big',signed= True)
            self.data_receive = [vx,vy,th]

        elif msg_type == Robot_to_PC_msg_type['SENSOR']:
            con_quay_hc = int.from_bytes(data_bytes[9],'big',signed= True)
            ax = int.from_bytes(data_bytes[10],'big',signed= True)
            ay = int.from_bytes(data_bytes[11],'big',signed= True)
            az = int.from_bytes(data_bytes[12],'big',signed= True)
            self.data_receive  = [con_quay_hc,ax,ay,az]
        
        elif msg_type == Robot_to_PC_msg_type['DRIVER_ERR']:
            b = int.from_bytes(data_bytes[9],'big')
            c = b[0] & b'\xaa'[0] 
            e = b[0] & b'\x55'[0] 
            d = str(bin(c))[:2].zfill(8)
            f = str(bin(e))[:2].zfill(8)
            res1 = [i+1 for i in range(len(d)) if d.startswith('1', i)]
            res2= [i+1 for i in range(len(f)) if f.startswith('1', i)]

            if res1 != []:
                raise Exception('Over current at drivers {}'.format(res1))
            if res2 != []: 
                raise Exception('Errors at drivers {}'.format(res2))
        
        elif msg_type == Robot_to_PC_msg_type['OTHER_ERR']:
            raise Exception('Some errors occur and I do not know!!!!!!!!!!!')

        elif msg_type == Robot_to_PC_msg_type['RESEND']:
            msg_need_to_resend = data_bytes[9]

            if msg_need_to_resend[0] == b'\x81'[0]:
                msg_type_resend = 'RUN'
                data_resend = self.velocity_data

            elif msg_need_to_resend[0] == b'\x82'[0]:
                msg_type_resend = 'STOP'
            elif msg_need_to_resend[0] == b'\x83'[0]:
                msg_type_resend = 'SENSOR'
            elif msg_need_to_resend[0] == b'\x84'[0]:
                msg_type_resend = 'ENCODER'  
 
        if self.data_receive != None:
            print('OKKKKKKK!!!')
            
toan = PC_ROBOT_Communication()
toan.PC_to_ROBOT('RUN',[12,13,14])




