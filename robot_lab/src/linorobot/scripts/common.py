
frame_start  = b'\x7A'
receive_add  = b'\x00'
group_id = b'\x00'
sender_add = b'\x02'
frame_end = b'\x7F'
msg_count  = b'\x00\x00'

PC_to_Robot_msg_type = {'RUN':b'\x81','STOP':b'\x82','SENSOR':b'\x83',
                        'ENCODER':b'\x84','RESEND':b'\x8F'}
Robot_to_PC_msg_type = {'ENCODER':b'\x61','DRIVER_ERR':b'\x63','SENSOR':b'\x62',
                        'OTHER_ERR':b'\x64','RESEND':b'\x6F'}

def create_msg(frame_length,data , data_length,msg_type):
    check_sum = frame_length
    if data == None:
        return None
    msg = frame_start + receive_add + frame_length + group_id + sender_add + msg_type + data_length + data + check_sum + frame_end 

    return msg


