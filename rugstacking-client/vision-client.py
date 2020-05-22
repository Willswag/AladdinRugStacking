#Spencer Waguespack
#mohawk rugg stacking
#vision client
#this program facilitates comunication between robot and vision server
#
import zmq
import re
import numpy as np


ku =3#focal length along x direction
gam =4#skew factor
uo =10#principal point in x direction
kv =20#focal length along y
vo = 30#pp in y direction

A = np.matrix([[ku,gam,uo,0],[0,kv,vo,0],[0, 0, 1,0],[0,0,0,1]])#internal camera paramaters
B = np.matrix([[1,0,0,0],[0,1,0,0],[0,1,0,0],[0,0,0,1]])#external cooridnates camera to object 

def cam_to_bot(pos):
    new_pos = B*A @ pos.T
    return new_pos


def bytes_to_list(msg):
    msg_str = msg.decode("utf-8")
    print("Message received:\n")# + msg_str)
    msg_str_values = re.findall(r"[-+]?\d*\.\d+|\d+", msg_str)
    msg_values = [float(x) for x in msg_str_values]

    return msg_values

def get_rug_poses():
    print("Sending request for poses")
    vision_socket.send(b"Request peg poses")
    msg = vision_socket.recv()
    ret = bytes_to_list(msg)
    
    return ret



if __name__ == "__main__":
    # Connect to server providing machine vision for peg poses and spool circles
    print('Connecting to machine-vision server')
    vision_context = zmq.Context()
    vision_socket = vision_context.socket(zmq.REQ)
    vision_socket.connect("tcp://127.0.0.1:43001")
while True:
    rug = get_rug_poses()
    #print(rug[2])
    pos = np.array([rug[0],rug[1],0,rug[2]])
    #print(pos)
    rob = cam_to_bot(pos)
    print(rob.T)




