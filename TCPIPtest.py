import time
from datetime import datetime
import socket
from PyQt5.QtWidgets import QMessageBox
import sys
from ui_ROBOT import *
import struct

Mat = [[ 6.30534639e-02,-1.39677331e-04,2.17401582e+02],
 [4.36198708e-04,-6.38837907e-02,7.53214724e+02]]
A = Mat[0][0]
B = Mat[0][1]
C = Mat[0][2]
D = Mat[1][0]
E = Mat[1][1]
F = Mat[1][2]

class tcpip(Ui_Dialog):
    def __init__(self):
        super(tcpip, self).__init__()
        self.s = None
        self.client = None
        self.accept = None
        self.working = None
        self.addr = None

    def socket_connect(self,flag):
        self.working = True

        address = ('192.168.0.20', 65056)
        max_size = 1000
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        '''
        socket.AF_INET表示创建一个IP套接字；socket.SOCK_STREAM 表示流式socket , for TCP
        sock_DGRAM表示数据报式socket , for UDP
        '''
        self.s.bind(address)
        self.s.listen(5)
        print('server listening...')
        self.client, self.addr = self.s.accept()
        # print('客户端地址：',addr)
        self.client.sendall(b'Connected\r\n')
        data = self.client.recv(max_size)
        if len(data) != 0:
            print('connected success!')
        else:
            print('no data received!')
        # print('接收到的数据类型：',type(data))
            # elif flag == 1:
            #     client.sendall(b'200,600,460,45,0.7,178\r\n')   #b'381.875,638.197,514.418,86.835,48.696,-93.537\r\n'
            # elif flag == 2:
            #     client.sendall(b'')
            # elif flag == 3:
            #     client.sendall(b'')
        # client.sendall(b'200,600,460,45,0.7,178\r\n')
        # time.sleep(10)
            # client.close()
            # self.s.close()

    def move_Place1(self):
        self.client.sendall(b'348.061,637.934,479.113,86.835,48.696,-93.537\r\n')
        time.sleep(10)

    def move_Place2(self):
        self.client.sendall(b'231.235,638.196,531.674,86.835,48.696,-93.537\r\n')

    def move_Place3(self):
        self.client.sendall(b'76.956,638.197,531.674,86.835,48.696,-93.537\r\n')

    def catch_parts(self,Xor,Yor,angle):
        X_Bot = int(A * Xor + B * Yor + C + 12)
        Y_Bot = int(D * Xor + E * Yor + F - 2)
        angle = int(abs(46.712-(90-angle[0])))
        # print(Y_Bot)
        X_Bot = struct.pack('i',X_Bot)
        Y_Bot = struct.pack('i',Y_Bot)
        angle = struct.pack('i',angle)
        # print(X_Bot)
        angle = str((-1)*angle[0])
        angle =angle.encode('utf-8')

        xbot = str(hex(X_Bot[1])+hex(X_Bot[0]))
        # print(xbot)
        xbot = xbot[3] + xbot[2] + xbot[5] + xbot[6]
        xbot = str(int(xbot,16))
        # xbot = int(xbot,16).to_bytes(length=8,byteorder='little',signed=False)
        xbot = xbot.encode('utf-8')
        ybot = str(hex(Y_Bot[1])+hex(Y_Bot[0]))
        # print(ybot)
        ybot = ybot[3] + ybot[2] + ybot[5] + ybot[6]

        ybot = str(int(ybot,16))
        ybot = ybot.encode('utf-8')

        # X_Bot = X_Bot.to_bytes(length=4,byteorder='little',signed=False)
        self.client.sendall(xbot+b','+ybot+b',223,46.712,1.640,-177.818\r\n')


