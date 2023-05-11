# import threading
# import struct
# import cv2
# import numpy as np
import sys, os
sys.path.append('D:/pythonProject/DownloadCode/Python_HIKVISION_Samples/MvImport')
# from MvCameraControl_class import *
from CamOperation_class import *
# from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread,pyqtSignal
# from PyQt5.QtGui import QImage,QPixmap
import ui_manipulator as u1
import ui_ROBOT as u2
# from ui_ROBOT import *
# from ui_manipulator import *
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog,QLabel
import openCam
from os import getcwd
# import time
from TCPIPtest import *

def is_chinese(uchar):
    """判断一个unicode是否是汉字"""

    if uchar >= u'/u4e00' and uchar <= u'/u9fa5':

        return True

    else:

        return False

class mysencondWin(QMainWindow,Ui_Dialog):
    def __init__(self):
        super(mysencondWin, self).__init__()
        self.u2 = u2.Ui_Dialog()
        self.u2.setupUi(self)
        self.u2.pushButton_4.clicked.connect(self.tcpipconnect)
        self.u2.pushButton_3.clicked.connect(self.get_coordinate_Place1)
        self.u2.pushButton_5.clicked.connect(self.get_coordinate_Place2)
        self.u2.pushButton_7.clicked.connect(self.get_coordinate_Place3)


    def tcpipconnect(self):
        self.u2.pushButton_4.setEnabled(False)
        self.u2.pushButton_4.setText('连接中...')
        self.tcpip = threading.Thread(target=tcpip.socket_connect,args=(self,0))
        self.tcpip.start()
        self.updata_btn_2()

    def updata_btn_2(self):
        self.u2.pushButton_4.setText('已连接')

    def get_coordinate_Place1(self):
        self.u2.lineEdit.setText('381.875')
        self.u2.lineEdit_2.setText('86.835')
        self.u2.lineEdit_3.setText('638.197')
        self.u2.lineEdit_4.setText('48.696')
        self.u2.lineEdit_5.setText('514.418')
        self.u2.lineEdit_6.setText('-93.537')
        self.tcpip = threading.Thread(target=tcpip.move_Place1,args=(self,))
        # self.tcpip = threading.Thread(target=tcpip.move_Place1,args=(self,))
        self.tcpip.start()

    def get_coordinate_Place2(self):
        self.u2.lineEdit.setText('231.235')
        self.u2.lineEdit_2.setText('86.835')
        self.u2.lineEdit_3.setText('638.196')
        self.u2.lineEdit_4.setText('48.696')
        self.u2.lineEdit_5.setText('531.674')
        self.u2.lineEdit_6.setText('-93.537')
        self.tcpip = threading.Thread(target=tcpip.move_Place2, args=(self,))
        self.tcpip.start()

    def get_coordinate_Place3(self):
        self.u2.lineEdit.setText('76.956')
        self.u2.lineEdit_2.setText('86.835')
        self.u2.lineEdit_3.setText('638.197')
        self.u2.lineEdit_4.setText('48.696')
        self.u2.lineEdit_5.setText('531.674')
        self.u2.lineEdit_6.setText('-93.537')
        self.tcpip = threading.Thread(target=tcpip.move_Place3, args=(self,))
        self.tcpip.start()

    def move_success(self):
        QtWidgets.QMessageBox.warning(self,'成功','move success！',QtWidgets.QMessageBox.Ok)

class myWin(QMainWindow,Ui_Dialog):
    def __init__(self):
        super(myWin,self).__init__()
        self.u1 = u1.Ui_Dialog()
        self.u1.setupUi(self)
        self.u1.pushButton.clicked.connect(self.device_running)
        self.u1.pushButton_2.clicked.connect(self.close_and_destroy_device)
        self.u1.pushButton_3.clicked.connect(self.getting_device_txt)
        self.u1.pushButton_4.clicked.connect(self.slot1)
        self.u1.pushButton_5.clicked.connect(self.take_photo)
        self.u1.pushButton_6.clicked.connect(self.connection)
        self.u1.comboBox.addItem("选择相机型号")
        self.cam = MvCamera()

    def slot1(self):
        mysecondwin.show()     #直接用mysecondWin类就不行，用主程序里的mysencondwn就可以，不知道为什么

    def open_warning(self,ret):
        r = QtWidgets.QMessageBox.warning(self,'warning','fail',QtWidgets.QMessageBox.Ok)

    def close_warning_1(self):
        res = QtWidgets.QMessageBox.warning(self,'warning','camera is not running!',QtWidgets.QMessageBox.Ok)

    def close_warning_2(self):
        res1 = QtWidgets.QMessageBox.warning(self,'warning','stop grabbing fail!',QtWidgets.QMessageBox.Ok)

    def close_warning_3(self):
        res2 = QtWidgets.QMessageBox.warning(self,'warning','close device fail!',QtWidgets.QMessageBox.Ok)

    def close_warning_4(self):
        res3 = QtWidgets.QMessageBox.warning(self,'warning','destroy gandle fail!',QtWidgets.QMessageBox.Ok)

    # def dispaly_image(self):
    #     print('1')
    def select_CamType(self):
        res4 = QtWidgets.QMessageBox.warning(self,'warning','Please select the camera you want!',QtWidgets.QMessageBox.Ok)

    def close_and_destroy_device(self):
        # 停止取流
        ret = self.camera.MV_CC_StopGrabbing()
        ret = self.camera.MV_CC_CloseDevice()
        ret = self.camera.MV_CC_DestroyHandle()
        self.u1.pushButton.setText('开始采集')
        self.u1.pushButton.setEnabled(True)

        # if ret != 0:
        #     print("stop grabbing fail! ret[0x%x]" % ret)
        #     self.close_warning_2()
            # 关闭设备
            # ret = cam.MV_CC_CloseDevice()
            # if ret != 0:
            #     print("close deivce fail! ret[0x%x]" % ret)
            #     self.close_warning_3()
            # # 销毁句柄
            # ret = cam.MV_CC_DestroyHandle()
            # if ret != 0:
            #     print("destroy handle fail! ret[0x%x]" % ret)
            #     self.close_warning_4()

    # def get_device(self,device = 0,device_way= False):
    #     if device_way == False:
    #         if device == 0:
    #             tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
    #             deviceList = MV_CC_DEVICE_INFO_LIST()
    #             # 枚举设备
    #             ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)   #不把下拉列表放到ret后面就会出错 我也不知道为什么
    #             for i in range(0, deviceList.nDeviceNum):
    #                 mvcc_dev_info = cast(device
    #                 02List.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
    #             for i in range(deviceList.nDeviceNum):
    #                 self.comboBox.addItem('[{} {}]'.format(i,chr(mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName[i])))
    #             if ret != 0:
    #                 print("enum devices fail! ret[0x%x]" % ret)
    #                 self.open_warning()
    #             if deviceList.nDeviceNum == 0:
    #                 print("find no device!")
    #                 self.open_warning()
    #             print("Find %d devices!" % deviceList.nDeviceNum)
    #     elif device_way == True:
    #         pass

    def take_photo(self):
        self.ext = ExtractOnePic()
        self.ext.signal1.connect(self.show_photo)
        self.ext.start()

    def rotate_bound(image, angle):
        """

        :param image: 原图像
        :param angle: 旋转角度
        :return: 旋转后的图像
        """
        # grab the dimensions of the image and then determine the
        # center
        (h, w) = image.shape[:2]
        # (cX, cY) = (w // 2, h // 2)
        # (cX, cY) = (1385, 1299)
        (cX, cY) = (955, 767)
        # grab the rotation matrix (applying the negative of the
        # angle to rotate clockwise), then grab the sine and cosine
        # (i.e., the rotation components of the matrix)
        M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])

        # compute the new bounding dimensions of the image
        nW = int((h * sin) + (w * cos))
        nH = int((h * cos) + (w * sin))

        # adjust the rotation matrix to take into account translation
        M[0, 2] += (nW / 2) - cX
        M[1, 2] += (nH / 2) - cY
        img = cv2.warpAffine(image, M, (nW, nH))
        # perform the actual rotation and return the image
        return img

    def show_photo(self):
        point = []
        points = []
        angle = []
        kernel = np.ones((5, 5), np.uint8)
        img = cv2.imread('C:/Users/93100/Desktop/0.bmp',0)
        img1 = cv2.imread('C:/Users/93100/Desktop/1.bmp',0)
        img = np.subtract(img, img1)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=3)
        # img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        # thresh = cv2.medianBlur(img,5)
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 17, 9)
        # thresh = cv2.medianBlur(thresh, 9)
        contours, hierachy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        for i in range(len(contours)):
            a = cv2.contourArea(contours[i])
            b = cv2.minAreaRect(contours[i])
            if int(a) > 60000 and int(a) < 120000:
                point.append(contours[i])
            if b[1][0] > 100:
                angle.append(b[2])
        # cv2.drawContours(img, point, -1, (0, 255, 0), 5)
        for i in point:
            M = cv2.moments(i)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            points.append([cX,cY])
            print((cX,cY))
            # 画出轮廓和中点
            cv2.drawContours(img, point, -1, (0, 255, 0), 5)
            cv2.circle(img, (cX, cY), 20, (0, 0, 255), -1)
        # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        _image = QtGui.QImage(img[:], img.shape[1], img.shape[0], img.shape[1] * 3, QtGui.QImage.Format_RGB888)
        jpg_out = QtGui.QPixmap(_image).scaled(self.u1.label.width(), self.u1.label.height())  # 设置图片大小
        self.u1.label.setPixmap(jpg_out)
        self.tcpip = threading.Thread(target=tcpip.catch_parts,args=(self,points[0][0],points[0][1],angle))
        self.tcpip.start()

    def connection(self):
        self.tcpip = threading.Thread(target=tcpip.socket_connect,args=(self,0))
        self.tcpip.start()

    # def detect_parts(self,img):
    #     point = []
    #     kernel = np.ones((5, 5), np.uint8)
    #     img = cv2.imread('C:/Users/93100/Desktop/0.jpg', 0)
    #     # thresh = cv2.medianBlur(img,5)
    #     thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 17, 9)
    #     thresh = cv2.medianBlur(thresh, 9)
    #     contours, hierachy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    #     img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    #     for i in range(len(contours)):
    #         a = cv2.contourArea(contours[i])
    #
    #         if int(a) > 70000 and int(a) < 100000:
    #             point.append(contours[i])
    #     for i in point:
    #         M = cv2.moments(i)
    #         cX = int(M["m10"] / M["m00"])
    #         cY = int(M["m01"] / M["m00"])
    #         # 画出轮廓和中点
    #         cv2.drawContours(img, point, -1, (0, 255, 0), 5)
    #         cv2.circle(img, (cX, cY), 50, (0, 0, 255), -1)
    #     thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
    #     img = cv2.resize(img, (700, 700))
    #     thresh = cv2.resize(thresh, (700, 700))

    def getting_device_txt(self):
        self.u1.pushButton_3.setText('检测中...')
        self.u1.pushButton_3.setEnabled(False)
        self.enum = EnumCamThread()
        self.enum._signal.connect(self.update_btn)
        self.enum._signal1.connect(self.open_warning)
        self.enum.start()

    def update_btn(self,num):
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        for i in range(deviceList.nDeviceNum):
            self.u1.comboBox.addItem('[{}]{}'.format(i, chr(mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName[i])))
        print("Find %d devices!" % deviceList.nDeviceNum)
        self.u1.pushButton_3.setText('检测相机')
        self.u1.pushButton_3.setEnabled(True)

    def device_running(self):
        # nConnectionNum = self.comboBox.currentText()   #判断选择的相机型号是否合法
        # nConnectionNum = str(nConnectionNum)
        # if is_chinese(nConnectionNum) == False:
        #     self.select_CamType()
        self.u1.pushButton.setText('运行中...')
        self.u1.pushButton.setEnabled(False)
        self.run = GrabImage()
        self.run.back.connect(self.update_btn_running)
        self.run.start()
        self.camera = self.run.get_result()

    def update_btn_running(self):
        self.u1.pushButton.setText('开始采集')
        self.u1.pushButton.setEnabled(True)

class ExtractOnePic(QThread,QLabel):   #触发拍照，一次只拍一张
    signal1 = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.mywin = myWin()

    def run(self,index=0):
        # 枚举设备
        deviceList = openCam.enum_devices(self)
        # 判断不同类型设备
        openCam.identify_different_devices(deviceList)
        # 输入需要被连接的设备
        nConnectionNum = openCam.input_num_camera(deviceList)

        cam, stDeviceList = openCam.creat_camera(self.mywin.cam, deviceList, nConnectionNum)

        openCam.open_device(self, self.mywin.cam)
        cam.MV_CC_SetFloatValue("ExposureTime", 5000)
        # 开启设备取流
        openCam.start_grab_and_get_data_size(self.mywin.cam)

        ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        # ch:获取数据包大小 | en:Get payload size
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))

        ret = self.mywin.cam.MV_CC_GetIntValue("PayloadSize", stParam)

        self.nPayloadSize = stParam.nCurValue
        stDeviceList = MV_FRAME_OUT_INFO_EX()
        memset(byref(stDeviceList), 0, sizeof(stDeviceList))
        self.data_buf = (c_ubyte * self.nPayloadSize)()

        ret = self.mywin.cam.MV_CC_GetOneFrameTimeout(byref(self.data_buf), self.nPayloadSize, stDeviceList, 1000)
        if ret == 0:
            # print ("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (stDeviceList.nWidth, stDeviceList.nHeight, stDeviceList.nFrameNum))
            nRGBSize = stDeviceList.nWidth * stDeviceList.nHeight * 3
            stConvertParam = MV_SAVE_IMAGE_PARAM_EX()
            stConvertParam.nWidth = stDeviceList.nWidth
            stConvertParam.nHeight = stDeviceList.nHeight
            stConvertParam.pData = self.data_buf
            stConvertParam.nDataLen = stDeviceList.nFrameLen
            stConvertParam.enPixelType = stDeviceList.enPixelType
            stConvertParam.nImageLen = stConvertParam.nDataLen
            stConvertParam.nJpgQuality = 70
            stConvertParam.enImageType = MV_Image_Jpeg
            stConvertParam.pImageBuffer = (c_ubyte * nRGBSize)()
            stConvertParam.nBufferSize = nRGBSize
            # ret = self.cam.MV_CC_ConvertPixelType(stConvertParam)
            # print(stConvertParam.nImageLen)
            ret = self.mywin.cam.MV_CC_SaveImageEx2(stConvertParam)
            if ret != 0:
                print("convert pixel fail ! ret[0x%x]" % ret)
                del self.data_buf
                sys.exit()
            file_path = "C:/Users/93100/Desktop/" + str(index) + ".bmp"
            file_open = open(file_path.encode('ascii'), 'wb+')
            img_buff = (c_ubyte * stConvertParam.nImageLen)()
            cdll.msvcrt.memcpy(byref(img_buff), stConvertParam.pImageBuffer, stConvertParam.nImageLen)
            file_open.write(img_buff)

        ret = self.mywin.cam.MV_CC_StopGrabbing()
        ret = self.mywin.cam.MV_CC_CloseDevice()
        ret = self.mywin.cam.MV_CC_DestroyHandle()
        self.signal1.emit(str(1))

class EnumCamThread(QThread):    #检测相机线程
    _signal = pyqtSignal(str)
    _signal1 = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.mywin = myWin()

    def run(self, device=0, device_way=False):
        if device_way == False:
            if device == 0:

                tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
                deviceList = MV_CC_DEVICE_INFO_LIST()
                # 枚举设备
                ret = self.mywin.cam.MV_CC_EnumDevices(tlayerType, deviceList)  # 不把下拉列表放到ret后面就会出错 我也不知道为什么
                # for i in range(0, deviceList.nDeviceNum):
                #     mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
                # for i in range(deviceList.nDeviceNum):
                #     self.comboBox.addItem('[{} {}]'.format(i, chr(mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName[i])))
                if ret != 0:
                    print("enum devices fail! ret[0x%x]" % ret)
                    self._signal1.emit(str(ret))
                if deviceList.nDeviceNum == 0:
                    print("find no device!")
                    self._signal1.emit(str(ret))
                # print("Find %d devices!" % deviceList.nDeviceNum)
        self._signal.emit(str(deviceList.nDeviceNum))

class GrabImage(QThread):   #采图线程
    back = pyqtSignal(str)
    def __init__(self,b_open_device = False):
        super(GrabImage, self).__init__()
        self.mywin = myWin()

    def run(self):


        # 枚举设备
        deviceList = openCam.enum_devices(self)
        # 判断不同类型设备
        openCam.identify_different_devices(deviceList)
        # 输入需要被连接的设备
        nConnectionNum = openCam.input_num_camera(deviceList)

        cam, stDeviceList = openCam.creat_camera(self.mywin.cam,deviceList, nConnectionNum)

        openCam.open_device(self,self.mywin.cam)
        cam.MV_CC_SetFloatValue("ExposureTime", 5000)
        # 开启设备取流
        openCam.start_grab_and_get_data_size(self.mywin.cam)
        # 主动取流方式抓取图像
        openCam.access_get_image(self,cam,active_way = 'getImagebuffer')
        # 关闭设备与销毁句柄
        self.back.emit(str(1))


    def get_result(self):
        return self.mywin.cam


class openCam():
    def __init__(self,b_start_grabbing = False):
        super(openCam, self).__init__()

        self.mywin = myWin()
        self.b_start_grabbing = b_start_grabbing
        self.image = None

    def enum_devices(self):
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
        deviceList = MV_CC_DEVICE_INFO_LIST()
                # 枚举设备
        ret = self.mywin.cam.MV_CC_EnumDevices(tlayerType, deviceList)
        return deviceList

    # def close_and_destroy_device(self):
    #     # 停止取流
    #     cam = MvCamera()
    #     if False == self.b_start_grabbing:
    #         self.close_warning_1()
    #     elif True == self.b_start_grabbing:
    #         ret = cam.MV_CC_StopGrabbing()
    #         self.pushButton.setText('开始采集')
    #         self.pushButton.setEnabled(True)
    #         if ret != 0:
    #             print("stop grabbing fail! ret[0x%x]" % ret)
    #             self.close_warning_2()
    #         # 关闭设备
    #         ret = cam.MV_CC_CloseDevice()
    #         if ret != 0:
    #             print("close deivce fail! ret[0x%x]" % ret)
    #             self.close_warning_3()
    #         # 销毁句柄
    #         ret = cam.MV_CC_DestroyHandle()
    #         if ret != 0:
    #             print("destroy handle fail! ret[0x%x]" % ret)
    #             self.close_warning_4()

    def identify_different_devices(deviceList):
        # 判断不同类型设备，并输出相关信息
        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            # 判断是否为网口相机
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                print("\n网口设备序号: [%d]" % i)
                # 获取设备名
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                    strModeName = strModeName + chr(per)
                print("当前设备型号名: %s" % strModeName)
                # 获取当前设备 IP 地址
                nip1_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                nip1_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                nip1_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                nip1_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                print("当前 ip 地址: %d.%d.%d.%d" % (nip1_1, nip1_2, nip1_3, nip1_4))
                # 获取网口 IP 地址
                nip4_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0xff000000) >> 24)
                nip4_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x00ff0000) >> 16)
                nip4_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x0000ff00) >> 8)
                nip4_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x000000ff)
                print("当前连接的网口 IP 地址 : %d.%d.%d.%d" % (nip4_1, nip4_2, nip4_3, nip4_4))

    def input_num_camera(deviceList):
        # nConnectionNum = input("please input the number of the device to connect:")
        nConnectionNum = mywin.u1.comboBox.currentText()
        nConnectionNum = str(nConnectionNum)
        # if isinstance(nConnectionNum,int) == False:    #判断选择的相机型号是否合法
        #     print('you dont select the right camera!')
        #     sys.exit()
        # if int(nConnectionNum[1]) >= deviceList.nDeviceNum:
        #     print("intput error!")
        #     sys.exit()
        return int(nConnectionNum[1])

    def creat_camera(cam,deviceList, nConnectionNum, log=True, log_path=getcwd()):
        """
        :param deviceList:        设备列表
        :param nConnectionNum:    需要连接的设备序号
        :param log:               是否创建日志
        :param log_path:          日志保存路径
        :return:                  相机实例和设备列表
        """
        # 创建相机实例
        stDeviceList = cast(deviceList.pDeviceInfo[nConnectionNum], POINTER(MV_CC_DEVICE_INFO)).contents
        if log == True:
            ret = cam.MV_CC_SetSDKLogPath(log_path)
            print(log_path)
            if ret != 0:
                print("set Log path  fail! ret[0x%x]" % ret)
                sys.exit()
            # 创建句柄,生成日志
            ret = cam.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                print("create handle fail! ret[0x%x]" % ret)
                sys.exit()
        elif log == False:
            # 创建句柄,不生成日志
            ret = cam.MV_CC_CreateHandleWithoutLog(stDeviceList)
            print(1111)
            if ret != 0:
                print("create handle fail! ret[0x%x]" % ret)
                sys.exit()
        return cam, stDeviceList

    def open_device(self,cam):
        # ch:打开设备 | en:Open device
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print("open device fail! ret[0x%x]" % ret)
            sys.exit()

    # def image_show(image, name):
    #     image = cv2.resize(image, (600, 400), interpolation=cv2.INTER_AREA)
    #     name = str(name)
    #     cv2.imshow(name, image)
    #     k = cv2.waitKey(1) & 0xff
    #
    # def image_control(data, stFrameInfo):    #显示图像
    #     if stFrameInfo.enPixelType == 17301505:
    #         image = data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
    #         openCam.image_show(image=image, name=stFrameInfo.nHeight)
    #     # elif stFrameInfo.enPixelType == 17301514:
    #     #     data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
    #     #     image = cv2.cvtColor(data, cv2.COLOR_BAYER_GB2RGB)
    #     #     openCam.image_show(image=image, name=stFrameInfo.nHeight)
    #     # elif stFrameInfo.enPixelType == 35127316:
    #     #     data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
    #     #     image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    #     #     openCam.image_show(image=image, name=stFrameInfo.nHeight)
    #     # elif stFrameInfo.enPixelType == 34603039:
    #     #     data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
    #     #     image = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
    #     #     openCam.image_show(image=image, name=stFrameInfo.nHe

    def access_get_image(self,cam,active_way = 'getImagebuffer'):
        if active_way == "getImagebuffer":
            stOutFrame = MV_FRAME_OUT()
            memset(byref(stOutFrame), 0, sizeof(stOutFrame))
            while True:    #该循环可以运行，用于循环主动取流抓取图像
                ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
                # print(stOutFrame.stFrameInfo.enPixelType)
                # print(hex(ret))
                if None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 17301505:
                    # print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                    pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                    cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                    data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight),dtype=np.uint8)
                    if stOutFrame.stFrameInfo.enPixelType == 17301505:
                        self.image = data.reshape((stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth))
                        self.image = cv2.resize(self.image, (600, 400), interpolation=cv2.INTER_AREA)
                        name = str(stOutFrame.stFrameInfo.nHeight)
                        cv2.imshow(name, self.image)
                        k = cv2.waitKey(1) & 0xff
                        # cv2.destroyAllWindows()
                    elif stOutFrame.stFrameInfo.enPixelType == 0:
                        pass
                    nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)  #注意要释放imagebuffer，否则会阻塞


    def start_grab_and_get_data_size(cam):
        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            print("开始取流失败! ret[0x%x]" % ret)
            sys.exit()


    # def close_and_destroy_device(cam, data_buf=None):
    #     # 停止取流
    #     global b_is_run
    #     if b_is_run == False:
    #         myWin.close_warning()
    #     elif b_is_run == True:
    #         ret = cam.MV_CC_StopGrabbing()
    #         if ret != 0:
    #             print("stop grabbing fail! ret[0x%x]" % ret)
    #             sys.exit()
    #         # 关闭设备
    #         ret = cam.MV_CC_CloseDevice()
    #         if ret != 0:
    #             print("close deivce fail! ret[0x%x]" % ret)
    #             del data_buf
    #             sys.exit()
    #         # 销毁句柄
    #         ret = cam.MV_CC_DestroyHandle()
    #         if ret != 0:
    #             print("destroy handle fail! ret[0x%x]" % ret)
    #             del data_buf
    #             sys.exit()
    #         del data_buf


if __name__ == '__main__':

    app = QtWidgets.QApplication(sys.argv)
    mywin = myWin()
    mywin.show()
    mysecondwin = mysencondWin()
    sys.exit(app.exec_())
