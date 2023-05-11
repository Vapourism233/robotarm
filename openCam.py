import os
import sys
import numpy as np
from os import getcwd
import cv2
import msvcrt
from ctypes import *

sys.path.append("C:/pythonProject/DownloadCode/Python_HIKVISION_Samples/MvImport")
from MvCameraControl_class import *

class openCam:
    def __init__(self):
        global b_is_run
        b_is_run = False

    def enum_devices(device = 0,device_way= False):
        if device_way == False:
            if device == 0:
                tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
                deviceList = MV_CC_DEVICE_INFO_LIST()
                # 枚举设备
                ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
                if ret != 0:
                    print("enum devices fail! ret[0x%x]" % ret)
                    sys.exit()
                if deviceList.nDeviceNum == 0:
                    print("find no device!")
                    sys.exit()
                print("Find %d devices!" % deviceList.nDeviceNum)
                return deviceList
            else:
                pass
        elif device_way == True:
            pass

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
        nConnectionNum = input("please input the number of the device to connect:")
        if int(nConnectionNum) >= deviceList.nDeviceNum:
            print("intput error!")
            sys.exit()
        return nConnectionNum

    def creat_camera(deviceList, nConnectionNum, log=True, log_path=getcwd()):
        """
        :param deviceList:        设备列表
        :param nConnectionNum:    需要连接的设备序号
        :param log:               是否创建日志
        :param log_path:          日志保存路径
        :return:                  相机实例和设备列表
        """
        # 创建相机实例
        cam = MvCamera()
        # 选择设备并创建句柄
        stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
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

    def open_device(cam):
        # ch:打开设备 | en:Open device
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print("open device fail! ret[0x%x]" % ret)
            sys.exit()

    def image_show(image, name):
        image = cv2.resize(image, (600, 400), interpolation=cv2.INTER_AREA)
        name = str(name)
        cv2.imshow(name, image)
        cv2.imwrite("name.bmp", image)
        k = cv2.waitKey(1) & 0xff

    def image_control(data, stFrameInfo):
        if stFrameInfo.enPixelType == 17301505:
            image = data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
            openCam.image_show(image=image, name=stFrameInfo.nHeight)
        elif stFrameInfo.enPixelType == 17301514:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            image = cv2.cvtColor(data, cv2.COLOR_BAYER_GB2RGB)
            openCam.image_show(image=image, name=stFrameInfo.nHeight)
        elif stFrameInfo.enPixelType == 35127316:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            openCam.image_show(image=image, name=stFrameInfo.nHeight)
        elif stFrameInfo.enPixelType == 34603039:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            image = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
            openCam.image_show(image=image, name=stFrameInfo.nHeight)

    winfun_ctype = WINFUNCTYPE
    stFrameInfo = POINTER(MV_FRAME_OUT_INFO_EX)
    pData = POINTER(c_ubyte)
    FrameInfoCallBack = winfun_ctype(None, pData, stFrameInfo, c_void_p)

    def image_callback(pData, pFrameInfo, pUser):
        global img_buff
        img_buff = None
        stFrameInfo = cast(pFrameInfo, POINTER(MV_FRAME_OUT_INFO_EX)).contents
        if stFrameInfo:
            print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
            stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
        if img_buff is None and stFrameInfo.enPixelType == 17301505:
            img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight)()
            cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight)
            data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight), dtype=np.uint8)
            openCam.image_control(data=data, stFrameInfo=stFrameInfo)
            del img_buff
        elif img_buff is None and stFrameInfo.enPixelType == 17301514:
            img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight)()
            cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight)
            data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight), dtype=np.uint8)
            openCam.image_control(data=data, stFrameInfo=stFrameInfo)
            del img_buff
        elif img_buff is None and stFrameInfo.enPixelType == 35127316:
            img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight * 3)()
            cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight * 3)
            data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight * 3), dtype=np.uint8)
            openCam.image_control(data=data, stFrameInfo=stFrameInfo)
            del img_buff
        elif img_buff is None and stFrameInfo.enPixelType == 34603039:
            img_buff = (c_ubyte * stFrameInfo.nWidth * stFrameInfo.nHeight * 2)()
            cdll.msvcrt.memcpy(byref(img_buff), pData, stFrameInfo.nWidth * stFrameInfo.nHeight * 2)
            data = np.frombuffer(img_buff, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight * 2), dtype=np.uint8)
            openCam.image_control(data=data, stFrameInfo=stFrameInfo)
            del img_buff

    CALL_BACK_FUN = FrameInfoCallBack(image_callback)

    def call_back_get_image(cam):
        # ch:注册抓图回调 | en:Register image callback
        ret = cam.MV_CC_RegisterImageCallBackEx(openCam.CALL_BACK_FUN, None)
        if ret != 0:
            print("register image callback fail! ret[0x%x]" % ret)
            sys.exit()

    def start_grab_and_get_data_size(cam):
        global b_is_run
        ret = cam.MV_CC_StartGrabbing()
        if ret == 0:
            b_is_run = True
        if ret != 0:
            print("开始取流失败! ret[0x%x]" % ret)
            sys.exit()

    def close_and_destroy_device(cam, data_buf=None):
        # 停止取流
        global b_is_run
        if b_is_run == False:
            print("camera is not running!")
            sys.exit()
        ret = cam.MV_CC_StopGrabbing()
        if ret != 0:
            print("stop grabbing fail! ret[0x%x]" % ret)
            sys.exit()
        # 关闭设备
        ret = cam.MV_CC_CloseDevice()
        if ret != 0:
            print("close deivce fail! ret[0x%x]" % ret)
            del data_buf
            sys.exit()
        # 销毁句柄
        ret = cam.MV_CC_DestroyHandle()
        if ret != 0:
            print("destroy handle fail! ret[0x%x]" % ret)
            del data_buf
            sys.exit()
        del data_buf

def main():
    # 枚举设备
    global b_is_run
    deviceList = openCam.enum_devices(device=0, device_way=False)
    # 判断不同类型设备
    openCam.identify_different_devices(deviceList)
    # 输入需要被连接的设备
    nConnectionNum = openCam.input_num_camera(deviceList)

    cam, stDeviceList = openCam.creat_camera(deviceList, nConnectionNum)

    openCam.open_device(cam)

    openCam.call_back_get_image(cam)
        # 开启设备取流
    openCam.start_grab_and_get_data_size(cam)
        # 当使用 回调取流时，需要在此处添加
    print("press a key to stop grabbing.")
    msvcrt.getch()
        # 关闭设备与销毁句柄
    openCam.close_and_destroy_device(cam)


if __name__ == '__main__':
    main()
