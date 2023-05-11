import cv2
import numpy as np
import glob
from math import *
import pandas as pd
import os

# K=np.array([[6519.775923856544978,0.000000000000000,926.421162990203243],
#  [0.000000000000000,6545.784343329142757,138.971923583354140],
#  [0.000000000000000,0.000000000000000,1.000000000000000]])  #海康威视相机内参

K = np.array([[9928.3539317642,0.0000000000,1257.6364470423],
 [0.0000000000,10033.2476598101,1713.5120499046],
 [0.0000000000,0.0000000000,1.0000000000]])

# K = [[8483.232008654415040 0.000000000000000 1239.313031247763774]
# #  [0.000000000000000 8517.699245337436878 592.105660141382032]
# #  [0.000000000000000 0.000000000000000 1.000000000000000]])

chess_board_x_num=6#棋盘格x方向格子数
chess_board_y_num=8#棋盘格y方向格子数
chess_board_len=8#单位棋盘格长度,mm

#用于根据欧拉角计算旋转矩阵
def myRPY2R_robot(x, y, z):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rx@Ry@Rz
    return R

#用于根据位姿计算变换矩阵
def pose_robot(x, y, z, Tz, Ty, Tx):
    thetaZ = Tz / 180 * pi
    thetaY = Ty / 180 * pi
    thetaX = Tx / 180 * pi
    R = myRPY2R_robot(thetaX, thetaY, thetaZ)
    t = np.array([[x], [y], [z]])
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
    # RT1=np.linalg.inv(RT1)
    return RT1

#用来从棋盘格图片得到相机外参
def get_RT_from_chessboard(img_path,chess_board_x_num,chess_board_y_num,K,chess_board_len):
    '''
    :param img_path: 读取图片路径
    :param chess_board_x_num: 棋盘格x方向格子数
    :param chess_board_y_num: 棋盘格y方向格子数
    :param K: 相机内参
    :param chess_board_len: 单位棋盘格长度,mm
    :return: 相机外参
    '''
    img=cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, (chess_board_x_num, chess_board_y_num), None)  #该函数找到的corners是像素坐标系
    # print(corners)
    corner_points=np.zeros((2,corners.shape[0]),dtype=np.float64)
    for i in range(corners.shape[0]):
        corner_points[:,i]=corners[i,0,:]
    # print(corner_points)
    object_points=np.zeros((3,chess_board_x_num*chess_board_y_num),dtype=np.float64)
    flag=0
    for i in range(chess_board_y_num):
        for j in range(chess_board_x_num):
            object_points[:2,flag]=np.array([(7-j-1)*chess_board_len,(7-i-1)*chess_board_len])
            flag+=1
    # print(object_points)

    #solvePnP用于获取世界坐标系到像素坐标系的转换矩阵
    retval,rvec,tvec  = cv2.solvePnP(object_points.T,corner_points.T, K, distCoeffs=None)
    # print(rvec.reshape((1,3)))
    # RT=np.column_stack((rvec,tvec))

    #RT表示将世界坐标系转换为像素坐标系的矩阵
    RT = np.column_stack(((cv2.Rodrigues(rvec)[0]),tvec))
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    # RT=pose(rvec[0,0],rvec[1,0],rvec[2,0],tvec[0,0],tvec[1,0],tvec[2,0])
    # print(RT)

    # print(retval, rvec, tvec)
    # print(RT)
    # print('')
    return RT

folder = r"C:/Users/93100/MVS/Data"#棋盘格图片存放文件夹
# np.set_printoptions(threshold=np.inf)
np.set_printoptions(formatter={'float':'{:.10f}'.format},threshold=np.inf)

# files = os.listdir(folder)
# file_num=len(files)
# RT_all=np.zeros((4,4,file_num))

# print(get_RT_from_chessboard('calib/2.bmp', chess_board_x_num, chess_board_y_num, K, chess_board_len))
good_picture=[1,2,3,4,5,6,7,8,9,10,11]#存放可以检测出棋盘格角点的图片
# good_picture=[1,3,10,11,12]
file_num=len(good_picture)

#世界坐标系对像素坐标系的变换矩阵
R_all_chess_to_cam_1=[]
T_all_chess_to_cam_1=[]
for i in good_picture:
    # print(i)
    image_path=folder+'/'+str(i)+'.bmp'
    RT=get_RT_from_chessboard(image_path, chess_board_x_num, chess_board_y_num, K, chess_board_len)

    # RT=np.linalg.inv(RT)

    R_all_chess_to_cam_1.append(RT[:3,:3])   #R_all_chess_to_cam_1取外参矩阵的左上3*3部分，即旋转矩阵
    T_all_chess_to_cam_1.append(RT[:3, 3].reshape((3,1)))   #T_all_chess_to_cam_1取外参矩阵的右上3*1部分，即平移向量

# print(T_all_chess_to_cam.shape)

#计算end to base变换矩阵
file_address=r"C:/Users/93100/MVS/Data/0.xlsx"#从记录文件读取机器人六个位姿
sheet_1 = pd.read_excel(file_address)
R_all_end_to_base_1=[]   #机械臂工具坐标系对于基坐标系的变换矩阵
T_all_end_to_base_1=[]
# print(sheet_1.iloc[0]['ax'])
for i in good_picture:
    # print(sheet_1.iloc[i-1]['ax'],sheet_1.iloc[i-1]['ay'],sheet_1.iloc[i-1]['az'],sheet_1.iloc[i-1]['dx'],
    #                                   sheet_1.iloc[i-1]['dy'],sheet_1.iloc[i-1]['dz'])
    #RT代表机械臂末端对机器人基底坐标系的转换矩阵
    RT=pose_robot(sheet_1.iloc[i-1]['ax'],sheet_1.iloc[i-1]['ay'],sheet_1.iloc[i-1]['az'],sheet_1.iloc[i-1]['dz'],
                                      sheet_1.iloc[i-1]['dy'],sheet_1.iloc[i-1]['dx'])

    # RT=np.column_stack(((cv2.Rodrigues(np.array([[sheet_1.iloc[i-1]['ax']],[sheet_1.iloc[i-1]['ay']],[sheet_1.iloc[i-1]['az']]])))[0],
    #                    np.array([[sheet_1.iloc[i-1]['dx']],
    #                                   [sheet_1.iloc[i-1]['dy']],[sheet_1.iloc[i-1]['dz']]])))
    # RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    # RT = np.linalg.inv(RT)

    R_all_end_to_base_1.append(RT[:3, :3])
    T_all_end_to_base_1.append(RT[:3, 3].reshape((3, 1)))

# print(R_all_end_to_base_1)
R,T=cv2.calibrateHandEye(R_all_end_to_base_1,T_all_end_to_base_1,R_all_chess_to_cam_1,T_all_chess_to_cam_1,method=cv2.CALIB_HAND_EYE_TSAI)#手眼标定
# print('旋转矩阵为：')
# print(R)
RT=np.column_stack((R,T))
RT = np.row_stack((RT, np.array([0, 0, 0, 1])))#即为cam to end变换矩阵
# print(RT)
# print('相机相对于末端的变换矩阵为：')
# print(R_all_end_to_base_1)
for i in range(len(good_picture)):

    # 得到机械手末端到基座的变换矩阵，通过机械手末端到基座的旋转矩阵与平移向量先按列合并，然后按行合并形成变换矩阵格式
    RT_end_to_base=np.column_stack((R_all_end_to_base_1[i],T_all_end_to_base_1[i]))
    RT_end_to_base=np.row_stack((RT_end_to_base,np.array([0,0,0,1])))
    # print(RT_end_to_base)

    # 标定版相对于相机的齐次矩阵
    RT_chess_to_cam=np.column_stack((R_all_chess_to_cam_1[i],T_all_chess_to_cam_1[i]))
    RT_chess_to_cam=np.row_stack((RT_chess_to_cam,np.array([0,0,0,1])))
    # print(RT_chess_to_cam)

    # 手眼标定变换矩阵
    RT_cam_to_end=np.column_stack((R,T))
    RT_cam_to_end=np.row_stack((RT_cam_to_end,np.array([0,0,0,1])))

    RT_chess_to_base=RT_chess_to_cam@RT_cam_to_end@RT_end_to_base
    # RT_chess_to_base=np.linalg.inv(RT_chess_to_base[:3,:3])
    # RT_chess_to_base=np.linalg.inv(RT_chess_to_base)
    print('第',i,'次')
    print(RT_chess_to_base[:3,:])
