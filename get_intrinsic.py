import numpy as np
import cv2
import glob
from fractions import Fraction

w = 6
h = 8

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
mtx = np.zeros((w*h,3),np.float32)
mtx[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
mtx = mtx * 10
# print(mtx)
image = glob.glob('C:/Users/93100/MVS/Data/*.bmp')
# print(image)
# img = str('C:/Users/93100/MVS/Data/15.bmp')

objpoints = [] # 在世界坐标系中的三维点
imgpoints = [] # 在图像平面的二维点
i = 0
# np.set_printoptions(formatter={'all':lambda x: str(Fraction(x).limit_denominator())})'float':'{:.3f}'.format}
np.set_printoptions(formatter={'float':'{:.15f}'.format})
for img in image:
        i = i + 1
        img = cv2.imread(str(img))
        gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        u,v = img.shape[:2]
        ret, corners = cv2.findChessboardCorners(gray, (w,h),None)
        if ret == True:
                # 在原角点的基础上寻找亚像素角点
                cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                #追加进入世界三维点和平面二维点中
                objpoints.append(mtx)   #世界坐标系
                imgpoints.append(corners)  #图像坐标系
                # 将角点在图像上显示
                cv2.drawChessboardCorners(img, (w,h), corners, ret)
                cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('findCorners', 640, 480)
                cv2.imshow('findCorners',img)
                cv2.waitKey(0)

# img = cv2.imread(str(img))
# gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
# u,v = img.shape[:2]
# ret, corners = cv2.findChessboardCorners(gray, (w,h),None)
# # print(corners)
# if ret == True:
#         # 在原角点的基础上寻找亚像素角点
#         cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
#         #追加进入世界三维点和平面二维点中
#         objpoints.append(mtx)
#         imgpoints.append(corners)
#         # 将角点在图像上显示
#         cv2.drawChessboardCorners(img, (w,h), corners, ret)
#         cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
#         cv2.resizeWindow('findCorners', 640, 480)
#         cv2.imshow('findCorners',img)
#         cv2.waitKey(0)


cv2.destroyAllWindows()
#%% 标定
print('正在计算')
#标定
#rvecs表示棋盘世界坐标系到相机坐标系的旋转矩阵
#tvecs表示棋盘世界坐标系到相机坐标系的平移矩阵
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# print(cv2.Rodrigues(rvecs[0])[0])
RT = np.column_stack(((cv2.Rodrigues(rvecs[0])[0]),tvecs[0]))
RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
total_error = 0
# for i in range(len(objpoints)):
#     imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
#     # print(imgpoints2)
#     error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
#     total_error += error
print("total error: ", total_error/len(objpoints))
# print("ret:",ret  )
# print("mtx:\n",mtx)      # 内参数矩阵
# print("dist畸变值:\n",dist   )   # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
# print("rvecs旋转（向量）外参:\n",rvecs)   # 旋转向量  # 外参数
# print("tvecs平移（向量）外参:\n",tvecs)    # 平移向量  # 外参数
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (u, v), 0, (u, v))
print('newcameramtx外参',newcameramtx)