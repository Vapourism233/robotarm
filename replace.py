import cv2
import numpy as np
import time

class Replace_Template:
    def py_nms(dets, thresh):
        """pure python nms baseline."""
        # x1、y1、x2、y2、以及score赋值
        # （x1、y1）（x2、y2）为box的左上和右下角标
        x1 = dets[:, 0]
        y1 = dets[:, 1]
        x2 = dets[:, 2]
        y2 = dets[:, 3]
        scores = dets[:, 4]
        # 每一个候选框的面积
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        # order是按照score降序排序的
        order = scores.argsort()[::-1]
        # print("order:",order)

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            # 计算当前概率最大矩形框与其他矩形框的相交框的坐标，会用到numpy的broadcast机制，得到的是向量
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            # print(x1[order[1:]])
            # 计算相交框的面积,注意矩形框不相交时w或h算出来会是负数，用0代替
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            # 计算重叠度iou：重叠面积/（面积1+面积2-重叠面积）
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            # 找到重叠度不高于阈值的矩形框索引
            inds = np.where(ovr <= thresh)[0]
            # print("inds:",inds)
            # 将order序列更新，由于前面得到的矩形框索引要比矩形框在原order序列中的索引小1，所以要把这个1加回来
            order = order[inds + 1]
        return keep

    def template(img_gray, template_img, template_threshold):
        '''
        img_gray:待检测的灰度图片格式
        template_img:模板小图，也是灰度化了
        template_threshold:模板匹配的置信度
        '''

        h, w = template_img.shape[:2]
        res = cv2.matchTemplate(img_gray, template_img, cv2.TM_CCOEFF_NORMED)
        start_time = time.time()
        loc = np.where(res >= template_threshold)  # 大于模板阈值的目标坐标

        score = res[res >= template_threshold]  # 大于模板阈值的目标置信度
        # 将模板数据坐标进行处理成左上角、右下角的格式
        xmin = np.array(loc[1])
        ymin = np.array(loc[0])
        xmax = xmin + w
        ymax = ymin + h
        xmin = xmin.reshape(-1, 1)  # 变成n行1列维度
        xmax = xmax.reshape(-1, 1)  # 变成n行1列维度
        ymax = ymax.reshape(-1, 1)  # 变成n行1列维度
        ymin = ymin.reshape(-1, 1)  # 变成n行1列维度
        score = score.reshape(-1, 1)  # 变成n行1列维度
        data_hlist = []
        data_hlist.append(xmin)
        data_hlist.append(ymin)
        data_hlist.append(xmax)
        data_hlist.append(ymax)
        data_hlist.append(score)
        data_hstack = np.hstack(data_hlist)  # 将xmin、ymin、xmax、yamx、scores按照列进行拼接
        thresh = 0.1  # nms里面的iou交互比阈值

        keep_dets = Replace_Template.py_nms(data_hstack, thresh)
        # print("nms time:", time.time() - start_time)  # 打印数据处理到nms运行时间
        dets = data_hstack[keep_dets]  # 最终的nms获得的矩形框
        return dets

    def cut(img, left1, left2, right1, right2, i):
        screenshot = img[right1:right2, left1:left2]
        screenshot = cv2.cvtColor(screenshot, cv2.COLOR_RGB2GRAY)
        return screenshot

    def exercute(path1,path2):

        np.set_printoptions(threshold=np.inf)
        img_rgb = cv2.imread(path1)  # 需要检测的图片
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)  # 转化成灰色
        template_img = cv2.imread(path2, 0)  # 模板小图
        template_threshold = 0.8  # 模板置信度
        dets = Replace_Template.template(img_gray, template_img, template_threshold)
        i = 0
        sum1 = np.zeros((int(template_img.shape[0]), int(template_img.shape[1])))

        for coord in dets:
            i += 1
            left = []
            right = []
            left.append((coord[0], coord[2]))
            right.append((coord[1], coord[3]))
            sum = Replace_Template.cut(img_rgb, int(left[0][0]), int(left[0][1]), int(right[0][0]), int(right[0][1]), i)
            sum1 = sum1 + sum

        sum1 = sum1 / dets.shape[0]
        sum1 = sum1.astype(np.uint8)
        cv2.imwrite(path2,sum1)

        # cv2.imshow('1',sum1)
        # cv2.waitKey()
        # print(left)
        # print(right)
        # print(list(set(coord)))

