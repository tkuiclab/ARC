#!/usr/bin/env python
# coding:utf8

import cv2
import numpy as np
import copy
import rospy
from fake_roi.srv import Detect, DetectResponse

_MASK_SIZE = 5
_OBJ_AREA_MIN = 3000
_OBJ_AREA_MAX = 80000


def detect_KNN(bg, fg):

    history = 15

    bs = cv2.createBackgroundSubtractorKNN(detectShadows=True)  # 背景减除器，设置阴影检测
    bs.setkNNSamples(3)
    bs.setHistory(history)

    frames = 0
    min = []
    max = []
    while True:
        frame = bg if frames < history else fg
        fg_mask = bs.apply(frame)   # 获取 foreground mask

        if frames < history:
            frames += 1
            continue

        # 对原始帧进行膨胀去噪
        th = cv2.threshold(copy.copy(fg_mask), 245, 255, cv2.THRESH_BINARY)[1]

        kernel = np.ones((_MASK_SIZE, _MASK_SIZE), np.uint8)
        res = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)
        res = cv2.medianBlur(res, 5)

        # 获取所有检测框
        image, contours, hier = cv2.findContours(
            res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_obj = {'bound': None, 'area': 0}
        for c in contours:
            # 获取矩形框边界坐标
            x, y, w, h = cv2.boundingRect(c)
            # 计算矩形框的面积
            area = cv2.contourArea(c)
            # print area

            if _OBJ_AREA_MIN < area:
                if max_obj['area'] < area:
                    max_obj['area'] = area
                    max_obj['bound'] = [x, y, w, h]

        if max_obj['bound'] is not None:
            [x, y, w, h] = max_obj['bound']
            min.append(x)
            min.append(y)
            max.append(x+w)
            max.append(y+h)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            min.append(0)
            min.append(0)
            max.append(0)
            max.append(0)
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return frame, res, fg_mask, min, max


def detect_MOG(bg, fg):

    bs = cv2.bgsegm.createBackgroundSubtractorMOG()
    bs.apply(bg)
    min = []
    max = []
    fg_mask = bs.apply(fg)   # 获取 foreground mask

    # 对原始帧进行膨胀去噪
    th = cv2.threshold(copy.copy(fg_mask), 245, 255,
                       cv2.THRESH_BINARY)[1]

    kernel = np.ones((_MASK_SIZE, _MASK_SIZE), np.uint8)
    res = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)

    # 获取所有检测框
    image, contours, hier = cv2.findContours(
        res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        # 获取矩形框边界坐标
        x, y, w, h = cv2.boundingRect(c)

        # 计算矩形框的面积
        area = cv2.contourArea(c)
        if _OBJ_AREA_MIN < area < _OBJ_AREA_MAX:
            cv2.rectangle(fg, (x, y), (x + w, y + h), (0, 255, 0), 2)
            min.append(x)
            min.append(y)
            max.append(x + w)
            max.append(y + h)

    return fg, res, fg_mask, min, max


def detect_MOG2(bg, fg):

    history = 20
    min = []
    max = []
    bs = cv2.createBackgroundSubtractorMOG2(
        history=history, detectShadows=True)  # 背景减除器，设置阴影检测
    bs.setHistory(history)

    frames = 0

    while True:
        frame = bg if frames < history else fg
        fg_mask = bs.apply(frame)   # 获取 foreground mask

        if frames < history:
            frames += 1
            continue

        # 对原始帧进行膨胀去噪
        th = cv2.threshold(copy.copy(fg_mask), 245, 255, cv2.THRESH_BINARY)[1]

        kernel = np.ones((_MASK_SIZE, _MASK_SIZE), np.uint8)
        res = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)

        # 获取所有检测框
        image, contours, hier = cv2.findContours(
            res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            # 获取矩形框边界坐标
            x, y, w, h = cv2.boundingRect(c)

            # 计算矩形框的面积
            area = cv2.contourArea(c)
            print (area)

            if _OBJ_AREA_MIN < area < _OBJ_AREA_MAX:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                min.append(x)
                min.append(y)
                max.append(x + w)
                max.append(y + h)

        return frame, res, fg_mask, min, max


def detect_obj(src, mask):

    kernel = np.ones((_MASK_SIZE, _MASK_SIZE), np.uint8)
    res = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 获取所有检测框
    image, contours, hier = cv2.findContours(
        res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_obj = {'bound': None, 'area': 0}
    for c in contours:
        # 获取矩形框边界坐标
        x, y, w, h = cv2.boundingRect(c)
        # 计算矩形框的面积
        area = cv2.contourArea(c)
        if _OBJ_AREA_MIN < area < _OBJ_AREA_MAX:
            if max_obj['area'] < area:
                max_obj['bound'] = [x, y, w, h]

    if max_obj['bound'] is not None:
        [x, y, w, h] = max_obj['bound']
        cv2.rectangle(src, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return src, res

def handle_request(req):
    bg = cv2.imread('/home/iclab-giga/Scene_empty.jpg')
    fg = cv2.imread('/home/iclab-giga/Scene_with_handweight.jpg')

    frame, res, fg_mask ,min, max = detect_KNN(copy.copy(bg), copy.copy(fg))
    return DetectResponse([min[0], min[1], max[0], max[1]], True)

if __name__ == '__main__':

    rospy.init_node('object_detect', anonymous=True)
    rospy.Service('detect', Detect, handle_request)
    rospy.spin()
'''
    # _test()
    min = []
    max = []
    from os.path import isfile, join, expanduser

    bg_folder = './image-20170417/empty'
    fg_folder = './image-20170417/colgate_toothbrushs/001'
    bg_name = 'empty'
    fb_name = 'colgate_toothbrushs'

    for filename in range(1, 20 + 1):
        #bg = cv2.imread(expanduser(
        #    join(bg_folder, '{}-{:05}.jpg'.format(bg_name, filename))))
        #fg = cv2.imread(expanduser(
        #    join(fg_folder, '{}-{:05}.jpg'.format(fb_name, filename))))

        bg = cv2.imread('/home/iclab-giga/Scene_empty.jpg')
        fg = cv2.imread('/home/iclab-giga/Scene_with_handweight.jpg')

        frame, res, fg_mask ,min, max = detect_KNN(copy.copy(bg), copy.copy(fg))
        # print ''
        # frame, res, fg_mask = detect_MOG(copy.copy(bg), copy.copy(fg))
        # frame, res, fg_mask = detect_MOG2(copy.copy(bg), copy.copy(fg))

        cv2.imshow("detection", frame)
        print "min: ",min," max: ",max
        cv2.imshow("back", res)
        cv2.imshow("fg_mask", fg_mask)

        if cv2.waitKey(0) & 0xFF == 27:
            break
'''
