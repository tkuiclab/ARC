#!/usr/bin/env python

"""Get information of item in json."""

from __future__ import print_function
from os.path import join
import sys
import rospkg
import json
import glob
from task_parser import read_json


class ObjInfo:
    """Information of objects were defined."""

    def __init__(self, **kwargs):
        """Init object for information of objects."""
        self.name = kwargs.get('name')
        self.dimensions = kwargs.get('dims')
        self.weight = kwargs.get('weight')
        self.type = kwargs.get('type')


class BoxInfo:
    """Information of boxes were defined."""

    def __init__(self, **kwargs):
        """Init object for information of boxes."""
        self.name = kwargs.get('name')
        self.dimensions = kwargs.get('dims')


def _get_path(file='obj'):
    pkg_path = rospkg.RosPack().get_path('arc')
    if file == 'box':
        return join(pkg_path, 'pick_task', 'box_sizes.json')
    elif file == 'order_box':
        return join(pkg_path, 'pick_task', 'order_file_test.json')
    else:
        return join(pkg_path, 'Training items')


def json_parser_obj(path):
    try:
        content = read_json(path)
        return ObjInfo(
            name=content['name'],
            type=content['type'],
            weight=content['weight'],
            dims=content['dimensions']
        )
    except Exception as e:
        print('============== Exception ==============')
        print(path, e)
        sys.exit(-1)


def json_parser_box(path):
    try:
        content = read_json(path)
        box_dict = dict()
        for box in content['boxes']:
            name = box['size_id']
            dims = box['dimensions']
            box_dict[name] = BoxInfo(name=name, dims=dims)
        return box_dict
    except Exception as e:
        print('============== Exception ==============')
        print(path, e)
        sys.exit(-1)


def json_parser_order(path):
    try:
        content = read_json(path)
        box_list = list()
        for box in content['orders']:
            box_list.append(box['size_id'])
        return box_list
    except Exception as e:
        print('============== Exception ==============')
        print(path, e)
        sys.exit(-1)


def parse_all_json():
    path = _get_path()
    print('[Obj_Info] Load Training items Info At Path = ' + path)
    folders = glob.glob(join(path, '*'))

    info = dict()
    for folder in folders:
        jsonfiles = glob.glob(join(folder, '*.json'))
        for filepath in jsonfiles:
            obj_info = json_parser_obj(filepath)
            info[obj_info.name] = obj_info
    return info


def get_box_width():
    """Get all of box width."""
    box_width = list()
    for box in order_boxes:
        box_width.append(box_info_dict[box].dimensions[1])
    box_width.sort(reverse=True)
    return box_width


def sort_box_by_width(box_info, boxes):
    for _ in range(len(boxes)):
        for i in range(len(boxes)-1):
            if box_info[boxes[i]].dimensions[1] < box_info[boxes[i+1]].dimensions[1]:
                boxes[i], boxes[i+1] = boxes[i+1], boxes[i]
    return tuple(boxes)


info_dict = parse_all_json()
box_info_dict = json_parser_box(_get_path('box'))

_box_list = json_parser_order(_get_path('order_box'))
order_boxes = sort_box_by_width(box_info_dict, _box_list)
