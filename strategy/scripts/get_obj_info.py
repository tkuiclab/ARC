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


def _get_info_path():
    pkg_path = rospkg.RosPack().get_path('arc')
    return join(pkg_path, 'Training items')


def json_parser(path):
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


def parse_all_json():
    path = _get_info_path()
    print('[Obj_Info] Load Training items Info At Path = ' + path)
    folders = glob.glob(join(path, '*'))

    info = dict()
    for folder in folders:
        jsonfiles = glob.glob(join(folder, '*.json'))
        for filepath in jsonfiles:
            obj_info = json_parser(filepath)
            info[obj_info.name] = obj_info
    return info


info_dict = parse_all_json()
