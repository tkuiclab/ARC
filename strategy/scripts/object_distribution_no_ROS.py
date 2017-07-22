#!/usr/bin/env python
# coding=UTF-8
"""Get information of item in json. and distribution to bin"""

from __future__ import print_function
from os.path import join
import sys
import json
import glob

Task_type = 'Init'

class ObjInfo:
    """Information of objects were defined."""

    def __init__(self, **kwargs):
        """Init object for information of objects."""
        self.name = kwargs.get('name')
        self.dimensions = kwargs.get('dims')
        self.weight = kwargs.get('weight')
        self.type = kwargs.get('type')

class BinInfo:
    """Information of bin."""
    def __init__(self, **kwargs):
        """Init object for information of shelf."""
        self.block = kwargs.get('block')
        #寬
        self.L = kwargs.get('L')
        #深
        self.W = kwargs.get('W')
        #高
        self.H = kwargs.get('H')
        self.TotalVolume = self.L*self.W*self.H
        self.NowVolume = 0
        self.ObjectNum = 0
        self.Object = []

def _get_info_path():
    pkg_path = "./"
    return join(pkg_path, 'Training items')

def read_json(path):
    """Read a JSON file from path, and convert to object of python."""
    try:
        with open(path) as f:
            content = json.load(f)
            # or following
            # content = json.loads(f.read())
            return content
    except IOError as e:
        print(e)
        return None

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

def stow_task_json_parser(path):
    try:
        content = read_json(path)
        return content
    except Exception as e:
        print('============== Exception ==============')
        print(path, e)
        sys.exit(-1)

def bin_json_parser(path):
    try:
        content = read_json(path)
        return content
    except Exception as e:
        print('============== Exception ==============')
        print(path, e)
        sys.exit(-1)

def parse_all_json():
    path = _get_info_path()
    folders = glob.glob(join(path, '*'))
    info = dict()
    for folder in folders:
        jsonfiles = glob.glob(join(folder, '*.json'))
        # print(jsonfiles)
        for filepath in jsonfiles:
            obj_info = json_parser(filepath)

            info[obj_info.name] = obj_info

    return info

def parse_shelf():

    # bin_spec_path = rospkg.RosPack().get_path('arc') + '/bin_spec.json'
    # print('bin_spec_path = ' + bin_spec_path)
    # content = bin_json_parser(bin_spec_path)
    bin_dict = dict()
    content = bin_json_parser("./bin_spec.json")

    bin_dict[0] = BinInfo(block='A', L=content['bins'][0]['dimensions'][0], W=content['bins'][0]['dimensions'][1], H=content['bins'][0]['dimensions'][2])
    bin_dict[1] = BinInfo(block='B', L=content['bins'][1]['dimensions'][0], W=content['bins'][1]['dimensions'][1], H=content['bins'][1]['dimensions'][2])
    bin_dict[2] = BinInfo(block='C', L=content['bins'][2]['dimensions'][0], W=content['bins'][2]['dimensions'][1], H=content['bins'][2]['dimensions'][2])
    bin_dict[3] = BinInfo(block='D', L=content['bins'][3]['dimensions'][0], W=content['bins'][3]['dimensions'][1], H=content['bins'][3]['dimensions'][2])
    bin_dict[4] = BinInfo(block='E', L=content['bins'][4]['dimensions'][0], W=content['bins'][4]['dimensions'][1], H=content['bins'][4]['dimensions'][2])
    bin_dict[5] = BinInfo(block='F', L=content['bins'][5]['dimensions'][0], W=content['bins'][5]['dimensions'][1], H=content['bins'][5]['dimensions'][2])
    bin_dict[6] = BinInfo(block='G', L=content['bins'][6]['dimensions'][0], W=content['bins'][6]['dimensions'][1], H=content['bins'][6]['dimensions'][2])
    bin_dict[7] = BinInfo(block='H', L=content['bins'][7]['dimensions'][0], W=content['bins'][7]['dimensions'][1], H=content['bins'][7]['dimensions'][2])
    bin_dict[8] = BinInfo(block='I', L=content['bins'][8]['dimensions'][0], W=content['bins'][8]['dimensions'][1], H=content['bins'][8]['dimensions'][2])
    bin_dict[9] = BinInfo(block='J', L=content['bins'][9]['dimensions'][0], W=content['bins'][9]['dimensions'][1], H=content['bins'][9]['dimensions'][2])

    return bin_dict

def Distribution(type,mission_obj,fullrate):
    '''type = 任務型態'''
    '''mission_obj = 任務所有的物體'''
    '''fullrate = 介於 0~1，定義一個bin填滿的百分比，例如fullrate = 0.9表示物體塞滿bin 90%的體積後就不能再放東西了，數字愈小代表平均一個bin能放的東西會愈少，但物體會愈平均放置於所有bin'''
    bin_dict = parse_shelf()
    object_belong = []
    info_dict = parse_all_json()

    # print('info_dict' + str(info_dict))
    out_dict = dict()

    if type == 'pick':
        LimitOfObj = 4
    else:
        LimitOfObj = 2

    #print(info_dict[mission_obj[4]].dimensions[0],info_dict[mission_obj[4]].dimensions[1],info_dict[mission_obj[4]].dimensions[2])
    for i in range(len(mission_obj)):
        for j in range(10):

            #排序物體和bin的三圍
            SortMissionObj = sorted([info_dict[mission_obj[i]].dimensions[0],info_dict[mission_obj[i]].dimensions[1],info_dict[mission_obj[i]].dimensions[2]])
            SortBinSize = sorted([bin_dict[j].L,bin_dict[j].W,bin_dict[j].H])

            #如果物體三維依大小順序都超出bin的三維，則換至下一個bin
            if (SortMissionObj[0]>SortBinSize[0] or SortMissionObj[1]>SortBinSize[1] or SortMissionObj[2]>SortBinSize[2]):
                # print(mission_obj[i],' over of ' ,bin_dict[j].block ,'dimensions')
                continue
            else:
                # 如果超出一個bin可容納的最大物體數，則換下一個bin
                if bin_dict[j].ObjectNum >= LimitOfObj:
                    # print('bin ',bin_dict[j].block,' full')
                    continue
                else:
                    # 計算bin剩下的體積
                    bin_dict[j].NowVolume = bin_dict[j].NowVolume + (info_dict[mission_obj[i]].dimensions[0] * info_dict[mission_obj[i]].dimensions[1] * info_dict[mission_obj[i]].dimensions[2])
                    # 如果bin已經容納不下，則換下一個bin
                    if bin_dict[j].NowVolume > bin_dict[j].TotalVolume*fullrate:
                        bin_dict[j].NowVolume = bin_dict[j].NowVolume - (info_dict[mission_obj[i]].dimensions[0] * info_dict[mission_obj[i]].dimensions[1] * info_dict[mission_obj[i]].dimensions[2])
                        # print('out of limit when ',mission_obj[i],'in ',bin_dict[j].block)
                        continue
                    else:
                        #計算bin內的物體數
                        bin_dict[j].ObjectNum = bin_dict[j].ObjectNum + 1
                        object_belong.append(bin_dict[j].block)
                        bin_dict[j].Object.append(mission_obj[i])
                        # print(mission_obj[i],' in bin',bin_dict[j].block)
                        # print(bin_dict[j].block, 'rest volume ', bin_dict[j].TotalVolume - bin_dict[j].NowVolume)
                        break

    out_dict = zip(object_belong,mission_obj)
    #print (out_dict)
    return out_dict,bin_dict


def main():
    stow_task_content = stow_task_json_parser('./stow_task/item_location_file.json')

    mission_object = []
    for i in range(len(stow_task_content['tote']['contents'])):
        mission_object.append(str(stow_task_content['tote']['contents'][i]))

    output , bin_content = Distribution('pick',mission_object,0.01)
    i = 0
    while len(output)<len(stow_task_content['tote']['contents']):
        i = i + 0.01
        output, bin_content = Distribution('stow', mission_object, i)
        print (i)

    print('===============output===============')
    print(output)
    print('===============Object in bin===============')
    for i in range(10):
        print('bin ',bin_content[i].block,' ',bin_content[i].Object)

if __name__ == '__main__':
    main()
