#!/usr/bin/env python
# coding=UTF-8
"""Get information of item in json. and distribution to bin"""

from __future__ import print_function
from os.path import join
import sys
import json
import glob
from  get_obj_info import *
#from task_parser import read_json
import sys
import rospkg


Task_type = 'Init'
bin_dict = dict()


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



def parse_shelf():
    bin_spec_path = rospkg.RosPack().get_path('arc') + '/bin_spec.json'
    
    #content = bin_json_parser("/home/luca/PycharmProjects/ARC_Object_Distribution/bin_spec.json")
    content = read_json(bin_spec_path)
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


def Word2Num(word):
    if word == 'A':
        return 0
    if word == 'B':
        return 1
    if word == 'C':
        return 2
    if word == 'D':
        return 3
    if word == 'E':
        return 4
    if word == 'F':
        return 5
    if word == 'G':
        return 6
    if word == 'H':
        return 7
    if word == 'I':
        return 8
    if word == 'J':
        return 9

allobj_dict = parse_all_json()

def DistributionV2(type, mission_obj, fullrate, out_of_bin):
    binsize_dict = parse_shelf()
    #allobj_dict = parse_all_json()
    sortedbinValue = []
    sortedbinName = []

    sortedmissionobjValue = []
    sortedmissionobjName = []

    object_belong = []

    LimitOfObj = []

    limit = 2
    if type == 'pick':
        limit = 4


    LimitOfObj = dict()

    #最後排序bin大小後的順序
    BinOrder = []
    MissionObjOrder = []

# bin大小排序
######################################################
    for i in range(len(binsize_dict)):
        sortedbinValue.append(binsize_dict[i].TotalVolume)
        sortedbinName.append(binsize_dict[i].block)

    sortedBinSize = sorted(zip(sortedbinValue, sortedbinName))

    for i in range(len(sortedBinSize)):
        BinOrder.append(sortedBinSize[i][1])
######################################################

#任務物體大小排序
######################################################
    for i in range(len(mission_obj)):
        sortedmissionobjValue.append(allobj_dict[mission_obj[i]].dimensions[0]*allobj_dict[mission_obj[i]].dimensions[1]*allobj_dict[mission_obj[i]].dimensions[2])
        sortedmissionobjName.append(mission_obj[i])

    sortedMissionObjSize = sorted(zip(sortedmissionobjValue, sortedmissionobjName))

    for i in range(len(sortedMissionObjSize)):
        MissionObjOrder.append(sortedMissionObjSize[i][1])

    # print (MissionObjOrder)
######################################################

    for i in range(len(BinOrder)):
        LimitOfObj[BinOrder[i]] = limit

    if out_of_bin == 1:
        LimitOfObj['D'] = 4
        LimitOfObj['J'] = 4

    for i in MissionObjOrder:
        for j in BinOrder:
            #排序物體和bin的三圍
            SortMissionObj = sorted([allobj_dict[i].dimensions[0], allobj_dict[i].dimensions[1], allobj_dict[i].dimensions[2]])
            SortBinSize = sorted([binsize_dict[Word2Num(j)].L,binsize_dict[Word2Num(j)].W,binsize_dict[Word2Num(j)].H])
            # 如果物體三維依大小順序都超出bin的三維，則換至下一個bin
            if (SortMissionObj[0] > SortBinSize[0] or SortMissionObj[1] > SortBinSize[1] or SortMissionObj[2] >SortBinSize[2]):
                # print(mission_obj[i],' over of ' ,bin_dict[j].block ,'dimensions')
                continue
            else:
                if binsize_dict[Word2Num(j)].ObjectNum >= LimitOfObj[j]:
                    continue
                else:
                    binsize_dict[Word2Num(j)].NowVolume = binsize_dict[Word2Num(j)].NowVolume + (allobj_dict[i].dimensions[0] * allobj_dict[i].dimensions[1] *allobj_dict[i].dimensions[2])
                    if binsize_dict[Word2Num(j)].NowVolume > binsize_dict[Word2Num(j)].TotalVolume * fullrate:
                        binsize_dict[Word2Num(j)].NowVolume = binsize_dict[Word2Num(j)].NowVolume - (allobj_dict[i].dimensions[0] * allobj_dict[i].dimensions[1] *allobj_dict[i].dimensions[2])
                        # print('out of limit when ',mission_obj[i],'in ',bin_dict[j].block)
                        continue
                    else:
                        # 計算bin內的物體數
                        binsize_dict[Word2Num(j)].ObjectNum = binsize_dict[Word2Num(j)].ObjectNum + 1
                        object_belong.append(binsize_dict[Word2Num(j)].block)
                        binsize_dict[Word2Num(j)].Object.append(i)
                        # print(binsize_dict[Word2Num(j)].block,' in bin',binsize_dict[Word2Num(j)].block)
                        # print(binsize_dict[Word2Num(j)].block, 'rest volume ', binsize_dict[Word2Num(j)].TotalVolume - binsize_dict[Word2Num(j)].NowVolume)
                        break

    out_dict = zip(object_belong, mission_obj)
    # print (out_dict)
    return out_dict, binsize_dict

def find_obj_in_bin_content(bin_content, want_obj):
    for i in range(10):
        print('-----' + bin_content[i].block + '------')

        for obj in bin_content[i].Object:
            print(obj)
        
            if obj == want_bj:
                return bin_content[i].block

def find_obj_in_distribution(bin_distri, want_obj):
    for item_bin in bin_distri:
        if item_bin[1] == want_obj:
            return item_bin[0]

def stow_distribution(stow_content):
    output, bin_content = DistributionV2('stow',stow_content,0.01, 0)
    i = 0
    while len(output) < len(stow_content):
        i = i + 0.01
        if i <= 1:
            output, bin_content = DistributionV2('stow', stow_content, i, 0)
        else:
            output, bin_content = DistributionV2('stow', stow_content, i, 1)
            print ('Out of bin')
            break
        sys.stdout.write("Use Volume: %d%%   \r" % (i*100) )
        sys.stdout.flush()

    # while len(output)<len(stow_content):
    #     i = i + 0.01
    #     if i <= 1 :
    #         output, bin_content = DistributionV2('stow', stow_content, i)
            
    #     else:
    #         output, bin_content = DistributionV2('obj_too_big', stow_content, i)
    #         break
    #     sys.stdout.write("Use Volume: %d%%   \r" % (i*100) )
    #     sys.stdout.flush()

    print('[object_distribution] Use ' + str(i*100) + '% can put all')


    return  output , bin_content

def show_bin_content(bin_content):
    for i in range(10):
        print('bin ',bin_content[i].block,' ',bin_content[i].Object)

def main():
    item_location_file_path = rospkg.RosPack().get_path('arc') + '/stow_task/stow_20.json'


    print("item_location_file_path =" + item_location_file_path)
    item_loc_json = read_json(item_location_file_path)

    mission_object = item_loc_json['tote']['contents']
    # for i in range(len(item_loc_json['tote']['contents'])):
    #     mission_object.append(str(item_loc_json['tote']['contents'][i]))


    output, bin_content = DistributionV2('stow',mission_object,0.01, 0)
    i = 0
    while len(output) < len(mission_object):
        i = i + 0.01
        if i <= 1:
            output, bin_content = DistributionV2('stow', mission_object, i, 0)
        else:
            output, bin_content = DistributionV2('stow', mission_object, i, 1)
            print ('Out of bin')
            break

        print (i)

    print('===============output===============')
    print(output)
    print('===============Object in bin===============')
    for i in range(10):
        print('bin ',bin_content[i].block,' ',bin_content[i].Object)
    '''
    output , bin_content = Distribution('pick',mission_object,0.01)
    i = 0
    while len(output)<len(item_loc_json['tote']['contents']):
        i = i + 0.01
        if i > 1 :
            output, bin_content = Distribution('obj too big!', mission_object, i)
            break
        else:
            output, bin_content = Distribution('stow', mission_object, i)
        show_bin_content(bin_content)
        print(i)
        
    # print('===============output===============')
    # print(output)
    print('===============Object in bin===============')
    show_bin_content(bin_content)

    # print("marbles -> " + find_obj_in_distribution(output, "marbles") )
    '''
if __name__ == '__main__':
    main()
