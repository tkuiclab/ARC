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

import rospkg


Task_type = 'Init'
bin_dict = dict()
orig_pos = [0.4, -0.12, 0]  # coordinate for the whole system(shelf, arm, LM)

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
        if (self.L or self.W or self.H) is not None:
            self.TotalVolume = self.L*self.W*self.H
        else:
            self.TotalVolume = None
        self.NowVolume = 0
        self.ObjectNum = 0
        self.Object = []

        # bound 
        self.min_y = 0
        self.max_y = 0
        self.min_z = 0
        self.max_z = 0
        



def parse_shelf(id = -1):
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

    # ================================ Initialize all bin's bound info ================================
    A = bin_dict[0]
    B = bin_dict[1]
    C = bin_dict[2]
    D = bin_dict[3]
    E = bin_dict[4]
    F = bin_dict[5]
    G = bin_dict[6]
    H = bin_dict[7]
    I = bin_dict[8]
    J = bin_dict[9]

    # ==================== [Bin A] ====================
    A.min_y = orig_pos[1]
    A.max_y = A.min_y + A.W
    A.min_z = orig_pos[2] + (D.H + I.H)
    A.max_z = A.min_z + A.H

    # ==================== [Bin B] ====================
    B.min_y = orig_pos[1] + A.W
    B.max_y = B.min_y + B.W
    B.min_z = orig_pos[2] + (D.H + I.H)
    B.max_z = B.min_z + B.H

    # ==================== [Bin C] ====================
    C.min_y = orig_pos[1] + A.W + B.W
    C.max_y = C.min_y + C.W
    C.min_z = orig_pos[2] + (D.H + I.H)
    C.max_z = C.min_z + C.H

    # ==================== [Bin D] ====================
    D.min_y = orig_pos[1]
    D.max_y = D.min_y + D.W
    D.min_z = orig_pos[2] + I.H
    D.max_z = D.min_z + D.H

    # ==================== [Bin E] ====================
    E.min_y = orig_pos[1] + D.W
    E.max_y = E.min_y + E.W
    E.min_Z = orig_pos[2] + (I.H + G.H)
    E.max_Z = E.min_Z + E.H

    # ==================== [Bin F] ====================
    F.min_y = orig_pos[1] + D.W + E.W
    F.max_y = F.min_y + F.W
    F.min_z = orig_pos[2] + J.H + H.H
    F.max_z = F.min_z + F.H

    # ==================== [Bin G] ====================
    G.min_y = orig_pos[1] + D.W
    G.max_y = G.min_y + G.W
    G.min_z = orig_pos[2] + I.H
    G.max_z = G.min_z + G.H

    # ==================== [Bin H] ====================
    H.min_y = orig_pos[1] + D.W + G.W
    H.max_y = H.min_y + H.W
    H.min_z = orig_pos[2] + J.H
    H.max_z = H.min_z + H.H
    
    # ==================== [Bin I] ====================
    I.min_y = orig_pos[1]
    I.max_y = I.min_y + I.W
    I.min_z = orig_pos[2]
    I.max_z = I.min_z + I.H

    # ==================== [Bin J] ====================
    J.min_y = orig_pos[1] + I.W
    J.max_y = J.min_y + J.W 
    J.min_z = orig_pos[2]
    J.max_z = J.min_z + J.H

    #===================================================
    bin_dict[0] = A
    bin_dict[1] = B
    bin_dict[2] = C
    bin_dict[3] = D
    bin_dict[4] = E
    bin_dict[5] = F
    bin_dict[6] = G
    bin_dict[7] = H
    bin_dict[8] = I
    bin_dict[9] = J

    # =================================================================================================


    if(id >= 0) and (id <= 9):
        return bin_dict[id]
    else:
        return bin_dict

def Distribution(type,mission_obj,fullrate):
    '''type = 任務型態'''
    '''mission_obj = 任務所有的物體'''
    '''fullrate = 介於 0~1，定義一個bin填滿的百分比，例如fullrate = 0.9表示物體塞滿bin 90%的體積後就不能再放東西了，數字愈小代表平均一個bin能放的東西會愈少，但物體會愈平均放置於所有bin'''
    bin_dict = parse_shelf()
    object_belong = []
    #info_dict = parse_all_json()

    # print('info_dict' + str(info_dict))
    #out_dict = dict()
    LimitOfObj = []

    if type == 'pick':
        limit = 4
    else :
        limit = 2


    for i in range(len(['A','B','C','D','E','F','G','H','I','J'])):
        LimitOfObj.append(limit) 

    if type == 'obj too big!':
        LimitOfObj[3] = 4
        LimitOfObj[9] = 4
 
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
                if bin_dict[j].ObjectNum >= LimitOfObj[j]:
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
    output , bin_content = Distribution('stow',stow_content,0.01)
    i = 0
    while len(output)<len(stow_content):
        i = i + 0.01
        output, bin_content = Distribution('stow', stow_content, i)
        #print (i)

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

if __name__ == '__main__':
    main()
