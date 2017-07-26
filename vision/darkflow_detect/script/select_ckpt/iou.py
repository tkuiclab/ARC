import os
from os.path import join
import sys
import json
import glob

from collections import namedtuple
import xml.etree.ElementTree as ET
import numpy as np


all_obj_name_ARC = [
    "avery_binder",
    "balloons",
    "band_aid_tape",
    "bath_sponge",
    "black_fashion_gloves",
    "burts_bees_baby_wipes",
    "colgate_toothbrush_4pk",
    "composition_book",
    "crayons",
    "duct_tape",
    "epsom_salts",
    "expo_eraser",
    "fiskars_scissors",
    "flashlight",
    "glue_sticks",
    "hand_weight",
    "hanes_socks",
    "hinged_ruled_index_cards",
    "ice_cube_tray",
    "irish_spring_soap",
    "laugh_out_loud_jokes",
    "marbles",
    "measuring_spoons",
    "mesh_cup",
    "mouse_traps",
    "pie_plates",
    "plastic_wine_glass",
    "poland_spring_water",
    "reynolds_wrap",
    "robots_dvd",
    "robots_everywhere",
    "scotch_sponges",
    "speed_stick",
    "table_cloth",
    "tennis_ball_container",
    "ticonderoga_pencils",
    "tissue_box",
    "toilet_brush",
    "white_facecloth",
    "windex"
]

all_obj_name = [
    "avery1BinderWhite",
    "bagOfBalloons",
    "johnsonjohnsonPaperTape",
    "theBatheryDelicateBathSponge",
    "knitGlovesBlack",
    "burtsBeesBabyWipes",
    "colgateToothbrushs",
    "greenCompositionBook",
    "crayolaCrayons24",
    "scotchClothDuctTape",
    "drtealsEpsomSalts",
    "expoEraser",
    "fiskarScissors",
    "arFlashlihgts",
    "elmersGlueSticks6Ct",
    "neopreneWeightPink",
    "hanesWhitteSocks",
    "spiralIndexCards",
    "steriliteIceCubeTray",
    "irishSpring",
    "laughOutLoundJokesForKids",
    "miniMarblesClearLustre",
    "targetBrandMeasuringSpoons",
    "meshPencilCup",
    "tomcatMousetraps",
    "reynoldsPiePans2ct",
    "plasticWineGlasses",
    "polandSpringsWaterBottle",
    "reynoldsWrap85Sqft",
    "dvdRobots",
    "robotsEverywhere",
    "scotchSponges",
    "speedStick2Pack",
    "tableCover",
    "wilsonTennisBalls",
    "ticonderogaPencils",
    "kleenexCoolTouchTissues",
    "cloroxToiletBrush",
    "whiteFaceCloth",
    "windexSprayBottle23oz"
]


# define the `Detection` object
Detection = namedtuple("Detection", ["image_path", "gt", "pred"])
DataList = []
IOU_Path = './IOU'


def trans(obj):
    index = all_obj_name.index(obj)
    return all_obj_name_ARC[index]


def ComputeIOU(boxA, boxB):
    # determine the (x, y)-coordinates of the intersection rectangle
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    # compute the area of intersection rectangle
    interArea = (xB - xA + 1) * (yB - yA + 1)

    # compute the area of both the prediction and ground-truth
    # rectangles
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = interArea / float(boxAArea + boxBArea - interArea)
    if iou < 0:
        iou = 0
    #return the intersection over union value
    return iou


def FileGroundTruth(FileName):
    list_objname = []
    list_bbox = []
    tree = ET.parse(FileName)
    root = tree.getroot()

    for obj in root.findall('object'):
        objname = obj.find('name').text
        list_objname.append(objname)

    for i in range(len(root)-6):
        for bbox in root[6+i].findall('bndbox'):
            xmin = bbox.find('xmin').text
            ymin = bbox.find('ymin').text
            xmax = bbox.find('xmax').text
            ymax = bbox.find('ymax').text
            objbbox = [int(xmin),int(ymin),int(xmax),int(ymax)]
            list_bbox.append(objbbox)
    # print(len(list_bbox))
    return list_objname,list_bbox


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


def predbox_json_parser(path):
    try:
        content = read_json(path)
        return content
    except Exception as e:
        print('============== Exception ==============')
        print(path, e)
        # sys.exit(-1)


def ComputeFileIOU(FileName,PredBox):
    gtname,gtbox = FileGroundTruth(FileName)

    dict_iou = dict()

    for j in range(len(PredBox)):
        if PredBox[j]['label'] not in gtname:
            dict_iou[str(PredBox[j]['label'])] = 0

    for i in range(len(gtname)):
        dict_iou[str(gtname[i])] = 0
        pred_obj_num = 0
        for j in range(len(PredBox)):
            if gtname[i] == PredBox[j]['label']:
                pred_obj_num = pred_obj_num + 1
                dict_iou[str(gtname[i])] = (dict_iou[str(gtname[i])] + ComputeIOU(gtbox[i], [PredBox[j]['topleft']['x'], PredBox[j]['topleft']['y'], PredBox[j]['bottomright']['x'], PredBox[j]['bottomright']['y']]))/(pred_obj_num)

    return dict_iou


def Write_IOU_File(ckptName,obj_iou):

    f = open(IOU_Path+'/'+ ckptName.split('.')[0], 'w')
    for i in range(len(all_obj_name)):
       f.write(all_obj_name[i])
       f.write(' ')
       f.write(str(obj_iou[all_obj_name[i]]))
       f.write('\n')
    f.close()


def Read_IOU_File(iouFile):

    f = open(IOU_Path + '/' + iouFile, 'r')
    iou_dict = dict()
    #read every line
    for line in f:
        key = trans(line.split(' ')[0])
        value = float(line.split(' ')[1])
        iou_dict[key] = value
    f.close()
    return iou_dict


def selectCKPT(tasktype,task):
    iou_Total = dict()
    stow_task_content = read_json(task)
    mission_object = []
    try:
        if tasktype == 'pick':
            for i in range(10):
                for j in range(len(stow_task_content['bins'][i]['contents'])):
                    mission_object.append(str(stow_task_content['bins'][i]['contents'][j]))

        elif tasktype == 'stow':
            for i in range(len(stow_task_content['tote'])):
                mission_object.append(str(stow_task_content['tote']["contents"][i]))
        else:
            for i in range(len(stow_task_content['tote'])):
                mission_object.append(str(stow_task_content['tote']["contents"][i]))

        for i in os.listdir(IOU_Path):
            score = 0
            for j in mission_object:
                score = Read_IOU_File(i)[j] + score
            iou_Total[score] = i

        return str(iou_Total[sorted(iou_Total)[-1]])+'.ckpt'
    except Exception as e:
        return "yolo-new-288000"


def main():

    total_iou_dict = dict()
    obj_num_dict = dict()
    total_iou = []
    obj_num = []

    for i in range(40):
        total_iou.append(0)
        obj_num_dict[all_obj_name[i]] = 0
        total_iou_dict[all_obj_name[i]] = 0

    print (len(total_iou_dict))
    gt_path = './groundtruth_xml/'
    pred_path = './predict_json/titanx2_ckpt/0724/'
    # print (sorted(os.listdir(gt_path)))
    # print (sorted(os.listdir(pred_path)))
    sorted(os.listdir(pred_path))
    #Compute all iou in the floder
    for i in os.listdir(pred_path):
        #predict form darkflow
        predbox = predbox_json_parser(pred_path+i)
        # print predbox

        i = i.split('.')[0] + '.xml'
        iou = ComputeFileIOU(gt_path+i, predbox)

        for j in range(len(iou)):
            if iou.values()[j] != 0:
                obj_num_dict[iou.keys()[j]] = obj_num_dict[iou.keys()[j]] + 1

            if obj_num_dict[iou.keys()[j]] == 0:
                total_iou_dict[iou.keys()[j]] = (total_iou_dict[iou.keys()[j]] + iou.values()[j])
            else:
                total_iou_dict[iou.keys()[j]] = (total_iou_dict[iou.keys()[j]] + iou.values()[j])/obj_num_dict[iou.keys()[j]]

    Write_IOU_File('titanx2_ckpt_0724', total_iou_dict)

    # Read_IOU_File()
    #Output of IOU
    # ioutotal = selectCKPT('./pick_task/pick_40.json')
    # print (ioutotal)

if __name__=='__main__':
    # main()
    ioutotal = selectCKPT("pick",'./pick_40.json')
    print (ioutotal)