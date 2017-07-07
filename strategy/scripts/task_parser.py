#!/usr/bin/env python

"""Get location of item in which bin and place into which box."""
"""for picking challenage."""

#from __future__ import print_function
from os import path
import rospkg
import json

class PickInfo:
    """ Information of picking task."""

    def __init__(self, item="", form_bin="", to_box=""):
        """Init object for information of picking task."""
        self.item = item
        self.from_bin = form_bin
        self.to_box = to_box

class StowInfo:
    """ Information of stowing task."""

    def __init__(self, item="", to_bin=""):
        """Init object for information of stowing task."""
        self.item = item
        self.to_bin = to_bin
        self.to_other_tote = False
        self.success = False

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

def write_json(path, content):
    """Write object of python to specify JSON file using JSON format."""
    try:
        with open(path, 'w') as f:
            # json.dump(content, f, indent=4, separators=(',', ': '), skipkeys=True) # sort_keys=True
            # or following, assume content is str obj
            f.write(json.dumps(content, f, indent=4, separators=(',', ': ')))
        return True
    except IOError as e:
        print(e)
        return False


def make_pick_list_from_path(item_loc_path, order_path):
    """Using item location file and order file to make a list for picking task."""
    item_loc_json = read_json(item_loc_path)
    order_json = read_json(order_path)

    pick_list = list()
    for order in order_json["orders"]:
        box_id = order["size_id"]
        for item in order["contents"]:
            bin_id = search_item(item_loc_json, item)
            if bin_id is not None:
                pick_list.append(PickInfo(item, bin_id, box_id))
    return pick_list

def make_pick_list(i_item_loc_json, i_order_json):
    """Using item location file and order file to make a list for picking task."""

    item_loc_json = json.loads(i_item_loc_json)
    order_json = json.loads(i_order_json)

    pick_list = list()
    for order in order_json["orders"]:
        box_id = order["size_id"]
        for item in order["contents"]:
            bin_id = search_item(item_loc_json, item)
            if bin_id is not None:
                pick_list.append(PickInfo(item, bin_id, box_id))
    return pick_list

def make_stow_list(i_item_loc_json):
    """Using item location file to make a list for stowing task."""

    item_loc_json = json.loads(i_item_loc_json)
    
    #print 'item_loc_json["tote"]["contents"] = ' + str(item_loc_json["tote"]["contents"])  

    stow_list = list()
    bin_id = 'e'
    for item in item_loc_json["tote"]["contents"]:
        stow_list.append(StowInfo(item, bin_id))
        #bin_id = bin_id + 1
        #bin_id = chr(ord(bin_id)+1)

    show_stow_list(stow_list)

    return stow_list

def show_stow_list(i_stow_list):
    for stow in i_stow_list:
        if stow.to_other_tote:
            print stow.item + '\t\t-> Unknown Tote' + ' (' + str(stow.success) + ')'
        else:
            print stow.item + '\t\t-> ' + stow.to_bin + ' (' + str(stow.success) + ')'
        

def get_json(json_str):
    return json.loads(json_str)

def search_item(item_loc_json, target):
    """Searching target item in which bin."""
    for bin in item_loc_json["bins"]:
        for item in bin["contents"]:
            if item == target:
                return bin["bin_id"]

    print '[task_parser.py] Say Cannot find '+ target
    return None

def read_config_pick_task():
    direcotry = rospkg.RosPack().get_path('config') + '/pick_task/'
    ilf = "item_location_file.json"
    orf = "order_file.json"

    item_loc_path = path.join(direcotry, ilf)
    order_path = path.join(direcotry, orf)

    return make_pick_list_from_path(item_loc_path, order_path)
     

def _test_pick():
    """Testing function."""

    pick_list = read_config_pick_task()
    

     #----------Test 1 -----------#
    # info = pick_list[0]
    # print("item:", info.item, "from_bin:", info.from_bin, "to_box:", info.to_box)

    # info = pick_list[1]
    # print("item:", info.item, "from_bin:", info.from_bin, "to_box:", info.to_box)

    # print 'pick_list.length = ' + str(len(pick_list))

    #----------Test 2 -----------#
    pick_list = read_config_pick_task()
    for info in pick_list:
        print("item:", info.item, "from_bin:", info.from_bin, "to_box:", info.to_box)

def read_init_pick_ilf(direcotry, container):
    # direcotry = rospkg.RosPack().get_path('config') + '/pick_task/base_item_location_file.json'
    item_loc_json = read_json(direcotry)
    pick_list = list()
    tmp_dict = dict()
    if container == "tote":
        print 'item = ' + str(item_loc_json[container]) #["contents"]
        tmp_dict = item_loc_json[container]
        
        pick_list.append(tmp_dict)
        print '\nbefore append ===>' 
        print  tmp_dict
        # tmp_ContList = list()
        # tmp_ContList = tmp_dict
        # for item in item_loc_json[container]:
        #     print "item = " + str(item)
        #     print "tmp_ContList = " + str(tmp_ContList)
        #     pick_list.append(tmp_ContList)

    elif container == "bins" or container == "boxes":
        for item in item_loc_json[container]:
            pick_list.append(item)
    else:
        print 'error input'
    print '\nafter append ===>\n' + str(pick_list)
    return pick_list

def Insert_Item_2_ContList(ContType, ContList, No, DesireItem):
    if ContType == "Bin" or ContType == "bin" :
        print 'bin'
        for item in ContList:
            if item["bin_id"] == No:
                item["contents"].append(DesireItem)
                break
    elif ContType == "Box" or ContType == "box":
        print 'box'
        for item in ContList:
            if item["size_id"] == No:
                item["contents"].append(DesireItem)
                break
    elif ContType == "Tote" or ContType == "tote":
        print 'tote'
        tmp_ContList = list()
        tmp_ContList = ContList
        for item in ContList:
            item["contents"].append(DesireItem)

    else:
        print "error Cont type"
    return ContList

def Delete_Item_from_ContList(ContType, ContList, No, DesireItem):
    if ContType == "Bin" or ContType == "bin" :
        print 'bin'
        for item in ContList:
            if item["bin_id"] == No:
                item["contents"].remove(DesireItem)
                break
    elif ContType == "Box" or ContType == "box":
        print 'box'
        for item in ContList:
            if item["size_id"] == No:
                item["contents"].remove(DesireItem)
                break

    elif ContType == "Tote" or ContType == "tote":
        print 'tote'
        tmp_ContList = list()
        tmp_ContList = ContList
        for item in ContList:
            item["contents"].remove(DesireItem)

    else:
        print "error Cont type"
    return ContList

def Convert_PickTaskList_to_JSON():
    print '[write json] write_PickInfo_2_JSON'
    base_pick_path = rospkg.RosPack().get_path('config') + '/pick_task/base_item_location_file.json'
    # Call the following fn when start pick task strategy
    Tote_List = read_init_pick_ilf(base_pick_path, "tote")
    Box_List  = read_init_pick_ilf(base_pick_path, "boxes")
    Bin_List  = read_init_pick_ilf(base_pick_path, "bins")

    # call the following fn when pick one item from bin and to box

    #before pick
    Bin_List = Insert_Item_2_ContList("bin", Bin_List, "A", "Desire_Item1")
    Bin_List = Insert_Item_2_ContList("bin", Bin_List, "A", "Desire_Item2")

    #after pick
    Bin_List  = Delete_Item_from_ContList("bin", Bin_List, "A", "Desire_Item1")
    Box_List = Insert_Item_2_ContList("box", Box_List, "1A5", "Desire_Item1")

    # call the following fn when complete task and want to save item_location_file
    pick_direcotry = rospkg.RosPack().get_path('config') + '/pick_task/save_json_test.json'
    pick_list = { "bins":Bin_List, "boxes":Box_List, "tote":Tote_List}
    write_json(pick_direcotry, pick_list)   

def Convert_StowTaskList_to_JSON():
    print '[write json] write_StowInfo_2_JSON'
    base_pick_path = rospkg.RosPack().get_path('config') + '/stow_task/base_item_location_file.json'
    # Call the following fn when start stow task strategy
    Bin_List  = read_init_pick_ilf(base_pick_path, "bins")
    Box_List  = read_init_pick_ilf(base_pick_path, "boxes")
    Tote_List = read_init_pick_ilf(base_pick_path, "tote")

    # call the following fn when stow one item from bin and to tote

    #before stow
    Tote_List = Insert_Item_2_ContList("tote", Tote_List, "", "Desire_Item1")
    Tote_List = Insert_Item_2_ContList("tote", Tote_List, "", "Desire_Item2")

    #after stow
    Tote_List = Delete_Item_from_ContList("tote", Tote_List, "", "Desire_Item1")
    Bin_List  = Insert_Item_2_ContList("bin", Bin_List, "C", "Desire_Item1")

    # call the following fn when complete task and want to save item_location_file
    pick_direcotry = rospkg.RosPack().get_path('config') + '/stow_task/save_json_test.json'
    pick_list = { "bins":Bin_List, "boxes":Box_List, "tote":Tote_List}
    write_json(pick_direcotry, pick_list) 


def write_PickInfo_2_JSON(): # jmp save json
    Convert_PickTaskList_to_JSON()
    Convert_StowTaskList_to_JSON()

if __name__ == "__main__":
    info = {'Name': 'Zara', 'Age': 7, 'Class': 'First'}
    print json.dumps(info)
    #_test_pick()
