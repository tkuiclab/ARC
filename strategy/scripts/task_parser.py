#!/usr/bin/env python

"""Get location of item in which bin and place into which box."""
"""for picking challenage."""

#from __future__ import print_function
from os import path
import rospkg
import json

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
            json.dump(content, f, indent=4, separators=(',', ': ')) # sort_keys=True
            # or following, assume content is str obj
            # f.write(json.dumps(content, f, indent=4, separators=(',', ': ')))
        return True
    except IOError as e:
        print(e)
        return False

class PickInfo:
    """Storage information of picking task."""

    def __init__(self, item="", form_bin="", to_box=""):
        """Init object for information of picking task."""
        self.item = item
        self.from_bin = form_bin
        self.to_box = to_box



def make_pick_list(item_loc_path, order_path):
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

    return make_pick_list(item_loc_path, order_path)
     

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
    

# if __name__ == "__main__":
#     _test_pick()
