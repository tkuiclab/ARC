"""Config position of linear mobile."""

import get_obj_info as box_info


def GetShift(Target_Type, LM_Dir, Target):
    """ Get the motion pulse of linear motor """
    # Bin
    if Target_Type == 'Bin':
        if Target in BinId:
            if LM_Dir == 'x':
                return BinShift_X[BinId.index(Target)]
            elif LM_Dir=='z':
                return BinShift_Z[BinId.index(Target)]
            else:
                print 'Error input Bin dir'
                return 0
        else:
            print 'Error input character'

    # Tote
    elif  Target_Type == 'Tote':
        if Target in ToteId:
            if LM_Dir == 'x':
                return ToteShift_X[ToteId.index(Target)]
            elif LM_Dir=='z':
                return ToteShift_Z[ToteId.index(Target)]
            else:
                print 'Error input Tote dir'
                return 0
        else:
            print 'Error input character'

    # Box
    elif  Target_Type == 'Box':
        if Target in BoxId:
            if LM_Dir == 'x':
                return BoxShift_X[BoxId.index(Target)]
            elif LM_Dir=='z':
                return BoxShift_Z[BoxId.index(Target)]
            else:
                print 'Error input dir'
                return 0
        else:
            print 'Error input Box character'
    
    else:
        print 'Error input Target Type'


def cal_box_x_shift():
    box_width = box_info.get_box_width()

    # Convert function
    m2mm = lambda m: m * 1000
    mm2pulse = lambda mm: mm * 100

    # Calculate pulse of x shift for order box size
    pulse = [60000]
    for i in range(len(box_info.order_boxes) - 1):
        pulse.append(pulse[i] - mm2pulse(m2mm(box_width[i]) / 2.0 + m2mm(box_width[i+1]) / 2.0))

    return tuple(pulse)


# Initialize Bin's Information
# BinId 	   		= [   'a',   'b',   'c',   'd',   'e',   'f',   'g',   'h',   'i',   'j',   'k',   'l' ,   'z']
# BinShift_Z 	    = [     0,     0,     0, 27000, 27000, 27000, 50000, 52000, 50000, 75000, 75000, 75000 , 80000]
# BinShift_X 	    = [ 60000, 35000,     0, 60000, 35000,     0, 60000, 35000,     0, 60000, 32500,     0 , 59000]

# New Initialize Bin's Information
BinId       = [   'a',   'b',   'c',   'd',   'e',   'f',   'g',   'h',   'i',   'j',   'k',   'l' ,   'z']
BinShift_X  = [ 39000, 16000,     0, 39000, 17000,  2000, 17000,  2000, 43000, 15000, 32500,     0 , 59000]
BinShift_Z  = [ 17000, 17000, 17000, 47000, 36000, 36000, 56000, 56000, 77000, 77000, 75000, 75000 , 80000]

Bin         = 'a'
BinCnt      = 0

# === Initialize Box's Information ===
BoxId 			= [  'a',   'b',   'c']
# BoxShift_X	= [59000, 32000,  1000]
BoxShift_X		= cal_box_x_shift()
BoxShift_Z		= [79000, 79000, 79000]

#=== Initialize Tote's Information ===
ToteId 		= [  'a',   'b']
#ToteShift_X	= [49000, 11000 ]
ToteShift_X	= [60000, 11000]
ToteShift_Z	= [76000, 76000]

ToteLeave_Z = 50000     # pick (suck)  finish


Box_A1 = 'A1'
Box_1AD = '1AD'
Box_1A5 = '1A5'
Box_1B2 = '1B2'
Box_K3 = 'K3'

print(BoxShift_X)
