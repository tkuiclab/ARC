# Initialize Bin's Information
BinId 	   		= [   'a',   'b',   'c',   'd',   'e',   'f',   'g',   'h',   'i',   'j',   'k',   'l' ,   'z']
BinShift_Z 	    = [     0,     0,     0, 25000, 25000, 25000, 50000, 50000, 50000, 75000, 75000, 75000 , 80000]
BinShift_X 	    = [ 60000, 35000,     0, 60000, 35000,     0, 60000, 35000,     0, 60000, 35000,     0 , 59000]
Bin 		   	= 'a'
BinCnt 	   	= 0

# === Initialize Box's Information ===
BoxId 			= [   'a',   'b',   'c']
BoxShift_X		= [59000, 32000,  1000 ]
BoxShift_Z		= [79000, 75000, 76000 ]

#=== Initialize Tote's Information === 
ToteId 		= [	  'a',   'b']
#ToteShift_X	= [49000, 11000 ]
ToteShift_X	= [60000, 11000 ]
ToteShift_Z	= [76000, 76000 ]


Box_A1 = 'A1'
Box_1AD = '1AD'
Box_1A5 = '1A5'
Box_1B2 = '1B2'
Box_K3 = 'K3'


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
