#!/usr/bin/env python

"""Define all of labels."""

from random import seed, randint


amazonLabelsList = [
    "avery_binder",
    "ballons",
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
    "windex",
    "all"
]

ourLabelsList = [
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
    "windexSprayBottle23oz",
    "all"
]

# Generate random color for objects
seed()
colors = dict()
for name in amazonLabelsList:
    color = (randint(100, 255), randint(100, 255), randint(100, 255))
    colors[name] = color