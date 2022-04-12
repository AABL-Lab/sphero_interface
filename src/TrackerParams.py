'''
@author james.staley625703@tufts.edu
TODO: This should go into a rosparam server
'''
from typing import NamedTuple
from matplotlib.colors import rgb_to_hsv
import rospy

class TrackerParams(NamedTuple):
    hsv_lower: tuple
    hsv_upper: tuple

TRACK_WITH_SQUARES = True
TRACK_WITH_CIRCLES = not TRACK_WITH_SQUARES

BLUE_RGB = (0, 0, 255)
BLUE_HSV = (100, 100, 200)

RED_RGB = (255, 0, 0)
RED_HSV = (170, 170, 200)

MAGENTA_RGB = (255, 0, 255)
MAGENTA_HSV = (140, 70, 200)

YELLOW_RGB = (255, 100, 100)
YELLOW_HSV = (165, 2, 255)

GREEN_RGB = (0, 255, 0)
GREEN_HSV = (145, 30, 120)

WHITE_RGB = (255, 255, 255)
WHITE_HSV = (0, 0, 255)

sphero_ids = ["sd9", "sf8", "se9", "sf6", "sdc", "sec", "sca", "sfd", "sfb", "sd7", "sd1", "sc8", "scD", "sf0", "sc9"]

Sphero_RGB_Color = {
    "sd9": RED_RGB, # jss home
    "sf8": BLUE_RGB, # jss home
    "se9": BLUE_RGB,
    "sf6": RED_RGB,
    "sdc": RED_RGB,
    "sec": RED_RGB, # in lab
    "sca": BLUE_RGB, # in lab
    "sd1": GREEN_RGB, # in lab
    "sfd": RED_RGB,
    "sfb": RED_RGB,
    "sd7": RED_RGB,
    "sc8": YELLOW_RGB, # in lab
    "scD": RED_RGB,
    "sf0": RED_RGB,
    "sc9": RED_RGB,
}

def string_for_color(color):
    if (color == RED_RGB): return "red"
    elif (color == BLUE_RGB): return "blue"
    elif (color == YELLOW_RGB): return "yellow"
    elif (color == MAGENTA_RGB): return "magenta"
    elif (color == GREEN_RGB): return "green"
    else: return "unknown"
    
id_to_colorstring = {k: string_for_color(v) for k, v in Sphero_RGB_Color.items()}

colorstring_to_id = {id_to_colorstring[k]:k for k in ["sd9", "sf8", "sec", "sca", "sd1"]}

Sphero_HSV_Color = dict()
def populate_hsv_dict():
    Sphero_HSV_Color.clear()
    # How the colors appear (done by hand)
    for sphero_id, color in Sphero_RGB_Color.items():
        if (color == RED_RGB): Sphero_HSV_Color[sphero_id] = RED_HSV
        elif (color == BLUE_RGB): Sphero_HSV_Color[sphero_id] = BLUE_HSV
        elif (color == YELLOW_RGB): Sphero_HSV_Color[sphero_id] = YELLOW_HSV
        elif (color == MAGENTA_RGB): Sphero_HSV_Color[sphero_id] = MAGENTA_HSV
        elif (color == GREEN_RGB): Sphero_HSV_Color[sphero_id] = GREEN_HSV
        else: Sphero_HSV_Color[sphero_id] = (-1,-1,-1)
populate_hsv_dict()

Sphero_Params_by_ID = {
    "sd9": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sf8": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "se9": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sf6": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sdc": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sec": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sca": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sfd": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sfb": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sd7": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sd1": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sc8": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sd1": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "scD": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sf0": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sc9": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
}

hue_range = 20
sat_range = 140
val_range = 100
# now go through and set the correct params
for k,v in Sphero_Params_by_ID.items():
    # hsv = rgb_to_hsv([entry/255. for entry in Sphero_RGB_Color[k]])
    # hsv = [int(255*entry) for entry in hsv]
    hsv = Sphero_HSV_Color[k]
    hsv_lower = (max(0, hsv[0] - hue_range), max(0, hsv[1] - sat_range), 100)
    hsv_upper = (min(255, hsv[0] + hue_range), min(255, hsv[1] + sat_range), 255)
    # hsv_lower = (max(0, hsv[0] - hue_range), max(0, hsv[1] - sat_range), max(0, hsv[2] - val_range))
    # hsv_upper = (min(255, hsv[0] + hue_range), min(255, hsv[1] + sat_range), min(255, hsv[2] + val_range))
    Sphero_Params_by_ID[k] = TrackerParams(hsv_lower=hsv_lower, hsv_upper=hsv_upper)


# get range for green (orientation light)
hsv = GREEN_HSV
LOWER_GREEN = (max(0, hsv[0] - hue_range), max(0, hsv[1] - sat_range), max(0, hsv[2] - val_range))
UPPER_GREEN = (min(255, hsv[0] + hue_range), min(255, hsv[1] + sat_range), min(255, hsv[2] + val_range)) 

hsv = WHITE_HSV
LOWER_WHITE = (max(0, hsv[0] - hue_range), max(0, hsv[1] - sat_range), max(0, hsv[2] - val_range))
UPPER_WHITE = (min(255, hsv[0] + hue_range), min(255, hsv[1] + sat_range), min(255, hsv[2] + val_range)) 

def hsv_bounds_for_id(sphero_id):
    hsv = Sphero_HSV_Color[sphero_id]
    hue_range = (rospy.get_param("/param_server/hue_min"), rospy.get_param("/param_server/hue_max"))
    sat_range = (rospy.get_param("/param_server/sat_min"), rospy.get_param("/param_server/sat_max"))
    val_range = (rospy.get_param("/param_server/val_min"), rospy.get_param("/param_server/val_max"))
    hsv_lower = (max(0, hsv[0] - hue_range[0]), max(0, hsv[1] - sat_range[0]), max(0, hsv[2] - val_range[0]))
    hsv_upper = (min(255, hsv[0] + hue_range[1]), min(255, hsv[1] + sat_range[1]), min(255, hsv[2] + val_range[1]))
    return (hsv_lower, hsv_upper)