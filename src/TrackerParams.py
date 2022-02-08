'''
@author james.staley625703@tufts.edu
TODO: This should go into a rosparam server
'''
from typing import NamedTuple
from matplotlib.colors import rgb_to_hsv

class TrackerParams(NamedTuple):
    hsv_lower: tuple
    hsv_upper: tuple


BLUE_RGB = (0, 0, 255)
BLUE_HSV = (110,100,250)

RED_RGB = (255, 0, 0)
RED_HSV = (170, 80, 240)

GREEN_RGB = (0, 126, 0)
GREEN_HSV = (82, 50, 200)

# YELLOW_RGB = (0, 255, 255)
# YELLOW_HSV = (30, 100, 240)

Sphero_RGB_Color = {
    "sd9": (255, 0, 0),
    "sf8": (0, 0, 255),
    "se9": (0, 0, 255),
    "sf6": (255, 0, 0),
    "sdc": (255, 0, 0),
    "sec": (255, 0, 0),
    "sca": (255, 0, 0),
    "sfd": (255, 0, 0),
    "sfb": (255, 0, 0),
    "sd7": (255, 0, 0),
    "sd1": (255, 0, 0),
    "sc8": (255, 0, 0),
    "sd1": (255, 0, 0),
    "scD": (255, 0, 0),
    "sf0": (255, 0, 0),
    "sc9": (255, 0, 0),
}

# How the colors appear (done by hand)
Sphero_HSV_Color = {
    "sd9": RED_HSV, #(35, 13, 240),
    "sf8": BLUE_HSV, #(92, 90, 255),
    "se9": BLUE_HSV, #(92, 90, 255),
    "sf6": RED_HSV,
    "sdc": RED_HSV,
    "sec": RED_HSV,
    "sca": RED_HSV,
    "sfd": RED_HSV,
    "sfb": RED_HSV,
    "sd7": RED_HSV,
    "sd1": RED_HSV,
    "sc8": RED_HSV,
    "sd1": RED_HSV,
    "scD": RED_HSV,
    "sf0": RED_HSV,
    "sc9": RED_HSV,
}
    
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

hue_range = 10
sat_range = 20
val_range = 10
# now go through and set the correct params
for k,v in Sphero_Params_by_ID.items():
    # hsv = rgb_to_hsv([entry/255. for entry in Sphero_RGB_Color[k]])
    # hsv = [int(255*entry) for entry in hsv]
    hsv = Sphero_HSV_Color[k]
    hsv_lower = (max(0, hsv[0] - hue_range), max(0, hsv[1] - sat_range), max(0, hsv[2] - val_range))
    hsv_upper = (min(255, hsv[0] + hue_range), min(255, hsv[1] + sat_range), min(255, hsv[2] + val_range))
    Sphero_Params_by_ID[k] = TrackerParams(hsv_lower=hsv_lower, hsv_upper=hsv_upper)
    # print(f"{k}: {hsv}")
    # print(f"{k}: {hsv_lower}, {hsv_upper}")


# get range for green (orientation light)
hsv = GREEN_HSV
LOWER_GREEN = (max(0, hsv[0] - hue_range), max(0, hsv[1] - sat_range), max(0, hsv[2] - val_range))
UPPER_GREEN = (min(255, hsv[0] + hue_range), min(255, hsv[1] + sat_range), min(255, hsv[2] + val_range)) 