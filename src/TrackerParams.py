'''
@author james.staley625703@tufts.edu
TODO: This should go into a rosparam server
'''
from typing import NamedTuple
from matplotlib.colors import rgb_to_hsv

class TrackerParams(NamedTuple):
    hsv_lower: tuple
    hsv_upper: tuple


Sphero_RGB_Color = {
    "sd9": (255, 0, 0),
    "sf8": (0, 0, 255),
}

# How the colors appear (done by hand)
Sphero_HSV_Color = {
    "sd9": (35, 13, 240),
    "sf8": (92, 90, 255),
}
    
Sphero_Params_by_ID = {
    "sd9": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255)),
    "sf8": TrackerParams(hsv_lower=(0, 0, 0), hsv_upper=(255, 255, 255))
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
    print(f"{k}: {hsv}")
    print(f"{k}: {hsv_lower}, {hsv_upper}")
