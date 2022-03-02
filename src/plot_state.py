from matplotlib import pyplot as plt

#!/usr/bin/env python3

'''
Spoof file to produce fake sphero data for testing (written for transfer entropy testing)
@author james.staley625703@tufts.edu
'''

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.path as mpath

from TrackerParams import Sphero_RGB_Color

plt.ion()
fig, ax = plt.subplots()
ax.set_title("Sphero Positions")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.plot()
colors = ['b', 'darkorange', 'r']

SCALE = 1
marker_path = [(SCALE*x, SCALE*y) for (x,y) in 
        [
           (-1.0, 0.8), (-1.0, -0.8), (1.0, 0.0), (-1.0, 0.8)
            # (0., -1.0), (0., 1.0), (1.0, 1.0), (1.0, -1.0), (0., -1.0)
        ]]
oriented_marker = mpath.Path(
    marker_path, 
    # [mpath.Path.MOVETO, mpath.Path.LINETO, mpath.Path.LINETO, mpath.Path.LINETO, mpath.Path.CLOSEPOLY])
    [mpath.Path.MOVETO, mpath.Path.LINETO, mpath.Path.LINETO, mpath.Path.CLOSEPOLY])

def plot_spheros(sphero_xythetas, sphero_ids, ax_x_range=[-0.05, 1.05], ax_y_range=[-0.05, 1.05]):
        ax.clear()
        xs = [x for x,y,theta in sphero_xythetas]
        ys = [y for x,y,theta in sphero_xythetas]
        thetas = [theta for x,y,theta in sphero_xythetas]
        for idx, (x,y,theta) in enumerate(sphero_xythetas):
            marker = oriented_marker.transformed(mpl.transforms.Affine2D().rotate(theta))
            # ax.scatter(x, y, marker=marker, s=500, c=colors[idx])
            ax.scatter(x, y, marker=marker, s=500, c=[entry/255. for entry in Sphero_RGB_Color[sphero_ids[idx]]])

        ax.set_title("Spoofed Sphero Positions")
        ax.set_ylim(ax_y_range)
        ax.set_xlim(ax_x_range)
        ax.legend(sphero_ids)
        fig.canvas.draw()
        fig.canvas.flush_events()


if __name__ == "__main__":
    pass
# main()