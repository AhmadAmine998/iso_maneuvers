import numpy as np
import matplotlib.pyplot as plt
import scipy

# utils
from utils import bezier_interpolation

def generate_splines(interp_linear=True):
    # Used to generate splines
    XP0 = START_X
    A = SECTION_LENGTHS[0] - 5.0
    XP1 = START_X + SECTION_LENGTHS[0]
    XP2 = XP1 + SECTION_LENGTHS[1]
    B = (SECTION_LENGTHS[0] + SECTION_LENGTHS[1]) + 5.0
    C = (B - 5.0 + SECTION_LENGTHS[2]) - 5.0
    XP3 = XP2 + SECTION_LENGTHS[2]
    XPM = (XP3 + XP2) / 2.0
    XP4 = XP3 + SECTION_LENGTHS[3]
    D = (C + 5.0 + SECTION_LENGTHS[3]) + 5.0
    XP5 = XP4 + SECTION_LENGTHS[4]

    waypoints_y = np.array([
                            XP0, 
                            A, 
                            XP1, 
                            XP2,
                            B, 
                            C, 
                            XP3,
                            XP4,
                            D,
                            XP5
                            ])

    # y-coordinates of spline
    wpt_offset = SECTION_WIDTHS[2] / 2.0 + SECTION_WIDTHS[0]/2.0 + SECTION_OFFSETS[2]

    waypoints_x = np.array([
                            START_Y,        # XP0
                            START_Y,        # A
                            START_Y,        # XP1
                            wpt_offset,     # XP2
                            wpt_offset,     # B
                            wpt_offset,     # C
                            wpt_offset,     # XP3
                            START_Y,        # XP4
                            START_Y,        # D
                            START_Y         # XP5
                            ])

    
    if interp_linear:
        # Dense linear interpolation for more accurate bezier curve
        spl = scipy.interpolate.interp1d(waypoints_x, waypoints_y, kind='slinear')
        x_new = np.arange(waypoints_x[0], waypoints_y[-1], 0.25)
        y_new = spl(x_new)

    points = np.vstack((x_new, y_new)).T
    x_new, y_new = bezier_interpolation(points)

    return x_new, y_new, waypoints_x, waypoints_y

# Vehicle parameters
VEHICLE_WIDTH = 2.0 # in meters

# ISO 3888-2 Standard all in meters
B1 = 1.1 * VEHICLE_WIDTH + 0.25
B3 = VEHICLE_WIDTH + 1
B5 = min(1.3 * VEHICLE_WIDTH + 0.25, 3.0)

SECTION_LENGTHS = np.array([12.0,   13.5, 11.0,   12.5, 12.0])
SECTION_WIDTHS  = np.array([  B1, np.inf,   B3, np.inf,   B5])
SECTION_OFFSETS = np.array([ 0.0,    0.0,  1.0,    0.0,  0.0])

CONE_WIDTH = 0.285

# Desired starting coordinates of trajectory
START_X = 0.0
START_Y = 0.0

# Desired cone spacing
CONE_SPACING = CONE_WIDTH * 10.0

# Arrays of cone coordinates
cones_x = None
cones_y = None

# Current x-coordinate along trajectory
curr_x = START_X

# width of last non-empty section
prev_width = 0.0
for i, length in enumerate(SECTION_LENGTHS):
    if SECTION_WIDTHS[i] == np.inf:
        # no cones
        curr_x += length
        continue
    
    center_to_cone = SECTION_WIDTHS[i] / 2.0
    if SECTION_OFFSETS[i] != 0:
        # find offset using ISO standard definition of offset
        offset = center_to_cone + prev_width/2.0 + SECTION_OFFSETS[i]
    else:
        offset = 0.0

    new_cones_x = np.array([np.arange(curr_x, curr_x + length, CONE_SPACING)]*2).flatten()
    new_cones_y_up   = np.repeat((START_Y + offset) + SECTION_WIDTHS[i]/2.0, new_cones_x.shape[0])
    new_cones_y_down = np.repeat((START_Y + offset) - SECTION_WIDTHS[i]/2.0, new_cones_x.shape[0])

    # Add cones_x up and down
    if cones_x is None:
        cones_x = new_cones_x
        cones_x = np.append(cones_x, new_cones_x)
    elif cones_x is not None:
        cones_x = np.append(cones_x, new_cones_x)
        cones_x = np.append(cones_x, new_cones_x)

    # Add cones_y up and down
    if cones_y is None:
        cones_y = new_cones_y_up
        cones_y = np.append(cones_y, new_cones_y_down)
    elif cones_y is not None:
        cones_y = np.append(cones_y, new_cones_y_up)
        cones_y = np.append(cones_y, new_cones_y_down)

    # move forward by a length
    curr_x += length

    # update previous width
    prev_width = SECTION_WIDTHS[i]

# Get interpolated spline from waypoints
x_new, y_new, waypoints_x, waypoints_y = generate_splines(interp_linear=True)

plt.figure()
plt.title("DLC Maneuver Cones and Trajectory")
plt.scatter(cones_x, cones_y, c='tab:orange', label='Cones')
plt.scatter(x_new, y_new)
plt.scatter(waypoints_x, waypoints_y, c='r')
plt.legend()
plt.show()