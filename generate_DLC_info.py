import numpy as np
import matplotlib.pyplot as plt
import math

# Vehicle parameters
VEHICLE_WIDTH = 2.0 # in meters

# ISO 3888-2 Standard all in meters
B1 = 1.1 * VEHICLE_WIDTH + 0.25 
B3 = VEHICLE_WIDTH + 1
B5 = min(1.3 * VEHICLE_WIDTH + 0.25, 3.0)

SECTION_LENGTHS = [12.0, 13.5, 11.0, 12.5, 12.0]
SECTION_WIDTHS  = [  B1, None,   B3, None,   B5]
SECTION_OFFSETS = [ 0.0,  0.0,  1.0,  0.0,  0.0]

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
    if SECTION_WIDTHS[i] is None:
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

plt.figure()
plt.title("DLC Maneuver Cones and Trajectory")
plt.scatter(cones_x, cones_y, c='tab:orange', label='Cones')
plt.legend()
plt.show()