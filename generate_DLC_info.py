from ast import Not
import numpy as np
import matplotlib.pyplot as plt
import scipy
import datetime
import os

# utils
from utils.bezier_interpolation import interp_bezier


CONE_WIDTH = 0.285

def generate_DLC_splines_and_waypoints(section_lengths, section_widths, section_offsets, cones_x_array, cones_y_array, n_maneuver = 0, interp_linear=True):
    repeat = n_maneuver - 1 

    # Used to generate splines
    XP0 = START_X
    A = section_lengths[0] - 5.0
    XP1 = START_X + section_lengths[0]
    XP2 = XP1 + section_lengths[1]
    B = (section_lengths[0] + section_lengths[1]) + 5.0
    C = (B - 5.0 + section_lengths[2]) - 5.0
    XP3 = XP2 + section_lengths[2]
    XPM = (XP3 + XP2) / 2.0
    XP4 = XP3 + section_lengths[3]
    D = (C + 5.0 + section_lengths[3]) + 5.0
    XP5 = XP4 + section_lengths[4]

    # y-coordinates of spline
    wpt_offset = section_widths[2] / 2.0 + section_widths[0]/2.0 + section_offsets[2]

    waypoints_x = np.array([
                            XP0, 
                            A, 
                            XP1, 
                            XP2,
                            B, 
                            C, 
                            XP3,
                            XP4,
                            # D,
                            # XP5
                            ])

    waypoints_y = np.array([
                            START_Y,        # XP0
                            START_Y,        # A
                            START_Y,        # XP1
                            wpt_offset,     # XP2
                            wpt_offset,     # B
                            wpt_offset,     # C
                            wpt_offset,     # XP3
                            START_Y,        # XP4
                            # START_Y,        # D
                            # START_Y         # XP5
                            ])

    waypoints_xrep = waypoints_x.copy()
    waypoints_yrep = waypoints_y.copy()

    cones_xrep = [cones_x.copy() for cones_x in cones_x_array]
    cones_yrep = [cones_y.copy() for cones_y in cones_y_array]
    SEGMENT_WIDTH = waypoints_xrep[-1]
    if repeat < 2:
        for i in range(repeat):
            waypoints_xrep = np.append(waypoints_xrep, waypoints_x + waypoints_xrep[-1])
            waypoints_yrep = np.append(waypoints_yrep, waypoints_y)
        
        waypoints_xrep = np.append(waypoints_xrep, np.array([D, XP5])+ waypoints_xrep[-1] - XP4)
        waypoints_yrep = np.append(waypoints_yrep, np.array([START_Y, START_Y]))

        for i, (cones_x, cones_y) in enumerate(zip(cones_xrep, cones_yrep)):
            # Clean generation of cones for reference
            cone_start = (np.abs(cones_x - XP2)).argmin()
            for j in range(repeat):
                cones_xrep[i] = np.append(cones_xrep[i], cones_x[cone_start:] + (SEGMENT_WIDTH)*(j+1))
                cones_yrep[i] = np.append(cones_yrep[i], cones_y[cone_start:])
    else:
        # Generate one pattern
        for i in range(2):
            waypoints_xrep = np.append(waypoints_xrep, waypoints_x + waypoints_xrep[-1])
            waypoints_yrep = np.append(waypoints_yrep, waypoints_y)

        waypoints_xrep = np.append(waypoints_xrep, np.array([D, XP5])+ waypoints_xrep[-1] - XP4)
        waypoints_yrep = np.append(waypoints_yrep, np.array([START_Y, START_Y]))

    if interp_linear:
        # Dense linear interpolation for more accurate bezier curve
        spl = scipy.interpolate.interp1d(waypoints_xrep, waypoints_yrep, kind='slinear')
        x_new = np.arange(waypoints_xrep[0], waypoints_xrep[-1], 0.16)
        y_new = spl(x_new)
    else:
        x_new = waypoints_xrep.copy()
        y_new = waypoints_yrep.copy()

    points = np.vstack((x_new, y_new)).T
    x_new, y_new = interp_bezier(points)

    if repeat >= 2:
        # repeat by copy
        loc_start = XP4 + SEGMENT_WIDTH
        idx_start = (np.abs(x_new - loc_start)).argmin()

        loc_end   = XP4 
        idx_end   = (np.abs(x_new - loc_end)).argmin()

        repeat_x = x_new[idx_start:idx_end].copy()
        repeat_y = y_new[idx_start:idx_end].copy()
        
        # Generate repeat pattern
        for i in range(repeat - 2):
            repeat_x = np.append(repeat_x, x_new[idx_start:idx_end] + repeat_x[-1])
            repeat_y = np.append(repeat_y, y_new[idx_start:idx_end])

        # Start at start
        x_new_rep = x_new[idx_end:].copy()
        y_new_rep = y_new[idx_end:].copy()

        # Replace middle with repeat
        x_new_rep = np.append(x_new_rep, repeat_x.copy())
        y_new_rep = np.append(y_new_rep, repeat_y.copy())

        # end at end
        end_seg_x = x_new[:idx_start]
        end_seg_y = y_new[:idx_start]
        x_new_rep = np.append(x_new_rep, x_new[:idx_start] - end_seg_x[-1] + repeat_x.max())
        y_new_rep = np.append(y_new_rep, y_new[:idx_start])

        x_new = x_new_rep.copy()
        y_new = y_new_rep.copy()

        # Clean generation of waypoints for reference
        waypoints_xrep = waypoints_x.copy()
        waypoints_yrep = waypoints_y.copy()
        for i in range(repeat):
            waypoints_xrep = np.append(waypoints_xrep, waypoints_x + waypoints_xrep[-1])
            waypoints_yrep = np.append(waypoints_yrep, waypoints_y)

        waypoints_xrep = np.append(waypoints_xrep, np.array([D, XP5])+ waypoints_xrep[-1] - XP4)
        waypoints_yrep = np.append(waypoints_yrep, np.array([START_Y, START_Y]))


        for i, (cones_x, cones_y) in enumerate(zip(cones_xrep, cones_yrep)):
            # Clean generation of cones for reference
            cone_start = (np.abs(cones_x - XP2)).argmin()
            for j in range(repeat):
                cones_xrep[i] = np.append(cones_xrep[i], cones_x[cone_start:] + (SEGMENT_WIDTH)*(j+1))
                cones_yrep[i] = np.append(cones_yrep[i], cones_y[cone_start:])
    
    return x_new, y_new, waypoints_xrep, waypoints_yrep, cones_xrep, cones_yrep

def generate_DLC_cone_locations(start_x, start_y, vehicle_width, cone_spacing):
    # ISO 3888-2 Standard all in meters
    B1 = 1.1 * vehicle_width + 0.25
    B3 = vehicle_width + 1
    B5 = min(1.3 * vehicle_width + 0.25, 3.0)

    section_lengths = np.array([12.0,   13.5, 11.0,   12.5, 12.0])
    section_widths  = np.array([  B1, np.inf,   B3, np.inf,   B5])
    section_offsets = np.array([ 0.0,    0.0,  1.0,    0.0,  0.0])

    # Arrays of cone coordinates
    cones_x_left  = None
    cones_x_right = None

    # Add cones_y up and down
    cones_y_left  = None
    cones_y_right = None

    # Current x-coordinate along trajectory
    curr_x = start_x

    # width of last non-empty section
    prev_width = 0.0
    for i, length in enumerate(section_lengths):
        if section_widths[i] == np.inf:
            # no cones
            curr_x += length
            continue
        
        center_to_cone = section_widths[i] / 2.0
        if section_offsets[i] != 0:
            # find offset using ISO standard definition of offset
            offset = center_to_cone + prev_width/2.0 + section_offsets[i]
        else:
            offset = 0.0

        new_cones_x_up   = np.array([np.arange(curr_x, curr_x + length, cone_spacing)]).flatten()
        new_cones_x_down = np.array([np.arange(curr_x, curr_x + length, cone_spacing)]).flatten()
        new_cones_y_up   = np.repeat((start_y + offset) + section_widths[i]/2.0, new_cones_x_up.shape[0])
        new_cones_y_down = np.repeat((start_y + offset) - section_widths[i]/2.0, new_cones_x_down.shape[0])

        # Add cones_x up and down
        if cones_x_left is None:
            cones_x_left  = new_cones_x_up
        elif cones_x_left is not None:
            cones_x_left  = np.append(cones_x_left, new_cones_x_up)
        if cones_x_right is None:
            cones_x_right = new_cones_x_down
        elif cones_x_right is not None:
            cones_x_right = np.append(cones_x_right, new_cones_x_down)

        # Add cones_y up and down
        if cones_y_left is None:
            cones_y_left  = new_cones_y_up
        elif cones_y_left is not None:
            cones_y_left  = np.append(cones_y_left, new_cones_y_up)
        if cones_y_right is None:
            cones_y_right = new_cones_y_down
        elif cones_y_right is not None:
            cones_y_right = np.append(cones_y_right, new_cones_y_down)

        # move forward by a length
        curr_x += length

        # update previous width
        prev_width = section_widths[i]
    
    return section_lengths, section_widths, section_offsets, cones_x_left, cones_x_right, cones_y_left, cones_y_right

if __name__ == '__main__':
    # Save Results ?
    SAVE = True

    # Vehicle parameters
    VEHICLE_WIDTH = 2.0 # in meters

    # Desired Number of Chained maneuvers
    NUM_CHAINED_MANEUVERS = 10

    # Desired starting coordinates of trajectory
    START_X = 0.0
    START_Y = 0.0

    # Desired cone spacing
    CONE_SPACING = CONE_WIDTH * 10.0

    # Generate waypoints from ISO-38882 standard
    section_lengths, section_widths, section_offsets, cones_x_left, cones_x_right, cones_y_left, cones_y_right = generate_DLC_cone_locations(START_X, START_Y, VEHICLE_WIDTH, CONE_SPACING)

    # Get interpolated spline from waypoints
    x_new, y_new, waypoints_x, waypoints_y, cones_x, cones_y = generate_DLC_splines_and_waypoints(section_lengths, section_widths, section_offsets, [cones_x_left, cones_x_right], [cones_y_left, cones_y_right], n_maneuver=NUM_CHAINED_MANEUVERS, interp_linear=True)

    plt.figure()
    plt.title("DLC Maneuver Cones and Trajectory")
    plt.scatter(cones_x, cones_y, c='tab:orange', label='Cones')
    plt.scatter(x_new, y_new)
    plt.scatter(waypoints_x, waypoints_y, c='r')
    plt.legend()
    plt.show()

    if SAVE:
        # Specify path
        saveDir = './results/DLC/'
        
        # Check if saveDir exists
        if os.path.exists(saveDir) == False:
            if os.path.exists('./results') == False:
                os.mkdir('./results')
            os.mkdir('./results/DLC')
        
        now = datetime.datetime.now()
        dt_string = now.strftime("%Y%m%d_%H%M%S")
        os.mkdir('./results/DLC/'+dt_string)
        saveDir = saveDir + dt_string

        # Key Waypoints
        np.savetxt(saveDir + '/waypoints' + dt_string + '.csv', np.vstack((waypoints_x, waypoints_y)).T, header="x_m,y_m", delimiter=',')

        # Trajectory
        np.savetxt(saveDir + '/trajectory' + dt_string + '.csv', np.vstack((x_new, y_new)).T, header="x_m,y_m", delimiter=',')

        # Cone Locations
        np.savetxt(saveDir + '/cones_left' + dt_string + '.csv', np.vstack((cones_x[0], cones_y[0])).T, header="x_m,y_m", delimiter=',')
        np.savetxt(saveDir + '/cones_right' + dt_string + '.csv', np.vstack((cones_x[1], cones_y[1])).T, header="x_m,y_m", delimiter=',')
