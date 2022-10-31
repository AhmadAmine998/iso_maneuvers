import numpy as np
import yaml

def point_to_coord(x, y, origin_x, origin_y, resolution):
    pixel_x = np.round((x - origin_x)/resolution)
    pixel_y = np.round((y - origin_y)/resolution)
    return pixel_x, pixel_y

def read_yaml_file(map_yaml, free_uncertain=False, interior_idx=None, exterior_idx=None):
    try:
        with open(map_yaml, "r") as stream:
            try:
                map_cfgs = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print("Error reading file " + map_yaml + "!")
                print(exc)
                return 
    except:
        print(map_yaml + " is either not valid or is not a string!")
        return
    
    return map_cfgs
    