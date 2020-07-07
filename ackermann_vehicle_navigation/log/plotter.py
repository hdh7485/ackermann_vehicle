import numpy as np
import matplotlib.pyplot as plt
import os
from os.path import isfile, join

# Data path ("./planning_time" "./execution_time")
PLANNING_DATA_PATH = "./planning_time/rrt_rrtstar_neuralrrt"
EXECUTION_DATA_PATH = "./execution_time/rrt_rrtstar_neuralrrt" 

def get_planning_data(planning_data_path_list):
    planning_data = {'rrt_star'       : {'me652_parking_map': [], 'map_vertical_revised': [], 'map_e_shape': []},
                     'rrt'            : {'me652_parking_map': [], 'map_vertical_revised': [], 'map_e_shape': []},
                     'Neural_rrt'     : {'me652_parking_map': [], 'map_vertical_revised': [], 'map_e_shape': []},
                     'Neural_rrt_star': {'me652_parking_map': [], 'map_vertical_revised': [], 'map_e_shape': []}
                     }
                     
    for path in planning_data_path_list:
        # Filtering npz
        if path[-4:] == '.npz':
            # Filtering filename
            filename = path[:-4]
            # Filtering algorithm
            if filename[:8] == "rrt_star":
                # Save planning data
                planning_data = save_planning_time(filename=filename,
                                                    planner_name=filename[:8],
                                                    path=path, planning_data=planning_data)

            elif filename[:3] == "rrt":
                # Save planning data
                planning_data = save_planning_time(filename=filename,
                                                    planner_name=filename[:3],
                                                    path=path, planning_data=planning_data)
            
            elif filename[:15] == "Neural_rrt_star":
                # Save planning data
                planning_data = save_planning_time(filename=filename,
                                                    planner_name=filename[:15],
                                                    path=path, planning_data=planning_data)
            
            elif filename[:10] == "Neural_rrt":
                # Save planning data
                planning_data = save_planning_time(filename=filename,
                                                    planner_name=filename[:10],
                                                    path=path, planning_data=planning_data)

    # print("planning_data")
    # print(planning_data['rrt']['me652_parking_map'])
    # print(np.mean(planning_data['rrt']['me652_parking_map']))
    # planning_data["rrt_star"]['me652_parking_map'] = np.mean(planning_data["rrt_star"]['me652_parking_map'])
    # planning_data["rrt_star"]['map_vertical_revised'] = np.mean(planning_data["rrt_star"]['map_vertical_revised'])
    # planning_data["rrt_star"]['map_e_shape'] = np.mean(planning_data["rrt_star"]['map_e_shape'])

    # planning_data["rrt"]['me652_parking_map'] = np.mean(planning_data["rrt"]['me652_parking_map'])
    # planning_data["rrt"]['map_vertical_revised'] = np.mean(planning_data["rrt"]['map_vertical_revised'])
    # planning_data["rrt"]['map_e_shape'] = np.mean(planning_data["rrt"]['map_e_shape'])

    # planning_data["Neural_rrt"]['me652_parking_map'] = np.mean(planning_data["Neural_rrt"]['me652_parking_map'])
    # planning_data["Neural_rrt"]['map_vertical_revised'] = np.mean(planning_data["Neural_rrt"]['map_vertical_revised'])
    # planning_data["Neural_rrt"]['map_e_shape'] = np.mean(planning_data["Neural_rrt"]['map_e_shape'])

    # planning_data["Neural_rrt_star"]['me652_parking_map'] = np.mean(planning_data["Neural_rrt_star"]['me652_parking_map'])
    # planning_data["Neural_rrt_star"]['map_vertical_revised'] = np.mean(planning_data["Neural_rrt_star"]['map_vertical_revised'])
    # planning_data["Neural_rrt_star"]['map_e_shape'] = np.mean(planning_data["Neural_rrt_star"]['map_e_shape'])

    return planning_data

def save_planning_time(filename, planner_name, path, planning_data):
    map_name = ""
    if filename[-17:] == "me652_parking_map":
        map_name =  "me652_parking_map"
    elif filename[-20:] == "map_vertical_revised":
        map_name =  "map_vertical_revised"
    elif filename[-11:] == "map_e_shape":
        map_name =  "map_e_shape"
    else:
        raise NotImplementedError

    planning_time = np.load(os.path.join(PLANNING_DATA_PATH, path)).get('time')
    planning_data[planner_name][map_name].append(planning_time)

    return planning_data


def get_execution_data(execution_data_path_list):
    execution_data = {'rrt_star'       : {'me652_parking_map': [], 'map_vertical_revised': [], 'map_e_shape': []},
                      'rrt'            : {'me652_parking_map': [], 'map_vertical_revised': [], 'map_e_shape': []},
                      'Neural_rrt'     : {'me652_parking_map': [], 'map_vertical_revised': [], 'map_e_shape': []},
                      'Neural_rrt_star': {'me652_parking_map': [], 'map_vertical_revised': [], 'map_e_shape': []}
                      }

    for path in execution_data_path_list:
        if path[:-4] == "parking_lot":
            print(os.path.join(EXECUTION_DATA_PATH, path))
            with open(os.path.join(EXECUTION_DATA_PATH, path), mode='r') as txtfile:
                path_filter = len('/home/usrg-dhhan/Downloads/final_path/')

                for f in txtfile:
                    word_list = []
                    for word in f.split():
                        word_list.append(word)

                    filename = word_list[0][path_filter:-5]
                    execution_time = float(word_list[1])

                    if filename[:8] == "rrt_star":
                        execution_data["rrt_star"]['me652_parking_map'].append(execution_time)

                    elif filename[:3] == "rrt":
                        execution_data["rrt"]['me652_parking_map'].append(execution_time)
                    
                    elif filename[:15] == "Neural_rrt_star":
                        execution_data["Neural_rrt_star"]['me652_parking_map'].append(execution_time)
                    
                    elif filename[:10] == "Neural_rrt":
                        execution_data["Neural_rrt"]['me652_parking_map'].append(execution_time)
                    else:
                        raise NotImplementedError

        elif path[:-4] == "e_shape":
            print(os.path.join(EXECUTION_DATA_PATH, path))
            with open(os.path.join(EXECUTION_DATA_PATH, path), mode='r') as txtfile:
                path_filter = len('/home/usrg-dhhan/Downloads/final_path/')

                for f in txtfile:
                    word_list = []
                    for word in f.split():
                        word_list.append(word)

                    filename = word_list[0][path_filter:-5]
                    execution_time = float(word_list[1])

                    if filename[:8] == "rrt_star":
                        execution_data["rrt_star"]['map_e_shape'].append(execution_time)

                    elif filename[:3] == "rrt":
                        execution_data["rrt"]['map_e_shape'].append(execution_time)
                    
                    elif filename[:15] == "Neural_rrt_star":
                        execution_data["Neural_rrt_star"]['map_e_shape'].append(execution_time)
                    
                    elif filename[:10] == "Neural_rrt":
                        execution_data["Neural_rrt"]['map_e_shape'].append(execution_time)
                    else:
                        raise NotImplementedError

        elif path[:-4] == "vertical_log":
            print(os.path.join(EXECUTION_DATA_PATH, path))
            with open(os.path.join(EXECUTION_DATA_PATH, path), mode='r') as txtfile:
                path_filter = len('/home/usrg-dhhan/Downloads/final_path/')

                for f in txtfile:
                    word_list = []
                    for word in f.split():
                        word_list.append(word)

                    filename = word_list[0][path_filter:-5]
                    execution_time = float(word_list[1])

                    if filename[:8] == "rrt_star":
                        execution_data["rrt_star"]['map_vertical_revised'].append(execution_time)

                    elif filename[:3] == "rrt":
                        execution_data["rrt"]['map_vertical_revised'].append(execution_time)
                    
                    elif filename[:15] == "Neural_rrt_star":
                        execution_data["Neural_rrt_star"]['map_vertical_revised'].append(execution_time)
                    
                    elif filename[:10] == "Neural_rrt":
                        execution_data["Neural_rrt"]['map_vertical_revised'].append(execution_time)
                    else:
                        raise NotImplementedError

#    execution_data["rrt_star"]['me652_parking_map'] = np.mean(execution_data["rrt_star"]['me652_parking_map'])
#    execution_data["rrt_star"]['map_vertical_revised'] = np.mean(execution_data["rrt_star"]['map_vertical_revised'])
#    execution_data["rrt_star"]['map_e_shape'] = np.mean(execution_data["rrt_star"]['map_e_shape'])
#
#    execution_data["rrt"]['me652_parking_map'] = np.mean(execution_data["rrt"]['me652_parking_map'])
#    execution_data["rrt"]['map_vertical_revised'] = np.mean(execution_data["rrt"]['map_vertical_revised'])
#    execution_data["rrt"]['map_e_shape'] = np.mean(execution_data["rrt"]['map_e_shape'])
#
#    execution_data["Neural_rrt"]['me652_parking_map'] = np.mean(execution_data["Neural_rrt"]['me652_parking_map'])
#    execution_data["Neural_rrt"]['map_vertical_revised'] = np.mean(execution_data["Neural_rrt"]['map_vertical_revised'])
#    execution_data["Neural_rrt"]['map_e_shape'] = np.mean(execution_data["Neural_rrt"]['map_e_shape'])
#
#    execution_data["Neural_rrt_star"]['me652_parking_map'] = np.mean(execution_data["Neural_rrt_star"]['me652_parking_map'])
#    execution_data["Neural_rrt_star"]['map_vertical_revised'] = np.mean(execution_data["Neural_rrt_star"]['map_vertical_revised'])
#    execution_data["Neural_rrt_star"]['map_e_shape'] = np.mean(execution_data["Neural_rrt_star"]['map_e_shape'])

    print(execution_data)

    return execution_data
                

def plotter_main():
    # Data list
    planning_data_path_list = [f for f in os.listdir(PLANNING_DATA_PATH) if isfile(join(PLANNING_DATA_PATH, f))]
    execution_data_path_list = [f for f in os.listdir(EXECUTION_DATA_PATH) if isfile(join(EXECUTION_DATA_PATH, f))]
    
    # Get data
    planning_data = get_planning_data(planning_data_path_list)
    execution_data = get_execution_data(execution_data_path_list)

    return planning_data, execution_data


if __name__ == '__main__':
    plotter_main()
