import numpy as np
import glob
import csv
import matplotlib.pyplot as plt
from plotter import plotter_main

execution_time_array = np.zeros((3, 10))
planning_time_array = np.zeros((3, 10))

with open('kino_time.txt') as execution_time_file:
    lines = execution_time_file.readlines()
    for line in lines:
        world_index = int(line[43])
        path_index = int(line[45])
        execution_time = float(line[52:-1])
        execution_time_array[world_index-1, path_index] = execution_time
        # line_index += 1

csv_list = glob.glob('*.csv')
for csv_file in csv_list:
    world_index = int(str(csv_file)[6])
    path_index = int(str(csv_file)[8])
    with open(csv_file, 'r') as csv_file:
        time_reader = csv.reader(csv_file)
        planning_time = float(next(time_reader)[4])
    print(world_index, path_index, planning_time)
    planning_time_array[world_index-1, path_index] = planning_time

print(planning_time_array)
print(execution_time_array)

mean_execution_time = np.zeros((3, 5))
mean_planning_time = np.zeros((3, 5))
std_execution_time = np.zeros((3, 5))
std_planning_time = np.zeros((3, 5))

planning_data, execution_data = plotter_main()

# map index
for map_index in range(3):
    if map_index == 0:
        map_key = 'me652_parking_map'
    elif map_index == 1:
        map_key = 'map_vertical_revised'
    elif map_index == 2:
        map_key = 'map_e_shape'

    for algorithm_index in range(5):
        if algorithm_index == 0:
            algorithm_key = 'rrt'
        elif algorithm_index == 1:
            algorithm_key = 'rrt_star'
        elif algorithm_index == 2:
            algorithm_key = 'Neural_rrt'
        elif algorithm_index == 3:
            algorithm_key = 'Neural_rrt_star'
        elif algorithm_index == 4:
            mean_execution_time[map_index, algorithm_index] = np.mean(execution_time_array[map_index])
            std_execution_time[map_index, algorithm_index] = np.std(execution_time_array[map_index])

            mean_planning_time[map_index, algorithm_index] = np.mean(planning_time_array[map_index])
            std_planning_time[map_index, algorithm_index] = np.mean(planning_time_array[map_index])
            continue

        mean_execution_time[map_index, algorithm_index] = np.mean(execution_data[algorithm_key][map_key])
        std_execution_time[map_index, algorithm_index] = np.std(execution_data[algorithm_key][map_key])

        mean_planning_time[map_index, algorithm_index] = np.mean(planning_data[algorithm_key][map_key])
        std_planning_time[map_index, algorithm_index] = np.mean(planning_data[algorithm_key][map_key])
    

# Plotting #
title = ['Horizontal Map', 'Vertical Map', 'E-shape Map']
for world_index in range(3):
    fig, ax = plt.subplots(figsize=(8, 6))
    
    labels = ['RRT', 'RRT*', 'Neural RRT', 'Neural RRT*', 'Kinodynamic RRT*']
    width = 0.35
    ax.bar(labels, mean_planning_time[world_index], width, yerr=std_planning_time[world_index], label='PlanningTime')
    ax.bar(labels, mean_execution_time[world_index], width, yerr=std_execution_time[world_index], bottom=mean_planning_time[world_index],
           label='ExecutionTime')
    
    ax.set_ylabel('Time (sec)')
    ax.set_title(title[world_index])
    ax.legend()
    
    for i in range(5):
        ax.text(i, mean_planning_time[world_index][i], str(mean_planning_time[world_index][i])[:5])
        ax.text(i, mean_execution_time[world_index][i]+mean_planning_time[world_index][i], str(mean_execution_time[world_index][i]+mean_planning_time[world_index][i])[:5])
    
    plt.show()