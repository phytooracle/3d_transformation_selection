#!/usr/bin/env python3
"""
Author : Emmanuel Gonzalez
Date   : 2023-06-14
Purpose: 3D Point Cloud Transformation Selector Graphical User Interface
"""
import open3d as o3d
import subprocess
import tkinter as tk
from tkinter import simpledialog
from tkinter import ttk
from tkinter import filedialog
import tarfile
import os
import sys
# from open3d.visualization import VisualizerWithKeyCallback
import numpy as np
import copy
import time
import pandas as pd
import multiprocessing
import json
from open3d.visualization.gui import Application
import h5py
sys.setrecursionlimit(10000) # Set the maximum recursion depth to 10000

root = tk.Tk()
root.withdraw()

dir_paths = {
    "Season 10": "/iplant/home/shared/phytooracle/season_10_lettuce_yr_2020/level_0/scanner3DTop/",
    "Season 11": "/iplant/home/shared/phytooracle/season_11_sorghum_yr_2020/level_0/scanner3DTop/",
    "Season 12": "/iplant/home/shared/phytooracle/season_12_sorghum_soybean_sunflower_tepary_yr_2021/level_0/scanner3DTop/",
    "Season 13": "/iplant/home/shared/phytooracle/season_13_lettuce_yr_2022/level_0/scanner3DTop/",
    "Season 14": "/iplant/home/shared/phytooracle/season_14_sorghum_yr_2022/level_0/scanner3DTop/",
    "Season 15": "/iplant/home/shared/phytooracle/season_15_lettuce_yr_2022/level_0/scanner3DTop/"
}

def select_dir_path(dir_paths):
    root = tk.Tk()
    root.withdraw()

    selected_dir_path = None

    def on_select(event):
        nonlocal selected_dir_path
        selected_dir_path = dir_paths[event.widget.get()]

    dialog = tk.Toplevel(root)
    dialog.title("Select Season")
    
    # Set window size
    window_width = 900
    window_height = 500
    
    # Calculate x and y coordinates for the top-left corner of the window
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width / 2) - (window_width / 2)
    y = (screen_height / 2) - (window_height / 2)
    
    # Set window geometry
    dialog.geometry(f'{window_width}x{window_height}+{int(x)}+{int(y)}')

    label = tk.Label(dialog, text="Select a season:")
    label.pack()

    dir_path_var = tk.StringVar()
    dir_path_var.set(list(dir_paths.keys())[0])
    
    # Set the width of the drop-down menu
    dir_path_menu = ttk.Combobox(dialog, textvariable=dir_path_var, values=list(dir_paths.keys()), width=50)
    
    dir_path_menu.bind("<<ComboboxSelected>>", on_select)
    dir_path_menu.pack()

    button = tk.Button(dialog, text="OK", command=dialog.destroy)
    button.pack()

    root.wait_window(dialog)

    return selected_dir_path


dir_path = select_dir_path(dir_paths)

result = subprocess.run(["ils", dir_path], stdout=subprocess.PIPE)
output = result.stdout.decode("utf-8")

tar_files = [line.strip() for line in output.split("\n") if line.endswith(".tar.gz")]

def select_tar_file(tar_files):
    root = tk.Tk()
    root.withdraw()

    selected_tar_file = None

    def on_select(event):
        nonlocal selected_tar_file
        selected_tar_file = event.widget.get()

    dialog = tk.Toplevel(root)
    dialog.title("Select Tar File")
    
    # Set window size
    window_width = 900
    window_height = 500
    
    # Calculate x and y coordinates for the top-left corner of the window
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width / 2) - (window_width / 2)
    y = (screen_height / 2) - (window_height / 2)
    
    # Set window geometry
    dialog.geometry(f'{window_width}x{window_height}+{int(x)}+{int(y)}')

    label = tk.Label(dialog, text="Select a tar file:")
    label.pack()

    tar_file_var = tk.StringVar()
    tar_file_var.set(tar_files[0])
    
    # Set the width of the drop-down menu
    tar_file_menu = ttk.Combobox(dialog, textvariable=tar_file_var, values=tar_files, width=50)
    
    tar_file_menu.bind("<<ComboboxSelected>>", on_select)
    tar_file_menu.pack()

    button = tk.Button(dialog, text="OK", command=dialog.destroy)
    button.pack()

    root.wait_window(dialog)

    return selected_tar_file


selected_tar_file = select_tar_file(tar_files)
local_path = selected_tar_file.split(".")[0]

if not os.path.isdir(local_path):

    print(f'Downloading {select_tar_file}.')
    subprocess.run(["iget", "-PVT", os.path.join(dir_path, selected_tar_file)]) #, local_path])
    
    print(f'Extracting at {local_path}.')
    tar = tarfile.open(selected_tar_file, "r:gz")
    tar.extractall()
    tar.close()

def ransac_transform_and_get_inliers(args):

    source_down_points = args[0]
    target_down_points = args[1]
    tr_x = args[2]
    tr_y = args[3]
    tr_z = args[4]
    voxel_size = args[5]

    source_down = o3d.geometry.PointCloud() 
    source_down.points = o3d.utility.Vector3dVector(source_down_points)

    target_down = o3d.geometry.PointCloud() 
    target_down.points = o3d.utility.Vector3dVector(target_down_points)
    
    target_tree = o3d.geometry.KDTreeFlann(target_down)

    translated_source = copy.deepcopy(source_down).translate((tr_x,tr_y, tr_z))
    translated_points = translated_source.points
    
    number_matched_points = 0
    
    mins = np.min(translated_points,axis=0)
    maxs = np.max(translated_points,axis=0)

    for point in translated_points:

        [k, idx, _] = target_tree.search_radius_vector_3d(point, voxel_size)
        if len(idx)>1:
            number_matched_points+=1

    return tr_x,tr_y,tr_z,number_matched_points

def execute_manual_location_based_RANSAC(source_down,target_down,num_samples,voxel_size=1,coefs=[2,1,1,1]):

    tr_mean = (0,0,0)
    tr_sigma = (voxel_size*coefs[0],voxel_size*coefs[1],voxel_size*coefs[2])

    coords_x=np.random.normal(tr_mean[0],tr_sigma[0],num_samples)
    coords_y=np.random.normal(tr_mean[1],tr_sigma[1],num_samples)
    coords_z=np.random.normal(tr_mean[2],tr_sigma[2],num_samples)

    args = []

    for i,x in enumerate(coords_x):
        y = coords_y[i]
        z = coords_z[i]
        args.append((np.asarray(source_down.points),np.asarray(target_down.points),x,y,0,voxel_size*coefs[3]))

    processes = multiprocessing.Pool(multiprocessing.cpu_count()-2)
    results = processes.map(ransac_transform_and_get_inliers,args)
    processes.close()

    best_x = 0
    best_y = 0
    best_z = 0
    best_n = -1

    for x,y,z,n in results:
        if n>best_n:
            best_n = n
            best_x = x
            best_y = y
            best_z = z

    return (best_x,best_y,best_z)

def translate_pcd(pcd,x,y,z):
    transformed_pcd = copy.deepcopy(pcd).translate((x,y,z))
    return transformed_pcd

def merge_east_west_ransac(east,west,down_east,down_west):
    tr = execute_manual_location_based_RANSAC(down_east,down_west,400,coefs=[5,5,0.1,0.5])
    
    new_east_down = translate_pcd(down_east,tr[0],tr[1],tr[2])
    new_east = translate_pcd(east,tr[0],tr[1],tr[2])

    east_points_down = np.array(new_east_down.points)
    west_points_down = np.array(down_west.points)
    merged_down = o3d.geometry.PointCloud() 
    merged_down.points = o3d.utility.Vector3dVector(np.concatenate([east_points_down,west_points_down]))

    east_points = np.array(new_east.points)
    west_points = np.array(west.points)
    merged = o3d.geometry.PointCloud() 
    merged.points = o3d.utility.Vector3dVector(np.concatenate([east_points,west_points]))

    return merged,merged_down,new_east,new_east_down


def rotate_pcd(pcd ,rotation_theta=90, center_pcd=None):

    theta = np.radians(rotation_theta)

    if center_pcd is not None:
        min_x, min_y, min_z = center_pcd.get_min_bound()
        max_x, max_y, max_z = center_pcd.get_max_bound()

        center_x = abs(max_x - min_x)/2
        center_y = abs(max_y - min_y)/2
        center_z = abs(max_z - min_z)/2
    else:
        min_x, min_y, min_z = pcd.get_min_bound()
        max_x, max_y, max_z = pcd.get_max_bound()

        center_x = abs(max_x - min_x)/2
        center_y = abs(max_y - min_y)/2
        center_z = abs(max_z - min_z)/2

    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1]])

    rotated_pcd = pcd.rotate(rotation_matrix, center=[center_x, center_y, center_z])

    return rotated_pcd

def load_metadata_dict(path):
    
    with open(path) as f:
        meta = json.load(f)['lemnatec_measurement_metadata']

    return meta

def get_direction(x_position: float) -> str:
    if x_position < 216:
        return "south"
    else:
        return "north"

# Create an empty list to store each pair of point clouds
pcd_pairs = []
pcd_directions = []
fields = []
z_positions = []
filenames = []

for subdir, dirs, files in os.walk(local_path):
    dirs.sort()
    ply_files = [file for file in files if file.endswith(".ply")]
    json_files = [file for file in files if file.endswith(".json")]

    if len(ply_files) == 2:
        try:
            # Iterate over the items in ply_files
            for item in ply_files:

                # Check if the item contains "east"
                if "east" in item:
                    # Measure the time it takes to read the east point cloud
                    start = time.time()
                    east_pcd = o3d.io.read_point_cloud(os.path.join(subdir, item),
                                                       remove_nan_points=True,
                                                       remove_infinite_points=True)
                    end = time.time()
                    print(f'Time taken to read {item}: {end - start:.2f} seconds')

                    # Measure the time it takes to downsample the east point cloud
                    start = time.time()
                    down_east_pcd = copy.deepcopy(east_pcd).voxel_down_sample(voxel_size=15)
                    end = time.time()
                    print(f'Time taken to downsample {item}: {end - start:.2f} seconds')
                
                # Check if the item contains "west"
                elif "west" in item:
                    # Measure the time it takes to read the west point cloud
                    start = time.time()
                    west_pcd = o3d.io.read_point_cloud(os.path.join(subdir, item),
                                                       remove_nan_points=True,
                                                       remove_infinite_points=True)
                    end = time.time()
                    print(f'Time taken to read {item}: {end - start:.2f} seconds')

                    # Measure the time it takes to downsample the west point cloud
                    start = time.time()
                    down_west_pcd = copy.deepcopy(west_pcd).voxel_down_sample(voxel_size=15)
                    end = time.time()
                    print(f'Time taken to downsample {item}: {end - start:.2f} seconds')
            
                meta_path = os.path.join(subdir, '_'.join([item.split('__')[0], 'metadata.json']))

            print(f'Opening metadata file {meta_path}.')
            metadata = load_metadata_dict(meta_path)

            # Get filename
            filename = ''.join([subdir, item.split('__')[0]])

            # Get field location
            x_position = float(metadata['gantry_system_variable_metadata']['position x [m]'])
            field = get_direction(x_position)

            # Get box height
            z_position = float(metadata['gantry_system_variable_metadata']['position z [m]'])
            
            # Add filename, field locations, and box heights to lists
            filenames.append(filename)
            fields.append(field)
            z_positions.append(z_position)

            print('Rotating point clouds.')
            new_east_down = rotate_pcd(down_east_pcd,90) #,merged_down_pcd)
            new_west_down = rotate_pcd(down_west_pcd,90) #,merged_down_pcd)

            if metadata['gantry_system_variable_metadata']['scanIsInPositiveDirection'] == "False":

                new_east_down = new_east_down.translate([0,(float(metadata['gantry_system_variable_metadata']['position x [m]'])-3.798989)/(8.904483-7.964989)*1000,0])
                new_west_down = new_west_down.translate([0,(float(metadata['gantry_system_variable_metadata']['position x [m]'])-3.798989)/(8.904483-7.964989)*1000,0])
                pcd_directions.append("Negative")
            else:

                new_east_down = new_east_down.translate([22280.82692587,(float(metadata['gantry_system_variable_metadata']['position x [m]'])-3.798989)/(8.904483-7.964989)*1000,0])
                new_west_down = new_west_down.translate([22280.82692587,(float(metadata['gantry_system_variable_metadata']['position x [m]'])-3.798989)/(8.904483-7.964989)*1000,0])
                pcd_directions.append("Positive")

            # store the pair of point clouds for later use

            print("Appending point cloud pair to list")
            pcd_pairs.append((new_east_down, new_west_down))
            print("Appended point cloud pair to list")

            del metadata, east_pcd, down_east_pcd, new_east_down, west_pcd, down_west_pcd, new_west_down

        except Exception as e:
            print(f"An error occurred while processing {ply_files[0]} and {ply_files[1]}: {e}")

def save_transformation(vis, source, target, transformations):
    trans_init = source.get_rotation_matrix_from_xyz((0, 0, 0)) @ np.linalg.inv(target.get_rotation_matrix_from_xyz((0, 0, 0)))
    transformations.append(trans_init)
    vis.clear_geometries()
    vis.register_animation_callback(None)
    vis.poll_events()
    vis.update_renderer()
    vis.destroy_window()
    del source
    del target

def update_view(vis, highest_point):

    ctr = vis.get_view_control()
    ctr.set_lookat(highest_point)
    return False

def update_visualization(vis, source):
    vis.update_geometry(source)
    vis.poll_events()
    vis.update_renderer()

def move_left(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[0,3] -= size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)

def move_right(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[0,3] += size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)

def move_up(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[1,3] += size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)

def move_down(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[1,3] -= size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)

def move_forward(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[2,3] -= size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)

def move_backward(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[2,3] += size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)

def next_pair(vis):
    # Stop the visualization
    # vis.register_animation_callback(None)
    # vis.poll_events()
    # vis.update_renderer()
    # vis.destroy_window()
    vis.clear_geometries()
    vis.close()

def save_transform_and_move_to_next_pair(vis,cumulative_transform,list_of_transforms):
    global e_pressed
    e_pressed = True
    print('Saving cumulative transformation')
    list_of_transforms.append(cumulative_transform)
    print('Saved cumulative transformation')
    
def close_window(vis):
    if e_pressed:
        print('Closing window.')
        # vis.destroy_window()
        vis.clear_geometries()
        vis.close()
        print('Window closed')
    else:
        print("Please press 'E' before quitting")

# Step 1: Align the point clouds within each pair (EW)
ew_final_transformations = []
for i, (source, target) in enumerate(pcd_pairs):
    
    e_pressed = False

    # Create a copy of source
    source_copy = copy.deepcopy(source)
    
    # Paint the point clouds for visualization
    print('Preparing point cloud pair')
    source.paint_uniform_color([1, 0.706, 0])
    source_copy.paint_uniform_color([1, 0.706, 0])
    target.paint_uniform_color([0, 0.651, 0.929])  

    # Set up the visualization
    print("Press 'W', 'A', 'S', 'D', 'R', or 'F' to move the source point cloud")
    print("Press 'E' to save transformation")
    print("Press 'Q' to move to next pair")
    print("Press 'I' to ignore this pair and move to the next pair")

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.toggle_full_screen()
    # Application.instance.run()

    # Add point clouds
    vis.add_geometry(source_copy)
    vis.add_geometry(target)

    cum_trans = np.eye(4)

    # Register key callbacks to move point cloud along the x-axis
    vis.register_key_callback(ord("W"), lambda vis: move_up(vis, source_copy, size=5))
    vis.register_key_callback(ord("A"), lambda vis: move_left(vis, source_copy, size=5))
    vis.register_key_callback(ord("S"), lambda vis: move_down(vis, source_copy, size=5))
    vis.register_key_callback(ord("D"), lambda vis: move_right(vis, source_copy, size=5))
    vis.register_key_callback(ord("R"), lambda vis: move_forward(vis, source_copy, size=5))
    vis.register_key_callback(ord("F"), lambda vis: move_backward(vis, source_copy, size=5))
    vis.register_key_callback(ord("I"), lambda vis: next_pair(vis))
    vis.register_key_callback(ord("E"), lambda vis: save_transform_and_move_to_next_pair(vis,cum_trans,ew_final_transformations))
    vis.register_key_callback(ord("Q"), close_window)

    # Run and destroy the visualization
    vis.poll_events()
    vis.run()
    vis.destroy_window()

    # Delete or reassign variables that are no longer needed
    del source_copy

# Calculate the final transformation based on all transformations
ew_final_transformation = np.mean(ew_final_transformations,axis=0)
np.save('ew_final_transformation.npy', ew_final_transformation)

# Apply final transformation to each source point cloud in pcd_pairs
for i, (source, target) in enumerate(pcd_pairs):
    source.transform(ew_final_transformation)

# Create a list of all point clouds
all_point_clouds = []
for i, (source, target) in enumerate(pcd_pairs):
    all_point_clouds.append(source)
    all_point_clouds.append(target)

# Visualize all point cloud pairs in a single visualization
o3d.visualization.draw_geometries(all_point_clouds, window_name='Final EW transformation')

del all_point_clouds, cum_trans

# Step 2: Align pairs to each other (NS)
merged_point_clouds = []
for i in range(len(pcd_pairs)):
    source = pcd_pairs[i][0]
    target = pcd_pairs[i][1]
    merged_point_cloud = source + target
    voxel_size = 15 # Set the voxel size for downsampling
    downsampled_merged_point_cloud = o3d.geometry.PointCloud.voxel_down_sample(merged_point_cloud, voxel_size)
    merged_point_clouds.append(downsampled_merged_point_cloud)

del pcd_pairs

ns_final_transformations = []
for i in range(len(merged_point_clouds)-1):
    if pcd_directions[i] == "Positive":
        e_pressed = False

        source = merged_point_clouds[i]
        target = merged_point_clouds[i+1]
        source_copy = copy.deepcopy(source)

        # Paint the point clouds for visualization
        print('Preparing point cloud pair')
        source.paint_uniform_color([1, 0.706, 0])
        source_copy.paint_uniform_color([1, 0.706, 0])
        target.paint_uniform_color([0, 0.651, 0.929])  
        
        # Set up the visualization
        print("Press 'W', 'A', 'S', 'D', 'R', or 'F' to move the source point cloud")
        print("Press 'E' to save transformation")
        print("Press 'Q' to move to next pair")
        print("Press 'I' to ignore this pair and move to the next pair")

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        vis.toggle_full_screen()

        # Add point clouds
        vis.add_geometry(source_copy)
        vis.add_geometry(target)

        cum_trans = np.eye(4)

        # Register key callbacks to move point cloud along the x-axis
        vis.register_key_callback(ord("W"), lambda vis: move_up(vis, source_copy, size=20))
        vis.register_key_callback(ord("A"), lambda vis: move_left(vis, source_copy, size=20))
        vis.register_key_callback(ord("S"), lambda vis: move_down(vis, source_copy, size=20))
        vis.register_key_callback(ord("D"), lambda vis: move_right(vis, source_copy, size=20))
        vis.register_key_callback(ord("R"), lambda vis: move_forward(vis, source_copy, size=20))
        vis.register_key_callback(ord("F"), lambda vis: move_backward(vis, source_copy, size=20))
        vis.register_key_callback(ord("I"), lambda vis: next_pair(vis))
        vis.register_key_callback(ord("E"), lambda vis: save_transform_and_move_to_next_pair(vis,cum_trans,ns_final_transformations))
        vis.register_key_callback(ord("Q"), close_window)

        # Run and destroy the visualization
        vis.poll_events()
        vis.run()
        vis.destroy_window()

# Calculate the final transformation based on all transformations
ns_final_transformation = np.mean(ns_final_transformations,axis=0)
np.save('ns_final_transformation.npy', ns_final_transformation)

# Apply final transformation to each target point cloud in pcd_pairs
for i in range(len(merged_point_clouds)):
    # if i % 2 == 0:
    if pcd_directions[i] == "Positive":
        print(f'i: {i}')
        merged_point_clouds[i].transform(ns_final_transformation)

# Visualize all merged point clouds in a single visualization
o3d.visualization.draw_geometries(merged_point_clouds, window_name='Final transformation')

del merged_point_clouds

with h5py.File(f'{local_path}_transformations.h5', 'w') as f:
    ew_grp = f.create_group('EW')
    ew_individual_grp = ew_grp.create_group('individual')
    for i, transformation in enumerate(ew_final_transformations):
        ew_individual_grp.create_dataset(f'transformation_{i}', data=transformation)
    ew_individual_grp.create_dataset('fields', data=fields)
    ew_individual_grp.create_dataset('z_positions', data=z_positions)
    ew_individual_grp.create_dataset('filenames', data=filenames, dtype=h5py.special_dtype(vlen=str))
    
    ew_average_grp = ew_grp.create_group('average')
    ew_average_grp.create_dataset('transformation', data=ew_final_transformation)
    
    ns_grp = f.create_group('NS')
    ns_individual_grp = ns_grp.create_group('individual')
    for i, transformation in enumerate(ns_final_transformations):
        ns_individual_grp.create_dataset(f'transformation_{i}', data=transformation)
    ns_individual_grp.create_dataset('fields', data=fields)
    ns_individual_grp.create_dataset('z_positions', data=z_positions)
    ns_individual_grp.create_dataset('filenames', data=filenames, dtype=h5py.special_dtype(vlen=str))

    ns_average_grp = ns_grp.create_group('average')
    ns_average_grp.create_dataset('transformation', data=ns_final_transformation)
# with h5py.File('transformations.h5', 'w') as f:
#     ew_grp = f.create_group('EW')
#     ew_individual_grp = ew_grp.create_group('individual')
#     for i, transformation in enumerate(ew_final_transformations):
#         ew_individual_grp.create_dataset(f'transformation_{i}', data=transformation)
#     ew_individual_grp.create_dataset('fields', data=fields)
#     ew_individual_grp.create_dataset('z_positions', data=z_positions)
#     ew_individual_grp.create_dataset('filenames', data=filenames, dtype=h5py.special_dtype(vlen=str))
    
#     ew_average_grp = ew_grp.create_group('average')
#     ew_average_grp.create_dataset('transformation', data=ew_final_transformation)
    
#     ns_grp = f.create_group('NS')
#     ns_individual_grp = ns_grp.create_group('individual')
#     for i, transformation in enumerate(ns_final_transformations):
#         ns_individual_grp.create_dataset(f'transformation_{i}', data=transformation)
#     ns_individual_grp.create_dataset('fields', data=fields)
#     ns_individual_grp.create_dataset('z_positions', data=z_positions)
#     ns_individual_grp.create_dataset('filenames', data=filenames, dtype=h5py.special_dtype(vlen=str))
    
#     ns_average_grp = ns_grp.create_group('average')
#     ns_average_grp.create_dataset('transformation', data=ns_final_transformation)