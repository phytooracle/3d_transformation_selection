#!/usr/bin/env python3
"""
Author : eg
Date   : 2023-06-14
Purpose: Rock the Casbah
"""
import subprocess
import tkinter as tk
from tkinter import simpledialog
from tkinter import ttk
from tkinter import filedialog
import tarfile
import os
import open3d as o3d
from open3d.visualization import VisualizerWithKeyCallback
import numpy as np
import csv
import copy
from plyfile import PlyData
import time
import pandas as pd
import multiprocessing


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
    dialog.title("Select Directory")
    dialog.geometry("500x300")

    label = tk.Label(dialog, text="Select a directory:")
    label.pack()

    dir_path_var = tk.StringVar()
    dir_path_var.set(list(dir_paths.keys())[0])
    dir_path_menu = ttk.Combobox(dialog, textvariable=dir_path_var, values=list(dir_paths.keys()))
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
    dialog.geometry("500x300")

    label = tk.Label(dialog, text="Select a tar file:")
    label.pack()

    tar_file_var = tk.StringVar()
    tar_file_var.set(tar_files[0])
    tar_file_menu = ttk.Combobox(dialog, textvariable=tar_file_var, values=tar_files)
    tar_file_menu.bind("<<ComboboxSelected>>", on_select)
    tar_file_menu.pack()

    button = tk.Button(dialog, text="OK", command=dialog.destroy)
    button.pack()

    root.wait_window(dialog)

    return selected_tar_file

selected_tar_file = select_tar_file(tar_files)

local_path = selected_tar_file.split(".")[0]

if not os.path.isfile(selected_tar_file):

    print(f'Downloading {select_tar_file}.')
    subprocess.run(["iget", "-PVT", os.path.join(dir_path, selected_tar_file)]) #, local_path])


if not os.path.isdir(local_path):
    
    print(f'Extracting {select_tar_file} at {local_path}.')
    tar = tarfile.open(selected_tar_file, "r:gz")
    tar.extractall()
    tar.close()

def open_ply(filepath):
    # Read the PLY file into a numpy array
    plydata = PlyData.read(filepath)
    vertex = plydata['vertex']
    x = vertex['x']
    y = vertex['y']
    z = vertex['z']
    xyz = np.column_stack((x, y, z))

    # Pass the numpy array to Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # Return the Open3D point cloud object
    return pcd

def pick_points(pcd):
    # Convert the point cloud to a numpy array
    points = np.asarray(pcd.points)

    # Calculate the centroid of the points with the maximum z-coordinate
    max_z = np.max(points[:, 2])
    tallest_points = points[points[:, 2] == max_z]
    centroid = np.mean(tallest_points, axis=0)

    # Calculate the bounding box of the point cloud
    bbox = pcd.get_axis_aligned_bounding_box()
    bbox_dim = bbox.get_extent()

    # Calculate an appropriate radius for the sphere based on the bounding box dimensions
    sphere_radius = np.min(bbox_dim) * 0.05

    # Create a sphere marker at the location of the centroid
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
    sphere.translate(centroid)
    sphere.paint_uniform_color([1, 0, 0])

    # Set up a visualization window
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.add_geometry(sphere)

    # Get the current view control
    view_control = vis.get_view_control()

    # Set the camera lookat to the centroid of the tallest points
    view_control.set_lookat(centroid)

    # Prompt the user to select a point or press "Escape" to cancel
    print("Select a point or press 'Escape' to cancel.")

    # Run the visualization
    vis.run()
    vis.destroy_window()

    if len(vis.get_picked_points()) == 0:
        return centroid
    else:
        return vis.get_picked_points() #points[vis.get_picked_points()[0]]

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
# Create an empty list to store each pair of point clouds
pcd_pairs = []

# Create an empty list to store the transformation and filenames for each pair of point clouds
transformations_and_filenames = []

for subdir, dirs, files in os.walk("."):
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
                    down_east_pcd = copy.deepcopy(east_pcd).voxel_down_sample(voxel_size=1)
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
                    down_west_pcd = copy.deepcopy(west_pcd).voxel_down_sample(voxel_size=1)
                    end = time.time()
                    print(f'Time taken to downsample {item}: {end - start:.2f} seconds')
            # metadata = load_metadata_dict(json_files[0])
            print('Rotating point clouds.')
            # merged_pcd,merged_down_pcd,new_east,new_east_down = merge_east_west_ransac(east_pcd,west_pcd,down_east_pcd,down_west_pcd)
            # new_east = rotate_pcd(new_east,90,merged_down_pcd)
            # new_west = rotate_pcd(west_pcd,90,merged_down_pcd)
            new_east_down = rotate_pcd(down_east_pcd,90) #,merged_down_pcd)
            new_west_down = rotate_pcd(down_west_pcd,90) #,merged_down_pcd)
            
            # merged_down_pcd = rotate_pcd(merged_down_pcd,90)
            # merged_pcd = rotate_pcd(merged_pcd,90)
            
            # select a point in each point cloud
            picked_points1 = pick_points(new_east_down)
            picked_points2 = pick_points(new_west_down)

            if len(picked_points1) == 0 or len(picked_points2) == 0:
                raise ValueError("No point was selected in one of the point clouds")

            point1 = np.asarray(new_east_down.points)[picked_points1[0]]
            point2 = np.asarray(new_west_down.points)[picked_points2[0]]

            # calculate the transformation to align these two points
            transformation = np.eye(4)
            transformation[:3, 3] = point1 - point2

            # transform pcd2 to align with pcd1
            new_east_down.transform(transformation)

        except Exception as e:
            print(f"An error occurred while processing {ply_files[0]} and {ply_files[1]}: {e}")

        # store the pair of point clouds for later use
        pcd_pairs.append((new_east_down, new_west_down))

        # store the transformation and filenames for this pair of point clouds
        transformations_and_filenames.append({
            "transformation": transformation,
            "filenames": (ply_files[0], ply_files[1])
        })

# Create a DataFrame from the transformations_and_filenames list
df = pd.DataFrame([{
    "filename1": item["filenames"][0],
    "filename2": item["filenames"][1],
    **{f"t{i}": t for i, t in enumerate(item["transformation"].flatten())}
} for item in transformations_and_filenames])

# Specify the filename of the CSV file
csv_filename = "ew_transformation.csv"

# Save the DataFrame as a CSV file
df.to_csv(csv_filename, index=False)

# Set up a visualization window
vis = VisualizerWithKeyCallback()
vis.create_window()

# Add all point clouds in pcd_pairs to the visualization
for pcd1, pcd2 in pcd_pairs:
    vis.add_geometry(pcd1)
    vis.add_geometry(pcd2)

# Create an empty list to store the transformations and names
transformations_and_names = []

# Define a callback function to handle key events
def handle_key_event(vis, event, key):
    shift_distance = 0.5

    if key == ord("A"):
        # If the user pressed "A", shift every other pair of point clouds to the left
        for i, (pcd1, pcd2) in enumerate(pcd_pairs):
            if i % 2 == 0:
                transformation = np.eye(4)
                transformation[0, 3] -= shift_distance
                pcd1.transform(transformation)
                pcd2.transform(transformation)

                # Store the transformation and names in the transformations_and_names list
                transformations_and_names.append({
                    "transformation": transformation,
                    "names": (pcd1.get_geometry_name(), pcd2.get_geometry_name())
                })
    elif key == ord("D"):
        # If the user pressed "D", shift every other pair of point clouds to the right
        for i, (pcd1, pcd2) in enumerate(pcd_pairs):
            if i % 2 == 0:
                transformation = np.eye(4)
                transformation[0, 3] += shift_distance
                pcd1.transform(transformation)
                pcd2.transform(transformation)

                # Store the transformation and names in the transformations_and_names list
                transformations_and_names.append({
                    "transformation": transformation,
                    "names": (pcd1.get_geometry_name(), pcd2.get_geometry_name())
                })

# Register the key event callback function
vis.register_key_callback(ord("A"), handle_key_event)
vis.register_key_callback(ord("D"), handle_key_event)

# Run the visualization
vis.run()
vis.destroy_window()

# Save the transformations and names as a CSV file using pandas
df = pd.DataFrame([{
    "name1": item["names"][0],
    "name2": item["names"][1],
    **{f"t{i}": t for i, t in enumerate(item["transformation"].flatten())}
} for item in transformations_and_names])

csv_filename = "ns_transformation.csv"
df.to_csv(csv_filename, index=False)
