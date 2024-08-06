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
import numpy as np
import copy
import time
import pandas as pd
import multiprocessing
import json
from open3d.visualization.gui import Application
import h5py
import shutil
import matplotlib.pyplot as plt
import re
from datetime import datetime, timedelta
import subprocess as sp
import utm
import glob
from multiprocessing import Pool

sys.setrecursionlimit(10000)

#-------------------------------------------------------------------------------
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


#-------------------------------------------------------------------------------
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


#-------------------------------------------------------------------------------
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


#-------------------------------------------------------------------------------
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


#-------------------------------------------------------------------------------
def translate_pcd(pcd,x,y,z):
    transformed_pcd = copy.deepcopy(pcd).translate((x,y,z))
    return transformed_pcd


#-------------------------------------------------------------------------------
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


#-------------------------------------------------------------------------------
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


#-------------------------------------------------------------------------------
def load_metadata_dict(path):
    
    with open(path) as f:
        meta = json.load(f)['lemnatec_measurement_metadata']

    return meta


#-------------------------------------------------------------------------------
def get_direction(x_position: float) -> str:
    if x_position < 216:
        return "south"
    else:
        return "north"


#-------------------------------------------------------------------------------
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


#-------------------------------------------------------------------------------
def update_view(vis, highest_point):

    ctr = vis.get_view_control()
    ctr.set_lookat(highest_point)
    return False


#-------------------------------------------------------------------------------
def update_visualization(vis, source):
    vis.update_geometry(source)
    vis.poll_events()
    vis.update_renderer()


#-------------------------------------------------------------------------------
def move_left(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[0,3] -= size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)
    # print(cum_trans)


#-------------------------------------------------------------------------------
def move_right(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[0,3] += size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)
    # print(cum_trans)


#-------------------------------------------------------------------------------
def move_up(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[1,3] += size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)
    # print(cum_trans)


#-------------------------------------------------------------------------------
def move_down(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[1,3] -= size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)
    # print(cum_trans)


#-------------------------------------------------------------------------------
def move_forward(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[2,3] -= size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)
    # print(cum_trans)


#-------------------------------------------------------------------------------
def move_backward(vis, source, size=1):
    global cum_trans
    trans = np.eye(4)
    trans[2,3] += size
    cum_trans = np.dot(trans,cum_trans)
    source.transform(trans)
    update_visualization(vis, source)
    # print(cum_trans)


#-------------------------------------------------------------------------------
def next_pair(vis):

    vis.clear_geometries()
    vis.close()


#-------------------------------------------------------------------------------
def save_transform_and_move_to_next_pair(vis,cumulative_transform,list_of_transforms, index, i):

    global e_pressed
    e_pressed = True
    print(f'Saving cumulative transformation\n{cumulative_transform}')
    list_of_transforms.append(cumulative_transform)
    print('Saved cumulative transformation')
    index.append(i)
    next_pair(vis)


#-------------------------------------------------------------------------------
def close_window(vis):
    if e_pressed:
        print('Closing window.')
        # vis.destroy_window()
        vis.clear_geometries()
        vis.close()
        print('Window closed')
    else:
        print("Please press 'E' before quitting")


#-------------------------------------------------------------------------------
def remove_directory(dir_path):

    try:
        shutil.rmtree(dir_path)
        print(f'Successfully removed {dir_path}.')
    except Exception as e:
        print(f'Error removing {dir_path}: {e}')


#-------------------------------------------------------------------------------
def remove_file(file_path):
    try:
        os.remove(file_path)
        print(f'Successfully removed {file_path}')
    except Exception as e:
        print(f'Error removing {file_path}: {e}')


#-------------------------------------------------------------------------------
def colorize_point_cloud(pcd, cmap):

    # Get the points as a numpy array
    points = np.asarray(pcd.points)

    # Calculate the colors based on the z values of the points
    colors = (points[:, 2] - np.min(points[:, 2])) / (np.max(points[:, 2]) - np.min(points[:, 2]))
    colors = plt.get_cmap(cmap)(colors)[:, :3]

    # Set the colors of the point cloud
    pcd.colors = o3d.utility.Vector3dVector(colors)


#-------------------------------------------------------------------------------
def extract_date(string: str) -> str:
    match = re.search(r'\d{4}-\d{2}-\d{2}', string)
    if match:
        return match.group()
    else:
        return "No date found in the string"


#-------------------------------------------------------------------------------
def get_env_dates(date_string):
    '''
    Given a date string, the function finds two flanking dates (+/- 1 day). 
    
    Input:
        - date_string: String containing the date in yyyy-MM-dd format
    Output: 
        - List containing the date from date_string variable in addition to the two flanking dates (+/- 1 day)
    '''

    match = re.search(r'\d{4}-\d{2}-\d{2}', date_string)

    if match:
        date_str = match.group()
        date = datetime.strptime(date_str, '%Y-%m-%d').date()
        day_before = date - timedelta(days=1)
        day_after = date + timedelta(days=1)
    
        return [day_before, date, day_after]


#-------------------------------------------------------------------------------
def get_file_list(data_path, sequence):
    '''
    Using the dictionary containing season, level, and sensor notations, this function finds all files matching the season, 
    level, and sensor paths, as well as an identifying sequence such as %.tar.gz. The % is similar to Linux's wild card "*"
    
    Input:
        - data_path: The CyVerse Data Store path created from dictionary
        - sequence: An identifying sequence, such as "%.tar.gz". The "%" character is similar to Linux's wild card "*" character.
    Output: 
        - List of files matching the season, level, sensor, and sequence
    '''
    result = sp.run(f'ilocate {os.path.join(data_path, "%", f"{sequence}")}', stdout=sp.PIPE, shell=True)
    files = result.stdout.decode('utf-8').split('\n')

    return files


#-------------------------------------------------------------------------------
def download_files(item, out_path):
    '''
    Uses iRODS to access the CyVerse Data Store. The function downloads data and extracts contents from ".tar" and "tar.gz" files if applicable.
    
    Input:
        - item: The list of CyVerse file paths to download locally.
        - out_path: Output directory where data will be saved. 
    Output: 
        - Downloaded and extracted data within the specified output directory 
    '''
    cwd = os.getcwd()
    os.chdir(out_path)

    if not 'deprecated' in item:

        try:
            item = os.path.normpath(item)

            try:

                match_str = re.search(r'\d{4}-\d{2}-\d{2}__\d{2}-\d{2}-\d{2}-\d{3}', item)
                date = match_str.group()
                # date = datetime.strptime(match_str.group(), '%Y-%m-%d').date()
            except:
                match_str = re.search(r'\d{4}-\d{2}-\d{2}', item)
                date = datetime.strptime(match_str.group(), '%Y-%m-%d').date()
                date = str(date)

            print(f"Found item {item}.")

            if not os.path.exists(os.path.join(cwd, out_path, date)):

                if not os.path.isdir(date):
                    print(f"Making directory {date}.")
                    os.makedirs(date)

                print(f"Downloading {item}.")
                sp.call(f'iget -KPVT {item}', shell=True)

                if '.tar.gz' in item: 
                    # print(f"Downloading {item}.")
                    # sp.call(f'iget -KPVT {item}', shell=True)

                    print(f"Extracting {item}.")
                    ret = sp.call(f'tar -xzvf {os.path.basename(item)} -C {date}', shell=True)
                    # ret = sp.call(f'tar -c --use-compress-program=pigz -f {os.path.basename(item)}', shell=True) #-C {date} 

                    if ret != 0:
                        print(f"Reattempting to extract {item}.")
                        sp.call(f'tar -xvf {os.path.basename(item)} -C {date}', shell=True)

                    sp.call(f'rm {os.path.basename(item)}', shell=True)

                elif '.tar' in item:
                    # print(f"Downloading {item}.")
                    # sp.call(f'iget -KPVT {item}', shell=True)
                    
                    print(f"Extracting {item}.")
                    sp.call(f'tar -xvf {os.path.basename(item)} -C {date}', shell=True)
                    sp.call(f'rm {os.path.basename(item)}', shell=True)

                else:
                    os.chdir(date)
                    sp.call(f'iget -KPVT {item}', shell=True)

            else:
                print(f'{os.path.basename(item)} already exists. Skipping download.')

        except:
            pass


#-------------------------------------------------------------------------------
def get_dict():
    '''
    Provides notation for CyVerse directories. 
    
    Input:
        - NA
    Output: 
        - A dictionary containing season, level, and sensor notations which will be used to query the CyVerse Data Store. 
    '''

    irods_dict = {
        'server_path': '/iplant/home/shared/phytooracle/',

        'season': {
            '10': 'season_10_lettuce_yr_2020',
            '11': 'season_11_sorghum_yr_2020',
            '12': 'season_12_sorghum_soybean_sunflower_tepary_yr_2021',
            '13': 'season_13_lettuce_yr_2022',
            '14': 'season_14_sorghum_yr_2022',
            '15': 'season_15_lettuce_yr_2022',
            '16': 'season_16_sorghum_yr_2023',
            '17': 'season_17_lettuce_yr_2023',
            '18': 'season_18_sorghum_yr_2024'
        },

        'level': {
            '0': 'level_0', 
            '1': 'level_1',
            '2': 'level_2',
            '3': 'level_3',
            '4': 'level_4'
        },

        'sensor': {
            'FLIR': 'flirIrCamera',
            'PS2': 'ps2Top',
            'RGB': 'stereoTop',
            '3D': 'scanner3DTop',
            'ENV': 'EnvironmentLogger'
        }
    }

    return irods_dict


#-------------------------------------------------------------------------------
def download_data(crop, season, level, sensor, sequence, cwd, outdir, download=True):
    '''
    Recursively runs `download_files` to download all data into a single output directory specified by the user.
    
    Input:
        - crop: The name of the crop data to download, either "lettuce" or "sorghum"
        - season: The season numer to download, either 14, 15, or 16
        - level: The level of data to download, either 0, 1, 2
        - sensor: The name of the sensor to download, either RGB, FLIR, PS2, or 3D
        - sequence: The identifying sequence to download, such as ".tar" or ".tar.gz"
        - cwd: The current working directory
        - outdir: The output directory
        - download: Boolean value to specify whether to download data (True) or not (False)

    Output: 
        - Downloaded and extracted data within the specified output directory 
    '''

    try:

        irods_dict = get_dict()
        # Create iRODS path from components. 
        data_path = os.path.join(irods_dict['server_path'], irods_dict['season'][season], irods_dict['level'][level], irods_dict['sensor'][sensor])
        if crop != "NA":
            data_path = os.path.join(irods_dict['server_path'], irods_dict['season'][season], irods_dict['level'][level], irods_dict['sensor'][sensor], crop)
        # Get list of all files that match a character sequence.
        print(f'Searching for files matching "{os.path.join(data_path, sequence)}". Note: This process may take 1-5 minutes.')
        files = get_file_list(data_path, sequence)
        print(f'Matches obtained: {files}')

        # Prepare to download data.
        out_path = os.path.join(outdir, irods_dict['season'][season], irods_dict['sensor'][sensor])
        
        if not os.path.isdir(out_path):
            os.makedirs(out_path)

        if download:
            os.chdir(out_path)

            # Download files.
            for item in files: 
                print(f'Downloading {item}.')
                download_files(item=item, out_path=os.path.join(cwd, out_path))
            # for item in files:
            #     if not os.path.exists(os.path.join(cwd, out_path, item)):
            #         print(f'Downloading {item}.')
            #         download_files(item=item, out_path=os.path.join(cwd, out_path))
            #     else:
            #         print(f'{item} already exists. Skipping download.')

            # os.chdir(cwd)

        return out_path
    
    except Exception as e:
        # code to handle the exception
        print(f"An error occurred while downloading data: {e}")


#-------------------------------------------------------------------------------
def utm_to_latlon(utm_x, utm_y):
    '''
    Convert coordinates from UTM 12N to lat/lon

    Input:
        - utm_x: UTM coordinate for gantry x position
        - utm_y: UTM coordinate for gantry y position
    Output: 
        - Latitude and longitude for the provided x, y gantry positions
    '''

    # Get UTM information from southeast corner of field
    SE_utm = utm.from_latlon(33.07451869, -111.97477775)
    utm_zone = SE_utm[2]
    utm_num  = SE_utm[3]

    return utm.to_latlon(utm_x, utm_y, utm_zone, utm_num)


#-------------------------------------------------------------------------------
def scanalyzer_to_latlon(gantry_x, gantry_y):
    '''
    Convert coordinates from gantry to lat/lon

    Input:
        - gantry_x: Raw gantry x position
        - gantry_y: Raw gantry y position
    Output: 
        - Latitude and longitude for the provided x, y gantry positions
    '''

    utm_x, utm_y = scanalyzer_to_utm(gantry_x, gantry_y)
    return utm_to_latlon(utm_x, utm_y)


#-------------------------------------------------------------------------------
def scanalyzer_to_utm(gantry_x, gantry_y):
    '''
    Convert coordinates from gantry to UTM 12N
    
    Input:
        - gantry_x: Raw gantry x position
        - gantry_y: Raw gantry y position
    Output: 
        - Easting and northing for the provided x, y gantry positions
    '''

    # TODO: Hard-coded
    # Linear transformation coefficients
    ay = 3659974.971; by = 1.0002; cy = 0.0078;
    ax = 409012.2032; bx = 0.009; cx = - 0.9986;

    utm_x = ax + (bx * gantry_x) + (cx * gantry_y)
    utm_y = ay + (by * gantry_x) + (cy * gantry_y)

    return utm_x, utm_y


#-------------------------------------------------------------------------------
def get_date_position(data_path):
    '''
    Downloads gantry raw data (level_0) and extracts position and timestamps for each data capture/acquisition. 

    Input:
        - data_path: Path containing the raw data (level_0)
    Output: 
        - Dataframe containing time, x position, y position, z positon, latitude, and longitude for each data capture/acquisition
    '''

    # Create empty dataframe and set up counter
    df = pd.DataFrame(columns=['time', 'x_position', 'y_position', 'z_position', 'latitude', 'longitude'])
    cnt = 0

    try:
        # Iterate through each metadata JSON file
        for jfile in glob.glob(data_path):
            
            # Update counter
            cnt += 1

            # Open JSON file
            with open(jfile) as f:
    
                data = json.load(f)['lemnatec_measurement_metadata']
                
                # Extract time, x gantry position, and y gantry position
                time = pd.to_datetime(data['gantry_system_variable_metadata']['time'])
                x_position = float(data['gantry_system_variable_metadata']['position x [m]'])
                y_position = float(data['gantry_system_variable_metadata']['position y [m]'])
                z_position = float(data['gantry_system_variable_metadata']['position z [m]'])

                # Apply offsets
                offset_x = -1.035
                x_position = x_position + offset_x

                offset_y = 1.684
                y_position = y_position + offset_y
                
                offset_z = 0.856
                z_position = z_position + offset_z


                # Convert gantry coordinate to latitude, longitude
                lat, lon = scanalyzer_to_latlon(x_position, y_position)

                # Save data to a dataframe
                df.loc[cnt] = [time, x_position, y_position, z_position, lat, lon]

        # Sort dataframe by time
        df = df.sort_values('time')

        # Add capture sequence numbers
        df['capture_sequence'] = df['time'].argsort()

    except:

        pass
    
    return df


#-------------------------------------------------------------------------------
def process_file(jfile):
    '''
    Processes a single Environmental Logger JSON file, correcting the timestamp and extracting the necessary environmental parameters. 
    
    Input:
        - jfile: Path to a single Environmental Logger JSON file
    Output: 
        - Dataframe containing corrected timestamp formats and environmental parameters from "environment_sensor_readings," including "timestamp," 
          "weather_station," and "sensor par." The "timestamp" value is in format yyyy.MM.dd-HH:mm:ss, which will be converted in subsequent functions. 
          The "weather_station values include sunDirection (degrees), airPressure (hPa), brightness (kilo Lux), relHumidity (relHumPerCent), temperature
          (DegCelsius), windDirection (degrees), precipitation (mm/h), windVelocity (m/s). The "sensor par" value includes photosynthetic active radiation (umol/(m^2*s)).  
    '''
        
    dfs = []

    try:
        with open(jfile) as f:
            data = json.load(f)
            for item in data['environment_sensor_readings']:
                # Convert to appropriate datetime format
                date = pd.to_datetime(item['timestamp'], format="%Y.%m.%d-%H:%M:%S")
                
                # Create a dataframe from the data
                data = {key: float(value['value']) for key, value in item['weather_station'].items()} #f"{key}_{value['unit']}"
                df = pd.DataFrame(data, index=[0])

                # Add datetime to the dataframe
                df['time'] = date

                # Add PAR to df 
                df['par'] = float(item['sensor par']['value'])
                
                dfs.append(df)

    except:
        
        print("An error occurred while reading the JSON file or processing the data.")
        dfs = []

    return dfs


#-------------------------------------------------------------------------------
def get_environment_df(data_path):
    '''
    Uses multiprocessing to run the function `process_file` to extract multiple Environmental Logger JSON files and combine them into a single dataframe. 
    
    Input:
        - data_path: Path containing the raw data (level_0)
    Output: 
        - Merged dataframe containing the timestamp and environmental parameters from multiple Environmental Logger JSON files
    '''
        
    with Pool() as pool:
        results = pool.map(process_file, glob.glob(data_path))
    dfs = [df for result in results for df in result]
    # Combine all dataframes in the list into one
    env_df = pd.concat(dfs, ignore_index=True)
    return env_df.sort_values('time')


#-------------------------------------------------------------------------------
def custom_sort(file, all_dirs):
    # extract the directory name from the file path
    directory = os.path.dirname(file)
    # find the index of the directory in the all_dirs list
    try:
        index = all_dirs.index(directory)
    except ValueError:
        # return a default value for files without a parent directory
        index = -1
    return index


#-------------------------------------------------------------------------------
# Initialize Tkinter
root = tk.Tk()
root.withdraw()

# Define season directory paths
dir_paths = {
    "Season 10": "/iplant/home/shared/phytooracle/season_10_lettuce_yr_2020/level_0/scanner3DTop/",
    "Season 11": "/iplant/home/shared/phytooracle/season_11_sorghum_yr_2020/level_0/scanner3DTop/",
    "Season 12": "/iplant/home/shared/phytooracle/season_12_sorghum_soybean_sunflower_tepary_yr_2021/level_0/scanner3DTop/",
    "Season 13": "/iplant/home/shared/phytooracle/season_13_lettuce_yr_2022/level_0/scanner3DTop/",
    "Season 14": "/iplant/home/shared/phytooracle/season_14_sorghum_yr_2022/level_0/scanner3DTop/",
    "Season 15": "/iplant/home/shared/phytooracle/season_15_lettuce_yr_2022/level_0/scanner3DTop/",
    "Season 16": "/iplant/home/shared/phytooracle/season_16_sorghum_yr_2023/level_0/scanner3DTop/",
    "Season 17": "/iplant/home/shared/phytooracle/season_17_lettuce_yr_2023/level_0/scanner3DTop/",
    "Season 18": "/iplant/home/shared/phytooracle/season_18_sorghum_yr_2024/level_0/scanner3DTop/"
}

# Select season
dir_path = select_dir_path(dir_paths)
season_number = str(dir_path.split(os.sep)[-4].split('_')[1])

# Get list of dates within season directory on CyVerse
result = subprocess.run(["ils", dir_path], stdout=subprocess.PIPE)
output = result.stdout.decode("utf-8")

# Get list of all tarballs on CyVerse
tar_files = [line.strip() for line in output.split("\n") if line.endswith(".tar.gz")]

# Define local path of tarball and extracted contents
selected_tar_file = select_tar_file(tar_files)
local_path = selected_tar_file.split(".")[0]

# Download and extract data to local path
if not os.path.isdir(local_path):

    print(f'Downloading {select_tar_file}.')
    subprocess.run(["iget", "-PVT", os.path.join(dir_path, selected_tar_file)]) #, local_path])
    
    print(f'Extracting at {local_path}.')
    tar = tarfile.open(selected_tar_file, "r:gz")
    tar.extractall()
    tar.close()

# Download Environment Logger data
date_species  = local_path.replace('scanner3DTop-', '')
# date_list = get_env_dates(date_string = date_species)
wd = os.getcwd()


# for date in date_list:

#     env_path = download_data(
#                     crop = "NA",
#                     season = season_number,
#                     level = '0',
#                     sensor = 'ENV',
#                     sequence = f'{date}.tar.gz',
#                     cwd = wd,
#                     outdir = 'environment_logger')
#                     # download=False)
    
#     os.chdir(wd)

# Get gantry metadata
meta_df = get_date_position(data_path = os.path.join(local_path, '*', '*', '*.json'))

# # Open environmental logger data
# env_df = get_environment_df(data_path = os.path.join(env_path, '*', '*', '*.json'))

# Create an empty list to store each pair of point clouds
pcd_pairs = []
pcd_directions = []
filenames = []
negative_filenames = []
positive_filenames = []
all_dirs = []
all_ply_files = []

for subdir, dirs, files in os.walk(local_path):

    dirs.sort()
    all_dirs.extend(dirs)

    files.sort(key=lambda file: custom_sort(file, all_dirs))
    ply_files = [file for file in files if file.endswith(".ply")]

    if len(ply_files) == 2:
        all_ply_files.append(tuple(ply_files))

date_format = '%Y-%m-%d__%H-%M-%S-%f'
datetimes = [datetime.strptime(d, date_format) for d in all_dirs]
formatted_datetimes = [dt.strftime('%Y-%m-%d %H:%M:%S') for dt in datetimes]

base_filenames = [name[0].split('__')[0] for name in all_ply_files]
print(base_filenames)

try:

    df = pd.DataFrame({'time': formatted_datetimes, 'directories': all_dirs, 'filename': base_filenames})
    df['time'] = pd.to_datetime(df['time'])
    result = df
    # result = pd.merge_asof(df, env_df, on='time', direction='nearest', tolerance=pd.Timedelta("1m"))

except Exception as e:

    print(f"An error occurred processing point clouds, a subdirectory may be an incomplete pair: {e}")

for (direc_path, ply_files, date_time) in zip(all_dirs, all_ply_files, formatted_datetimes):

    if len(ply_files) == 2:
        
        iter_df = result[result['time']==date_time]

        try:
  
            # Iterate over the items in ply_files
            for item in ply_files:


                # Check if the item contains "east"
                if "east" in item:
                    # Measure the time it takes to read the east point cloud
                    start = time.time()
                    east_pcd = o3d.io.read_point_cloud(os.path.join(local_path, direc_path, item),
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
                    west_pcd = o3d.io.read_point_cloud(os.path.join(local_path, direc_path, item),
                                                       remove_nan_points=True,
                                                       remove_infinite_points=True)
                    end = time.time()
                    print(f'Time taken to read {item}: {end - start:.2f} seconds')

                    # Measure the time it takes to downsample the west point cloud
                    start = time.time()
                    down_west_pcd = copy.deepcopy(west_pcd).voxel_down_sample(voxel_size=15)
                    end = time.time()
                    print(f'Time taken to downsample {item}: {end - start:.2f} seconds')
            
                meta_path = os.path.join(local_path, direc_path, '_'.join([item.split('__')[0], 'metadata.json']))

            print(f'Opening metadata file {meta_path}.')
            metadata = load_metadata_dict(meta_path)

            # Get filename
            filename = '/'.join([direc_path, item.split('__')[0]])
            
            # Get row index of dataframe
            row_index = result.loc[result['filename'] == item.split('__')[0]].index[0]

            # Get field location
            x_position = float(metadata['gantry_system_variable_metadata']['position x [m]'])
            field = get_direction(x_position)
            

            # Get box height
            x_position = float(metadata['gantry_system_variable_metadata']['position x [m]'])
            y_position = float(metadata['gantry_system_variable_metadata']['position y [m]'])
            z_position = float(metadata['gantry_system_variable_metadata']['position z [m]'])

            # Add filename, field locations, and box heights to lists
            filenames.append(filename)

            print('Rotating point clouds.')
            new_east_down = rotate_pcd(down_east_pcd,90)
            new_west_down = rotate_pcd(down_west_pcd,90)

            if metadata['gantry_system_variable_metadata']['scanIsInPositiveDirection'] == "False":

                new_east_down = new_east_down.translate([0,(float(metadata['gantry_system_variable_metadata']['position x [m]'])-3.798989)/(8.904483-7.964989)*1000,0])
                new_west_down = new_west_down.translate([0,(float(metadata['gantry_system_variable_metadata']['position x [m]'])-3.798989)/(8.904483-7.964989)*1000,0])
                pcd_direction = "Negative"
                pcd_directions.append(pcd_direction)
                negative_filenames.append(filename)

            else:

                new_east_down = new_east_down.translate([22280.82692587,(float(metadata['gantry_system_variable_metadata']['position x [m]'])-3.798989)/(8.904483-7.964989)*1000,0])
                new_west_down = new_west_down.translate([22280.82692587,(float(metadata['gantry_system_variable_metadata']['position x [m]'])-3.798989)/(8.904483-7.964989)*1000,0])
                pcd_direction = "Positive"
                pcd_directions.append(pcd_direction)
                positive_filenames.append(filename)

            # Save results to dataframe
            result.loc[row_index, 'field'] = field
            result.loc[row_index, 'x_position'] = x_position
            result.loc[row_index, 'y_position'] = y_position
            result.loc[row_index, 'z_position'] = z_position
            result.loc[row_index, 'scan_direction'] = pcd_direction

            # store the pair of point clouds for later use
            print("Appending point cloud pair to list")
            pcd_pairs.append((new_east_down, new_west_down))
            print("Appended point cloud pair to list")

            del metadata, east_pcd, down_east_pcd, new_east_down, west_pcd, down_west_pcd, new_west_down

        except Exception as e:
            print(f"An error occurred while processing {ply_files[0]} and {ply_files[1]}: {e}")

# Define the output directory
local_path_date = extract_date(string=local_path)
out_dir = os.path.join('scanner3DTop_Transformations', local_path_date)

# Create output directory
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

# Step 1: Align the point clouds within each pair (EW)
# ew_final_transformations = []
ew_positive_final_transformations = []
ew_negative_final_transformations = []
ew_index = []
for i, (source, target) in enumerate(pcd_pairs):
    
    e_pressed = False

    # Create a copy of source
    source_copy = copy.deepcopy(source)
    
    # Paint the point clouds for visualization
    print('Preparing point cloud pair')
    colorize_point_cloud(source, "viridis")
    colorize_point_cloud(source_copy, "viridis")
    colorize_point_cloud(target, "plasma")

    # Set up the visualization
    print("Press 'W', 'A', 'S', 'D', 'R', or 'F' to move the source point cloud")
    print("Press 'E' to save transformation")
    print("Press 'Q' to move to next pair")
    print("Press 'I' to ignore this pair and move to the next pair")

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()

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
    
    if pcd_directions[i] == "Positive":
        vis.register_key_callback(ord("E"), lambda vis: save_transform_and_move_to_next_pair(vis,cum_trans,ew_positive_final_transformations, ew_index, i))
    else:
        vis.register_key_callback(ord("E"), lambda vis: save_transform_and_move_to_next_pair(vis,cum_trans,ew_negative_final_transformations, ew_index, i))

    vis.register_key_callback(ord("Q"), close_window)

    # Run and destroy the visualization
    vis.poll_events()
    vis.run()
    vis.destroy_window()

    # Delete or reassign variables that are no longer needed
    del source_copy

# Calculate the negative final transformation based on all transformations
ew_negative_final_transformation = np.mean(ew_negative_final_transformations,axis=0)
np.save(os.path.join(out_dir, f'{local_path_date}_ew_negative_final_transformation.npy'), ew_negative_final_transformation)

# Calculate the positive final transformation based on all transformations
ew_positive_final_transformation = np.mean(ew_positive_final_transformations,axis=0)
np.save(os.path.join(out_dir, f'{local_path_date}_ew_positive_final_transformation.npy'), ew_positive_final_transformation)

# Apply final transformation to each source point cloud in pcd_pairs
for i, (source, target) in enumerate(pcd_pairs):

    if pcd_directions[i] == "Positive":
        source.transform(ew_positive_final_transformation)
    else:
        source.transform(ew_negative_final_transformation)

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
ns_index = []
for i in range(len(merged_point_clouds)-1):
    if pcd_directions[i] == "Positive":
        e_pressed = False

        source = merged_point_clouds[i]
        target = merged_point_clouds[i+1]
        source_copy = copy.deepcopy(source)

        # Paint the point clouds for visualization
        print('Preparing point cloud pair')
        colorize_point_cloud(source, "viridis")
        colorize_point_cloud(source_copy, "viridis")
        colorize_point_cloud(target, "plasma")
        
        # Set up the visualization
        print("Press 'W', 'A', 'S', 'D', 'R', or 'F' to move the source point cloud")
        print("Press 'E' to save transformation")
        print("Press 'Q' to move to next pair")
        print("Press 'I' to ignore this pair and move to the next pair")

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        # vis.toggle_full_screen()

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
        vis.register_key_callback(ord("E"), lambda vis: save_transform_and_move_to_next_pair(vis,cum_trans,ns_final_transformations, ns_index, i))
        vis.register_key_callback(ord("Q"), close_window)

        # Run and destroy the visualization
        vis.poll_events()
        vis.run()
        vis.destroy_window()

# Calculate the final transformation based on all transformations
ns_final_transformation = np.mean(ns_final_transformations,axis=0)
np.save(os.path.join(out_dir, f'{local_path_date}_ns_final_transformation.npy'), ns_final_transformation)

# Apply final transformation to each target point cloud in pcd_pairs
for i in range(len(merged_point_clouds)):
    if pcd_directions[i] == "Positive":
        # print(f'i: {i}')
        merged_point_clouds[i].transform(ns_final_transformation)

# Visualize all merged point clouds in a single visualization
o3d.visualization.draw_geometries(merged_point_clouds, window_name='Final transformation')

# Clean memory
del merged_point_clouds

# Create H5 file with average and individual transformations
with h5py.File(os.path.join(out_dir, f'{local_path_date}_transformations.h5'), 'w') as f:
    ew_grp = f.create_group('EW')
    ew_individual_grp = ew_grp.create_group('individual')
    ew_trans = ew_individual_grp.create_group('transformations')

    ew_positive_trans = ew_trans.create_group('positive')
    for i, transformation in enumerate(ew_positive_final_transformations):
        ew_idx = ew_index[i]
        ew_positive_trans.create_dataset(positive_filenames[i], data=transformation)

    ew_negative_trans = ew_trans.create_group('negative')
    for i, transformation in enumerate(ew_negative_final_transformations):
        ew_idx = ew_index[i]
        ew_negative_trans.create_dataset(negative_filenames[i], data=transformation)
    
    ew_average_grp = ew_grp.create_group('average')
    ew_negative_average_grp = ew_average_grp.create_group('negative')
    ew_positive_average_grp = ew_average_grp.create_group('positive')
    ew_negative_average_grp.create_dataset('transformation', data=ew_negative_final_transformation)
    ew_positive_average_grp.create_dataset('transformation', data=ew_positive_final_transformation)
    
    ns_grp = f.create_group('NS')
    ns_individual_grp = ns_grp.create_group('individual')
    ns_trans = ns_individual_grp.create_group('transformations')
    for i, transformation in enumerate(ns_final_transformations):
        ns_idx = ns_index[i]
        ns_trans.create_dataset(filenames[ns_idx], data=transformation)

    ns_average_grp = ns_grp.create_group('average')
    ns_average_grp.create_dataset('transformation', data=ns_final_transformation)

# assuming you have a DataFrame called results
result.to_hdf(os.path.join(out_dir, f'{local_path_date}_transformations.h5'), key='environment_logger')

# Define CyVerse output directory
cyverse_out_path = os.path.join(dir_path, 'scanner3DTop_Transformations')

# Upload data to CyVerse
subprocess.run(f"imkdir -p {cyverse_out_path} && icd {cyverse_out_path} && iput -rfKPVT {out_dir}", shell=True)

# Clean working directory
remove_directory(dir_path='scanner3DTop_Transformations')
remove_directory(dir_path=local_path)
# remove_directory(dir_path='environment_logger')
remove_file(file_path=selected_tar_file)
