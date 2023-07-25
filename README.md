# 3D Transformation selection
This code allows a user to calculate season-specific transformations for level_0 Scanner3DTop point cloud data. The application is a graphical user interface (GUI) that allows the user to (i) select a season, (ii) select a scan date, (iii) calculate a east-west (EW) transformation, and calculate a north-south (NS) transformation. The output is a single H5 file containing all calculated transformations.

## Running the GUI
The GUI was designed to run on Singularity. To download the container, run the following command:

```
singularity build 3d_transformation_selection.simg docker://phytooracle/3d_transformation_selection:latest
```

To open the GUI, run the following command:

```
singularity run 3d_transformation_selection.simg
```

The GUI will load up and prompt the user to select a season and scan date. Once selected, the data will be downloaded, extracted, and visualized.

## Interacting with the GUI
After selecting a season and scan date, the GUI visualizes the point clouds using Open3D. The user is then able to conduct EW and NS transformations using the following keystrokes:

- Up: W
- Down: S
- Left: A
- Right: D
- Forward: R
- Backward: F
- Ignore pair: I
- Save transformation: E
- Quit: Q

> **_NOTE:_** The user must hit `E` to save the transformation before hitting `Q` to move to the next pair. If the user does not want a pair to be included in the transformation calculation, they must hit `I` to ignore the pair and move to the next pair.
> **_NOTE:_** The user must ensure CAPS lock is disabled.
