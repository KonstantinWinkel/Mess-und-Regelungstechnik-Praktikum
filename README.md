Created by: Jonathan Erhard, Konstantin Winkel

Files provided by the university: 
- in catkin_ws:				build, Beispielpfade, devel 
- in catkin_ws/src:			lms100, openslam_gmapping, slam_gmapping, volksbot
- in catkin_ws/localization/include:	gio_path, include
- in catkin_ws/giovanni:		gio_path.cpp
- in catkin_ws/giovanni/src:		gio_path.cpp

Selfmade Files: everything else

You can find more resources about this practical assignment here: https://drive.google.com/drive/folders/1YWvwzzEX6GaDsA2NkhZ-dV1ToEcWV-ak?usp=sharing

Contents:
I. Setup
II. Localization Listener
III. Giovanni Simulation
IV. Giovanni Controller
V. Visualization with GNU Octave 
VI. Path Generator

-------------------------------------
I. Setup

1. Go to the catkin workspace and paste all the files and folders into it
2. Run catkin_make
3. You're ready to go

-------------------------------------
II. Localization Listener

The Localisation Listener is a node that listens to either the odometry or AMCL positions.
It then creates either the file "odometryData.csv" or "amclData.csv" in the workspace directory. These can then be plotted with GNU Octave to see the path the robot drove (see V). 
To select which topic the Localization Listener should listen to you need to start the node with the corresponding parameter. The parameters are "odom" for odometry and "amcl" for AMCL.
Please note that for the files to actually be generated the node has to be closed using CTRL+C.
Here are the full commands to paste into the terminal:

For odometry: rosrun localization csvgenerator _mode:=odom

For AMCL: rosrun localization csvgenerator _mode:=amcl

-------------------------------------
III. Giovanni Simulation

The node Giosim contains the simulation of the Giovanni Controller for the robot. 
When running the node as a simulation it will create the file "giosim.csv" in the workspace directory. This can then be plotted with GNU Octave to see the path the simulation thinks the robot would drive (see V).

Here is the full command to paste into the terminal: rosrun giovanni giosim _path:=(path to your .dat)

-------------------------------------
IV. Giovanni Controller

Based on the simulation described in III we have created a controller for the Volksbot robot. Using this controller a path from a dat-file can be followed. The controller can use either odometry or AMCL for its input.
To select which one the controller should use, a "mode"-parameter has to passed when running the node. A second parameter also needs to be passed, leading the controller to the desired dat-file.
If your dat-file was created using absolute coordinates rather than relative coordinates (see Path Generator using AMCL), an additional parameter "_abs:=y" has to be passed.
Here are the full commands to paste into the terminal:

For odometry: rosrun giovanni giocontrol _mode:=odom _path:=(path to your .dat)

For AMCL (relative): rosrun giovanni giocontrol _mode:=amcl _path:=(path to your .dat)

For AMCL (absolute): rosrun giovanni giocontrol _mode:=amcl _path:=(path to your .dat) _abs:=y

NOTE: If the robot doesn't start driving, try closing and restarting the controller node a few times.

-------------------------------------
V. Visualization with GNU Octave

Provided with our CPP code is a very simple GNU Octave file that can be used to easily plot the path the robot takes.
To do so, simply move the csv-file you want to plot into the Octave directory, open the file "CSV_Plotter.m", exchange the name of the csv-file to the name of your file and click run.
It should then plot the path your robot drove.
As an example the file already plots 4 paths, one path calculated by the Giovanni Simulation (see III), the path the robot uses while creating a map (AMCL and Odometry data) and one dataset of the robot following a simple path. 

-------------------------------------
VI. Path Generator

To create your own path that can be followed by the Giovanni Controller (or printed by the Giovanni Simulation), a node has been provided to create your own dat-files.
It works similarly to the Localization Listener. It subscribes to either odometry or AMCL position and creates a dat-file called "lastPath.dat" in your workspace.
We have found that the easiest way to create a path is to record it as a ROS-bag and then later use the Path Generator.
Please note that for the files to actually be generated the node has to be closed using CRTL+C.
Also note that when using the Path Generator with AMCL an additional parameter needs to be parsed to determine if the file should be created in absolute or relative coordinates. If you want to use the path with the Controller in "AMCL" mode, then using absolute coordinates is recommended. If you intend to use the path with the Controller in "Odom" mode, then you must use relative coordinates.
Here are the full commands to paste into the terminal:

For Odometry: rosrun localization pathgenerator _mode:=odom	

For AMCL (absolute): rosrun localization pathgenerator _mode:=amcl _amclmode:=abs

For AMCL (relative): rosrun localization pathgenerator _mode:=amcl _amclmode:=rel
