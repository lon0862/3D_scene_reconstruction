# 3D_scene_reconstruction
In this repository, I complete two tasks for the HW1 of Perception and Decision Making in Intelligent Systems, NYCU, in fall 2022.

# Abstact
- Task1: BEV projection<br>
Select points in the BEV (top view) image on the ground, mark an
area enclosed by these points.Then project the marked area from the
BEV image to the perspective (front view) image.<br>

- Task2: ICP Alignment and Reconstruction<br>
First using rgb and depth image to construct point cloud,then using open3d ICP and implement my own ICP to build two versions
of 3D scene.Finally add ground truth trajectory and estimated trajectory to 3D scene.<br>

# Quick Start
The requirement of the development environments:
- OS : ubuntu 18.04 , 20.04
- Python 3.6, 3.7 ( You can use conda to create new environment )
- opencv-contrib-python
- Open3d
- Habitat-lab and Habitat-sim
following the link https://github.com/facebookresearch/habitat-lab
to install it.

# Download Data
Download the replica datasets from the link below.<br>
Apartment_0 :https://drive.google.com/file/d/1zHA2AYRtJOmlRaHNuXOvC_OaVxHe56M4/view?usp=sharing<br>
Note : You can change agent_state.position to set the agent in
the first or second floor ( (0, 0, 0) for first floor and (0, 1, 0) for
second floor.

# Task 1
- Data collection: <br>
run the following command and use WAD to move camera, then use S to save front view's and BEV's rgb image. 
```
python load_task1.py
```
- Get BEV projection
run the following command and use mouse to select points in the BEV image, when select enough points you want then push down any key you will get projection in front view.<br>
And what points you select and result of projection will auto save.<br>
Push down any key again then will finish the program.
```
python bev.py
```
```
habitat-lab
    +- load_task1.py
    +- load_task2.py
    +- bev.py
    +- reconstruct.py
    +- task1_data
        +- front_view_path.png
	+- top_view_path.png
    +- task2_data 
	+- floor1
	    +- rgb
		+- rgb_0.png
		+- rgb_1.png
		...
	    +- depth
		+- depth_0.png
		+- depth_1.png
		...
	    +- GT
		+- GT.csv
	+- floor2
	    +- rgb
		+- rgb_0.png
		+- rgb_1.png
		...
	    +- depth
		+- depth_0.png
		+- depth_1.png
		...
	    +- GT
		+- GT.csv
```
