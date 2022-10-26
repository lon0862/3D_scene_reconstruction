# 3D_scene_reconstruction
In this repository, I complete two tasks for the HW1 of Perception and Decision Making in Intelligent Systems, NYCU, in fall 2022.

# Abstact
- Task1: BEV projection<br>
Select points in the BEV (top view) image on the ground, mark an
area enclosed by these points.Then project the marked area from the
BEV image to the perspective (front view) image.<br>

![image](https://github.com/lon0862/3D_scene_reconstruction/blob/main/pictures/bev_circle.png) ![image](https://github.com/lon0862/3D_scene_reconstruction/blob/main/pictures/front_projection.png)
- Task2: ICP Alignment and Reconstruction<br>
First using rgb and depth image to construct point cloud,then using open3d ICP and implement my own ICP to build two versions
of 3D scene.Finally add ground truth trajectory and estimated trajectory to 3D scene.<br>

![image](https://github.com/lon0862/3D_scene_reconstruction/blob/main/pictures/3D_scene.png)

# Quick Start
The requirement of the development environments:
- OS : ubuntu 18.04 , 20.04
- Python 3.6, 3.7 ( You can use conda to create new environment )
- opencv-contrib-python
- Open3d
- Habitat-lab and Habitat-sim
following the link https://github.com/facebookresearch/habitat-lab
to until Installation step3.
note: replace pip install -e habitat-lab => /home/{user_name}/anaconda3/envs/habitat/bin/pip install -e .


# Download Data
Download the replica datasets from the link below.<br>
Apartment_0: https://drive.google.com/file/d/1zHA2AYRtJOmlRaHNuXOvC_OaVxHe56M4/view?usp=sharing<br>
Note : You can change agent_state.position to set the agent in
the first or second floor ( (0, 0, 0) for first floor and (0, 1, 0) for
second floor.

# Task 1
- Data collection: <br>
run the following command and use WAD to move camera, then use S to save front view's and BEV's rgb image. <br>
F key will finish the program without save.
```
python load_task1.py
```
- Get BEV projection: <br>
run the following command and use mouse to select points in the BEV image, when select enough points you want then push down any key you will get projection in front view.<br>
what points you select and result of projection will auto save.<br>
push down any key again then will finish the program.
```
python bev.py
```
# Task 2
- Data collection: <br>
run the following command and use WAD to move camera. <br>
rgb image, depth image, ground truth trajectory will auto save when you move.<br>
F key will finish the program.<br>
O key will clean all rgb image, depth image, ground truth trajectory saved before
```
python load_task2.py
```
- Reconstruction 3D scene: <br>
run the following command to reconstruction 3D scene.<br>
```
python reconstruct.py
```
# Structure of directory
```
habitat-lab
    .......
    +- replica_v1
    	+- apartment_0
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
