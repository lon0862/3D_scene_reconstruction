# 3D_scene_reconstruction
In this repository, I complete two tasks for the HW1 of Perception and Decision Making in Intelligent Systems, NYCU, in fall 2022.

# Abstact
Task1: BEV projection<br>
Select points in the BEV (top view) image on the ground, mark an
area enclosed by these points.Then project the marked area from the
BEV image to the perspective (front view) image.<br><br>

Task2: ICP Alignment and Reconstruction<br>
First using rgb and depth image to construct point cloud,then using open3d ICP and implement my own ICP to build two versions
of 3D scene.Finally add ground truth trajectory and estimated trajectory to 3D scene.<br>

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
