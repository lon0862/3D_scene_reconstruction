# 3D_scene_reconstruction
In this repository, I complete two tasks for the HW1 of Perception and Decision Making in Intelligent Systems, NYCU, in fall 2022.

# Abstact
Task1: BEV projection
Select points in the BEV (top view) image on the ground, mark an
area enclosed by these points, and project the marked area from the
BEV image to the perspective (front view) image.

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
