# STD-Trees: Spatio-Temporal Deformable Trees for Multirotors Kinodynamic Planning
[[Preprint]](https://github.com/ZJU-FAST-Lab/std-trees/blob/main/misc/draft.pdf)
[[Video]](https://www.youtube.com/watch?v=uCOofavIp9w)

The code is now a bit of messy, I'll tide it up later.
## Build & Run
### Build
1. create a workspace, for example ~/ws/src
2. in ~/ws/src/, _git clone git@github.com:ZJU-FAST-Lab/std-trees.git_
3. in ~/ws/, run _catkin_make_

### Run
In two seperate terminals, in ~/ws/, _source_ first, then:
1. roslaunch state_machine rviz.launch
2. roslaunch state_machine test_planners.launch

If everything goes right, you'll see sth. like the following in Rviz.
<p align="center">
  <img src="misc/compare-wo.gif" width = "887" height = "200"/>
</p>

The color indications are light blue for kRRT, dark
blue for kRRT-ST, yellow for kRRT*, green for kRRT*-ST, orange for kRRT#, and red for kRRT#-ST. Planning with ST deformation
generates smoother trajectories with lower cost (red, green, and dark blue compared with orange, yellow, and light blue, respectively)