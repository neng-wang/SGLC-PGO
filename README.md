# SGLC-PGO

This repository integrates the SGLC into SLAM back-end, which is  implemented through a pose graph optimization framework. It primarily  references [SC-PGO](https://github.com/gisbi-kim/SC-A-LOAM?tab=readme-ov-file). 

And for simplicity, we have omitted the front-end  odometry estimation.

### Usage

```bash
mkdir catkin_ws/src
cd catkin_ws/src
git clone git@github.com:neng-wang/SGLC-PGO.git
catkin_make
source devel/setup.bash
roslaunch sglc demo_pgo_loop_correction.launch  // for slam 
#Tip: replace the lidar_path and label_path in the .launch file
```

**pose: [time x y z qx qy qz qw]** from the kissicp 



