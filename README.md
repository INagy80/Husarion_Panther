# BP2526_Module3_Group1

## Quick start

### Create workspace

```bash
mkdir ~/husarion_ws
cd ~/husarion_ws
git clone -b ros2 https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros
```

### Configure environment

Simulation:

```bash
export HUSARION_ROS_BUILD_TYPE=simulation
```

### Build

``` bash
vcs import src < src/husarion_ugv_ros/husarion_ugv/${HUSARION_ROS_BUILD_TYPE}_deps.repos

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source install/setup.bash
```


Simulation:

```bash
ros2 launch husarion_ugv_gazebo simulation.launch.py
```


### LIDAR Configuration 

we have edited the component.yaml to add the LIDAR 

component.yaml:
```yaml
components:
  - type: LDR06
    parent_link: base_link  
    xyz: 0.15 0.0 0.25    
    rpy: 0.0 0.0 0.0        
    frequency: 10.0       
    horizontal_samples: 720  
    horizontal_fov: 3.14159 
    range_min: 0.12        
    range_max: 12.0        
    noise_stddev: 0.01    
    topic: /scan           
    frame_id: laser_front_link 
```    



then build again with colcon to accept the new changes 

```bash
colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```



### Slam_toolbox Download 

```bash
sudo apt install ros-humble-slam-toolbox
```


Slam_toolbox Launch:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
 
> we used the argument use_sim_time:=true to subscripe to the /clock topic to use the simulation clock not the real time clock



then add the map to Rviz but we had a problem with the map because we were subscriping the wrong topic and the default topic for Slam_toolbox is  /scan  so after correcting this we got every thing setup and working 
