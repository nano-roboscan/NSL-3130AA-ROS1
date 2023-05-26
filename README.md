# NSL-3130AA ROS1
--- NSL-3130AA ROS1 demo ---

1. Build env
 - Ubuntu18.04
 - ROS1 Melodic
 - OPENCV 4.5.4
 
 
2. Build NSL-3130AA demo
```
$ cd NSL3130_driver
$ catkin_make
$ source ./devel/setup.bash
```
 
3. Start commands
```
$ rosrun roboscan_nsl3130 roboscan_publish_node
$ roslaunch roboscan_nsl3130 camera.Launch
```

# NSL-3130AA View

 ![aaa](https://user-images.githubusercontent.com/106071093/226831747-71e4c269-0fa9-483a-b781-78ac131eaf6b.png)

# Rollover Avoidance and Correction

  ![rollover](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/3d30e77f-0b82-48d1-be3e-2ca3455a9bba)

# Average FPS per Image type

  ![fps](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/532178f4-23ac-4eee-ae8d-a4f8fb03b747)


# Set parameters
```
$ rqt
 (reconfigure)
```

![bbbb](https://user-images.githubusercontent.com/106071093/226831796-d487fc42-5ae4-40c4-b5f9-e4f18af08d7c.png)


```

//
```

 



 
 
 
