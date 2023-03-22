# NSL-3130AA ROS2
--- NSL-3130AA ROS2 demo ---

1. Build env
 - Ubuntu22.04.1 LTS
 - ROS2 Humble
 - OPENCV 4.5.4
 
 
2. Build NSL-3130AA demo
```
$ cd NSL3130_driver
$ colcon build --packages-select roboscan_nsl3130
$ . install/setup.bash
```
 
3. Start commands
```
$ ros2 run roboscan_nsl3130 roboscan_publish_node
$ ros2 launch roboscan_nsl3130 camera.Launch.py
```

# NSL-3130AA View


 ![aaa](https://user-images.githubusercontent.com/106071093/226831747-71e4c269-0fa9-483a-b781-78ac131eaf6b.png)


# Set parameters
```
$ rqt
 (reconfigure)
```

![bbbb](https://user-images.githubusercontent.com/106071093/226831796-d487fc42-5ae4-40c4-b5f9-e4f18af08d7c.png)


```

//
```

 



 
 
 
