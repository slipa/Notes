暫存，尚未深入了解
```
version: '2'

networks:
  ros:
    driver: bridge

services:
  ros-master:
    image: ros:kinetic-ros-core
    command: stdbuf -o L roscore
    networks:
      - ros
    restart: always

  talker:
    image: ros:kinetic-ros-core
    depends_on:
      - ros-master
    environment:
      - "ROS_MASTER_URI=http://ros-master:11311"
      - "ROS_HOSTNAME=talker"
    command: stdbuf -o L rostopic pub /chatter std_msgs/String "hello" -r 1
    networks:
      - ros
    restart: always

  listener:
    image: ros:kinetic-ros-core
    depends_on:
      - ros-master
    environment:
      - "ROS_MASTER_URI=http://ros-master:11311"
      - "ROS_HOSTNAME=listener"
    command: stdbuf -o L rostopic echo /chatter
    networks:
      - ros
    restart: always
```

