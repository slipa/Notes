fork from [alecGraves/wiki](https://github.com/alecGraves/wiki/wiki/How-to-use-Docker-(with-ROS))

# The Bare Minimum
### If using ubuntu

    echo "alias docker='sudo docker'" >> ~/.bash_aliases
or 
```bash
sudo addgroup --system docker
sudo adduser $USER docker
newgrp docker
```

### Start an interactive container for catkin-build

    docker run -i -t \
    -v ~/catkin-build:/root/ros/catkin_ws/src \
    --name build0 ros:kinetic-ros-core /bin/bash


### Start an old container in interactive mode

    docker start -i build0


# Other Useful Commands

### Enter an active docker container from new terminal

    docker exec -i -t <container_name> /bin/bash

    docker exec -i -t <container_id> /bin/bash

### Exit a container

    exit

### Exit a container without killing it

```CTRL-p``` and ```CTRL-q```

### View active containers

    docker ps


### View all containers

    docker ps -a


### View all containers and their sizes

    docker ps -as


### Remove a container permanently

    docker rm <name>

    docker rm <container_id>


### Pull a specific image
(running a container will automatically pull the required image, so this is normally unnecessary)

    docker pull ros
    
    docker pull ros:kinetic-ros-core


### View all images on device

    docker images -a


### Remove a specific image

    docker rmi <image_id>

# Advanced

### catkin_make
```bash
# Set up variable for base container
echo "MY_ROS_IMAGE='ros:kinetic-ros-core'" >> ~/.bashrc
```

Add the following aliases to your ```~/.bash_aliases```:
```bash
# Clean catkin_make, removes build && devel
alias ccm='docker run --rm -itv ~/catkin_ws:/root/catkin_ws $MY_ROS_IMAGE\
           sh -c "cd /root/catkin_ws && mkdir src || : && rm -rf build/ devel/ && catkin_make"'
```

```bash
# Regular catkin_make
alias cm='docker run --rm -itv ~/catkin_ws:/root/catkin_ws $MY_ROS_IMAGE \
          sh -c "cd /root/catkin_ws && catkin_make"'
```

```bash
# full access to catkin env
alias env='docker run --rm -itv ~/catkin_ws:/root/catkin_ws -w="/root/catkin_ws/" $MY_ROS_IMAGE'
```

### Run a ros system with docker containers
```bash
# Create a network for your docker containers:
docker network create skynet
```

Set up this in your ```~/.bash_aliases```:
```bash
# run roscore container
alias roscore='docker run -it --rm \
    --net skynet \
    --name master \
    ros:kinetic-ros-core \
    roscore'
```

Now, with custom containers for each node,
```bash
docker run -it --rm \
    --net skynet \
    --name $NODE_NAME \
    --env ROS_HOSTNAME=$NODE_NAME \
    --env ROS_MASTER_URI=http://master:11311 \
    --device=/dev/ttyUSB1:/dev/ttyACM0 \
    $NODE_IMAGE \
    rosrun $PACKAGE $NODE_NAME
```

### Set up a containerized ros system across computers (swarm)
Open ports ```2377```, ```7946```, and ```4789```.
```bash
# Start a swarm
docker swarm init --advertise-addr=$HOSTIPADDRESS --listen-addr $HOSTIPADDRESS:2377
```

```bash
# Connect all systems to the swarm
docker swarm join --token $JOINTOKEN $HOSTIPADDRESS
```
* ```$JOINTOKEN``` is the worker join-token provided as output by the ```docker swarm init```

```bash
# Create an overlay network
docker network create --driver=overlay $NETWORKNAME
```

```bash
# View all swarm-nodes
docker node ls
```

```bash
# Add labels to distinguish between different nodes
docker node update --label-add $LABELNAME=$LABELVALUE $ID
```
* ```$LABELNAME=$LABELVALUE``` example: ```type=robot``` and ```type=ground_station```
* ```$ID``` can be seen from output of ```docker node ls```. ```HOSTNAME``` also works.

```bash
# run the roscore container on one of the machines
docker service create --name=master --network=$NETWORKNAME \
    --constraint node.labels.type=ground_station ros:kinetic-ros-core "roscore"
```
* ```docker service logs --follow``` command will continue streaming the new output from the service’s STDOUT and STDERR
* or use the above version of roscore.

```bash
# Run desired containers on each system
docker run -it --rm \
    --net $NETWORKNAME \
    --name $NODE_NAME \
    --env ROS_HOSTNAME=$NODE_NAME \
    --env ROS_MASTER_URI=http://master:11311 \
    --device=/dev/ttyUSB1:/dev/ttyACM0 \
    $NODE_IMAGE \
    rosrun $PACKAGE $NODE_NAME
```
* cannot use docker services for this until --device support for services is added.
* edit: may be able to access devices using volumes (```--mount```)?
