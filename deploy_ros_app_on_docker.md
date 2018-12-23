Deploying ROS apps on docker
========
 ## How to use Docker to ship ROS apps?

As we have learnt to make things easier so far, let's try to implement it step by step. Links to installation and proxy setup for Docker are given in the end of this documentation. After that, let's start.

### Volumes

By default Docker uses  `/root/.ros/`  directory to keep track of the logs and debugging info. If you want to change it to your home directory(say 'ubuntu'), perform :
```
$ docker run -v "/home/ubuntu/.ros/:/root/.ros/" ros
```

### Devices

Some application may require device access for acquiring images from connected cameras, control input from human interface device, or GPUS for hardware acceleration. This can be done using the --device run argument to mount the device inside the container, providing processes inside hardware access.

### Networks

Although one process per container is recommended, Docker networks can also be extended to use several running ROS apps. See documentation on  [NetworkSetup](https://hk.saowen.com/rd/aHR0cDovL3dpa2kucm9zLm9yZy9ST1MvTmV0d29ya1NldHVw)  .

## Start by an example

For the communication between our ROS nodes, there should be a virtual network. In this short example, we’ll create a virtual network, spin up a new container running roscore advertised as the master service on the new network, then spawn a message publisher and subscriber process as services on the same network.

### Build image

Build a ROS image that includes ROS tutorials using this  `Dockerfile`  :
```
FROM ros:indigo-ros-base
# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    ros-indigo-ros-tutorials \
    ros-indigo-common-tutorials \
    && rm -rf /var/lib/apt/lists/
```
Then to build the image from within the same directory:
```
$ docker build --tag ros:ros-tutorials .
```
### Create network

To create a new network foo, we use the network command:

docker network create foo

Now that we have a network, we can create services. Services advertise there location on the network, making it easy to resolve the location/address of the service specific container. We’ll use this make sure our ROS nodes can find and connect to our ROS master.

### Run services

To create a container for the ROS master and advertise it’s service:
```
$ docker run -it --rm \
    --net foo \
    --name master \
    ros:ros-tutorials \
    roscore
```
Now you can see that master is running and is ready manage our other ROS nodes. To add our talker node, we’ll need to point the relevant environment variable to the master service:
```
$ docker run -it --rm \
    --net foo \
    --name talker \
    --env ROS_HOSTNAME=talker \
    --env ROS_MASTER_URI=http://master:11311 \
    ros:ros-tutorials \
    rosrun roscpp_tutorials talker
```
Then in another terminal, run the listener node similarly:
```
$ docker run -it --rm \
    --net foo \
    --name listener \
    --env ROS_HOSTNAME=listener \
    --env ROS_MASTER_URI=http://master:11311 \
    ros:ros-tutorials \
    rosrun roscpp_tutorials listener
```
Alright! You should see listener is now echoing each message the talker broadcasting. You can then list the containers and see something like this:
```
$ docker service ls
SERVICE ID          NAME                NETWORK             CONTAINER
67ce73355e67        listener            foo                 a62019123321
917ee622d295        master              foo                 f6ab9155fdbe
7f5a4748fb8d        talker              foo                 e0da2ee7570a
```
And for the services:
```
$ docker ps
CONTAINER ID        IMAGE               COMMAND                CREATED              STATUS              PORTS               NAMES
a62019123321        ros:ros-tutorials   "/ros_entrypoint.sh    About a minute ago   Up About a minute   11311/tcp           listener
e0da2ee7570a        ros:ros-tutorials   "/ros_entrypoint.sh    About a minute ago   Up About a minute   11311/tcp           talker
f6ab9155fdbe        ros:ros-tutorials   "/ros_entrypoint.sh    About a minute ago   Up About a minute   11311/tcp           master
```
### Introspection

Ok, now that we see the two nodes are communicating, let get inside one of the containers and do some introspection what exactly the topics are:
```
$ docker exec -it master bash
  source /ros_entrypoint.sh
```
If we then use rostopic to list published message topics, we should see something like this:
```
$ rostopic list
/chatter
/rosout
/rosout_agg
```
### Tear down

To tear down the structure we’ve made, we just need to stop the containers and the services. We can stop and remove the containers using Ctrl^C where we launched the containers or using the stop command with the names we gave them:
```
$ docker stop master talker listener
$ docker rm master talker listener
```
### Compose

Now that you have an appreciation for bootstrapping a distributed ROS example manually, lets try and automate it using docker-compose.

Start by making a folder named rostutorials and moving the Dockerfile we used earlier inside this directory. Then create a yaml file named docker-compose.yml in the same directory and paste the following inside:
```
version: '2'
services:
  master:
    build: .
    container_name: master
    command:
      - roscore
  
  talker:
    build: .
    container_name: talker
    environment:
      - "ROS_HOSTNAME=talker"
      - "ROS_MASTER_URI=http://master:11311"
    command: rosrun roscpp_tutorials talker
  
  listener:
    build: .
    container_name: listener
    environment:
      - "ROS_HOSTNAME=listener"
      - "ROS_MASTER_URI=http://master:11311"
    command: rosrun roscpp_tutorials listener
```
Now from inside the same folder, use docker-copose to launch our ROS nodes and specify that they coexist on their own network:
```
$ docker-compose up -d
```
Notice that a new network named rostutorials_default has now been created, you can inspect it further with:
```
$ docker network inspect rostutorials_default
```
We can monitor the logged output of each service, such as the listener node like so:
```
$ docker-compose logs listener
```
Finally, we can stop and remove all the relevant containers using docker-copose from the same directory:
```
$ docker-compose stop
$ docker-compose rm
```
Note: the auto-generated network, rostutorials_default, will persist over the life of the docker engine or until you explicitly remove it using docker network rm.

[reference](https://hk.saowen.com/a/d908b97687e3a3cb6425a89626bf355f92ffac8f667d3cabbc06b404978d6af3)

<!--stackedit_data:
eyJoaXN0b3J5IjpbLTExNzcwMTYzXX0=
-->