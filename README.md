# IVALab RoboSLAM

## Docker Build and Run

```bash
# Clone.
git clone https://github.com/ivalab/cartographer_ros.git

# Download data to HOST data directory.
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag

# Build.
cd cartographer_ros

sudo docker build -t cartographer:noetic -f Dockerfile.noetic .

# Run docker.
sudo docker run -it --net=host -e DISPLAY=$DISPLAY -v $HOME/Downloads:/data -v /tmp/.X11-unix:/tmp/.X11-unix --runtime=nvidia --device /dev/nvidia0 --device /dev/nvidia-uvm --device /dev/nvidia-uvm-tools --device /dev/nvidiactl  --name="carto" cartographer:noetic

# Run cartographer.
cd /catkin_ws
source install_isolated/setup.bash

roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=/data/cartographer_paper_deutsches_museum.bag

# If met rviz/QT issue, run the following in HOST before docker run.
xhost +local:docker
```

```bash
# Restart the container.
sudo docker start `sudo docker ps -q -l` && sudo docker attach `sudo docker ps -q -l`
# OR
sudo docker start `CONTAINER_ID` && sudo docker attach `CONTAINER_ID`
```