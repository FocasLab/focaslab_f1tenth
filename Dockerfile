FROM tiryoh/ros-desktop-vnc:noetic
MAINTAINER Allen Emmanuel Binny

# set time/language
ENV TZ=Asia/Kolkata
ENV LANG C.UTF-8
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Update the image and install required packages
RUN apt-get update -y \
	&& apt-get install -y wget git unzip build-essential gcc g++ clang

# it continues to remain in bash in future images
SHELL ["/bin/bash", "-c"]

# install required ros packages for simulations
RUN apt-get install -y \
	ros-noetic-tf2-geometry-msgs \
	ros-noetic-ackermann-msgs \
	ros-noetic-map-server \
	ros-noetic-joy \
	&& echo "ubuntu\n" | sudo rosdep update

# creaking catkin workspace for ros
RUN mkdir -p ~/catkin_ws/src \
	&& cd ~/catkin_ws/src

# getting autonomous exploration packages.
COPY ./ /home/ubuntu/catkin_ws/src/

# building the packages
RUN source /opt/ros/noetic/setup.bash \
	&& cd ~/catkin_ws \
	&& rosdep install --from-paths src --ignore-src --rosdistro=noetic -y \
	&& catkin build \
	&& echo	"source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc