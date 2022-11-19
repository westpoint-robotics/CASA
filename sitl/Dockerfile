FROM ubuntu:jammy

# setup environment
ENV ROS_DISTRO humble

RUN apt-get update \
	&& apt-get install --quiet -y curl lsb-core git


# setup ros2 keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# setup sources.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

#RUN apt install tmux nano htop -y

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata


# install bootstrap tools
RUN apt-get install -y python3-pip \
		python3-colcon-common-extensions \
		python3-colcon-mixin \
		python3-rosdep \
		python3-vcstool \
		python3-genmsg \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata

# install ros2 desktop
RUN apt-get install --quiet --no-install-recommends -y \
		ros-$ROS_DISTRO-desktop-full \
		ros-$ROS_DISTRO-launch-testing-ament-cmake \
		ros-$ROS_DISTRO-ros2bag \
		ros-$ROS_DISTRO-rosidl-generator-dds-idl \
		ros-$ROS_DISTRO-gazebo-ros-pkgs \
		libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Install Python 3 packages needed for testing
RUN pip3 install -U \
		argcomplete \
		flake8 \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes \
		pytest-repeat \
		pytest-rerunfailures \
		kconfiglib \
		jinja2 \
		genmsg \
		jsonschema \
		future

# bootstrap rosdep

RUN rosdep init
RUN rosdep update

RUN curl -sSL http://get.gazebosim.org | sh

WORKDIR /src
# Repository forked in order to avoid by default "missing RC" failsafe when in offboard mode
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

WORKDIR /src/PX4-Autopilot/Tools/setup

RUN ./ubuntu.sh


WORKDIR /src/PX4-Autopilot
RUN HEADLESS=1 make px4_sitl_default gazebo
#RUN rm Tools/sitl_gazebo/models/iris/iris.sdf.jinja
#RUN mv iris.sdf.jinja Tools/sitl_gazebo/models/iris/
#RUN rm Tools/sitl_gazebo/worlds/iris_irlock.world
#RUN mv iris_irlock.world Tools/sitl_gazebo/worlds/
#RUN rm Tools/sitl_gazebo/models/iris_irlock/iris_irlock.sdf
#RUN mv iris_irlock.sdf Tools/sitl_gazebo/models/iris_irlock/
#UN HEADLESS=1 make px4_sitl_rtps gazebo

WORKDIR /

