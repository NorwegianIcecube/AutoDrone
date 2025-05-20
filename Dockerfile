FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

#RUN apt-get install software-properties-common
#RUN add-apt-repository universe
#RUN apt-get update 
#RUN apt-get install python3.8

RUN apt-get update && apt-get install curl gnupg2 lsb-release -y && \
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    gnome-terminal \
    git \
    build-essential \
    cmake \
    make \
    libopencv-dev \
    lsb-release \
    wget \
    gnupg2 \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    xvfb \
    x11-utils \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    sudo \
    supervisor \
    && rm -rf /var/lib/apt/lists/*
    

WORKDIR /root/


RUN git config --global core.compression 0 && \
    git clone --depth 1 https://github.com/PX4/PX4-Autopilot.git --recursive && \
    cd PX4-Autopilot && \
    git fetch --unshallow && \
    git pull --all && \
    cd .. && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    cd PX4-Autopilot/ && \
    make px4_sitl

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-foxy-ros-base \
    ros-dev-tools \
    python3-argcomplete \
    python3-colcon-common-extensions

#RUN apt-get install -y snapd && \
#    snap install micro-xrce-dds-agent --edge
RUN git clone -b foxy https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install && \
    sudo ldconfig /usr/local/lib/

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all,graphics

RUN pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools typing-extensions

WORKDIR /root/AutoDrone
COPY ./requirements.txt /root/AutoDrone/requirements.txt
RUN cd /root/AutoDrone && \
    pip3 install -r requirements.txt
COPY . /root/AutoDrone/


RUN rosdep init && rosdep update && rosdep install --from-paths /root/AutoDrone --ignore-src -r -y

#RUN colcon build && source install/setup.bash

# Copy world file over and rename iris_fpv_cam to iris_opt_flow (workaround for not being able to boot with iris_fpv_cam with the make command)
#COPY ./pilot-house.world .
#COPY ./pilot-house/ .

RUN rm -rf /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_opt_flow && \
    mv /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_fpv_cam /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_opt_flow && \
    mv /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_opt_flow/iris_fpv_cam.sdf /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_opt_flow/iris_opt_flow.sdf
COPY  world_stuff/pilothouse.world /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
COPY ./world_stuff/pilothouse /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/pilothouse

      
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

      
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]

    