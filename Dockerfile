FROM ros:kinetic

# create a "mars" user with sudo privileges
# TODO: add tidying commands to shrink image size
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y sudo && \
    useradd -ms /bin/bash mars && \
    usermod -a -G sudo mars && \
    sed -i "s/(ALL:ALL) ALL/(ALL) NOPASSWD: ALL/" "/etc/sudoers" && \
    mkdir -p /home/mars
USER mars
ENV USER mars
WORKDIR /home/mars

RUN git config --global url.https://github.com/.insteadOf git://github.com/

RUN sudo apt-get update \
 && sudo apt-get install -y --no-install-recommends ros-kinetic-gazebo-ros-pkgs \
                            ros-kinetic-gazebo-ros-control \
                            ros-kinetic-kobuki-gazebo \
                            apt-utils \
                            xvfb \
                            libignition-math2-dev \
                            vim \
                            wget \
                            curl \
                            python-catkin-tools

###############################################################################

# get a modified version of Gazebo that works on VMs
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
 && sudo apt-get update

ENV ROS_WS /home/mars/catkin_ws
WORKDIR ${ROS_WS}

# use a ROS install file to create a workspace
COPY pkgs.rosinstall pkgs.rosinstall
RUN sudo chown -R ${USER} . && \
    wstool init -j8 src pkgs.rosinstall

# install system dependencies
# --ignore-src (what does this do?)
RUN rosdep update \
 && rosdep install -i -y -r --from-paths src \
                        --ignore-src \
                        --skip-keys="python-rosdep python-catkin-pkg python-rospkg" \
                        --rosdistro="${ROS_DISTRO}" \
 && sudo apt-get clean \
 && sudo rm -rf /var/lib/apt/lists/*

# fix: https://github.com/ros/geometry/issues/144
# fix: https://github.com/ros-drivers/freenect_stack/issues/36
# fix BFL includes
RUN cd "${ROS_WS}/src/ros_comm/xmlrpcpp" && \
    sed -i "s#INCLUDE_DIRS include#INCLUDE_DIRS include include/xmlrpcpp#" CMakeLists.txt
RUN cd "${ROS_WS}/src/freenect_stack" && \
    find . -type f -exec sed -i "s#libfreenect/libfreenect.h#libfreenect.h#g" "{}" \; && \
    find . -type f -exec sed -i "s#libfreenect/libfreenect_registration.h#libfreenect_registration.h#g" "{}" \;
RUN cd "${ROS_WS}/src/navigation" && \
    find . -type f -exec sed -i "s#<bfl/#<#g" {} \;

# build
RUN sudo apt-get install -y libgazebo7-dev \
                            libignition-math2-dev \
                            ros-kinetic-gazebo-ros-control

# get a modified version of Gazebo that works on VMs
RUN sudo apt-get install -y libignition-math2-dev && \
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    sudo apt-get update && \
    sudo apt-get install -y gazebo7

RUN mkdir logs
# RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
#     catkin build kobuki_gazebo_plugins
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
 && catkin build

RUN sudo apt-get update \
 && sudo apt-get install -y python3-pip \
 && pip3 install --user \
      pyyaml \
      defusedxml \
      catkin_pkg \
      rospkg \
      netifaces

# add the entrypoint script
COPY turtletest/ turtletest/
COPY entrypoint.sh .
RUN sudo chown -R $(whoami) turtletest entrypoint.sh \
 && chmod +x entrypoint.sh
ENTRYPOINT ["/home/mars/catkin_ws/entrypoint.sh"]
CMD ["/bin/bash"]
