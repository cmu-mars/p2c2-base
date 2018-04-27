FROM cmu-mars/gazebo

ENV USER mars
ENV ROS_WS /home/mars/catkin_ws
WORKDIR ${ROS_WS}

# trash the workspace
RUN . /opt/ros/kinetic/setup.sh && \
    catkin_make clean && \
    rm -rf src/* build devel

# install basic utilities
RUN sudo apt-get update && \
    sudo apt-get install -y vim wget curl python-catkin-tools

# uninstall kobuki
RUN sudo apt-get remove -y ros-kinetic-kobuki \
                           ros-kinetic-kobuki-core

# use a ROS install file to create a workspace
ADD pkgs.rosinstall pkgs.rosinstall
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
RUN mkdir logs
# RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
#     catkin build kobuki_gazebo_plugins
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
 && catkin build

RUN sudo apt-get update \
 && sudo apt-get install -y python3-pip \
 && pip3 install --upgrade pip
RUN pip3 install --user \
      pyyaml \
      defusedxml \
      catkin_pkg \
      rospkg \
      netifaces

COPY turtletest/ .
RUN sudo chown -R $(whoami) turtletest
