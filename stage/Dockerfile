FROM ros:indigo

# install core utilities
WORKDIR /ros_ws
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      apt-utils \
      xvfb \
      vim \
      wget \
      curl \
      python-catkin-tools \
      python-rosinstall-generator \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# install ROS dependencies
# https://github.com/intel-ros/realsense/issues/63
RUN rosinstall_generator turtlebot_stage --deps --rosdistro "${ROS_DISTRO}" > pkgs.rosinstall
RUN wstool init -j8 src pkgs.rosinstall
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && apt-get update \
 && rosdep update \
 && rosdep install -i -y -r --from-paths src \
                        --ignore-src \
                        --skip-keys="python-rosdep python-catkin-pkg python-rospkg" \
                        --rosdistro="${ROS_DISTRO}" \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# build source code
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && catkin build

# add entrypoint
ENV ROS_WSPACE /ros_ws
WORKDIR "${ROS_WSPACE}"
RUN echo "#!/bin/bash \n\
set -e \n\
source \"/opt/ros/\${ROS_DISTRO}/setup.bash\" \n\
source \"${ROS_WSPACE}/devel/setup.bash\" \n\
exec \"\$@\"" > "${ROS_WSPACE}/entrypoint.sh" \
 && chmod +x "${ROS_WSPACE}/entrypoint.sh"
ENTRYPOINT ["/ros_ws/entrypoint.sh"]
CMD ["/bin/bash"]

ENV TURTLEBOT_STAGE_MAP_FILE /ros_ws/src/turtlebot_simulator/turtlebot_stage/maps/maze.yaml
ENV TURTLEBOT_STAGE_WORLD_FILE /ros_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/maze.world
COPY turtlebot_in_stage.launch .
