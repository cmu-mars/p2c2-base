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

# remove astra_camera from the catkin workspace
# see: https://github.mit.edu/brass/cmu-robotics/issues/155
RUN rm -rf src/astra_camera

# install Bear
RUN cd /tmp \
 && wget https://github.com/rizsotto/Bear/archive/2.3.11.tar.gz \
 && tar -xf 2.3.11.tar.gz \
 && cd Bear-2.3.11 \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make all \
 && make install \
 && rm -rf /tmp/*

# build source code
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && bear catkin build

RUN apt-get update \
 && apt-get install -y software-properties-common \
 && add-apt-repository ppa:deadsnakes/ppa \
 && apt-get update \
 && apt-get install -y python3.6 sudo
RUN curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py \
 && python3.6 /tmp/get-pip.py
RUN pip3.6 install pyyaml rospkg catkin_pkg netifaces gcovr

# build tests
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && catkin config --cmake-args -DENABLE_TESTS=ON \
 && (catkin build --continue-on-failure --no-status --make-args tests || exit 0) \
 && catkin build --no-status

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
COPY _debug.launch .
COPY turtletest/ turtletest/

# install Kaskara binaries
COPY --from=squareslab/kaskara:0.0.3 /opt/kaskara /opt/kaskara
ENV PATH "/opt/kaskara/scripts:${PATH}"
