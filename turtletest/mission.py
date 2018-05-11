from typing import Callable
import datetime
import subprocess
import math
import sys
import time
import os

import rospy
import roslaunch
import actionlib
import actionlib_msgs.msg
import move_base_msgs.msg
import kobuki_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

import helper
import outcome
import launch


class Mission(object):
    """
    * headless: bool
    * simulator: enum [gazebo, stage]

    TODO:
        allow simulator to be changed.
    """
    def __init__(self,
                 map_position_initial: helper.Position2D,
                 map_position_end: helper.Position2D,
                 base_launch_file: str,
                 map_file: str,
                 world_file: str,
                 time_limit: datetime.timedelta,
                 map_to_world: Callable[[helper.Position2D], helper.Position2D],
                 distance_threshold: float = 0.2
                 ) -> None:
        """
        Constructs a new mission.

        Parameters:
            base_launch: the path to the base launch file for this mission.
            pose_initial: the initial pose of the robot.
            time_limit: the length of time that the robot should be given to
                complete the mission.
            map_to_world: a function that transforms a position in the map
                coordinate frame to one in the world frame.
        """
        assert os.path.isfile(map_file), \
            "failed to find map file: {}".format(map_file)
        assert os.path.isfile(world_file), \
            "failed to find world file: {}".format(world_file)
        assert os.path.isfile(base_launch_file), \
            "failed to find base launch file: {}".format(base_launch_file)

        self.__time_limit = time_limit
        self.__map_to_world = map_to_world
        self.__map_file = map_file
        self.__world_file = world_file
        self.__base_launch_file = base_launch_file
        self.__map_position_initial = map_position_initial
        self.__expected_map_position_end = map_position_end
        self.__distance_threshold = distance_threshold

    @property
    def world_file(self) -> str:
        """
        The absolute path to the world file describing the simulated world in
        which this mission should take place.
        """
        return self.__world_file

    @property
    def map_file(self) -> str:
        """
        The absolute path to the map file that describes the robot's knowledge
        of its surroundings.
        """
        return self.__map_file

    @property
    def time_limit(self) -> datetime.timedelta:
        """
        The length of time that the robot is given to complete the mission. If
        the robot fails to complete the mission within the time limit, then the
        mission is considered to have failed.
        """
        return self.__time_limit

    @property
    def map_position_initial(self) -> helper.Position2D:
        """
        The initial position of the robot, described in the "map" frame.
        """
        return self.__map_position_initial

    @property
    def expected_map_position_end(self) -> helper.Position2D:
        """
        The expected position of the robot, within the 'map' frame, after
        completing the mission.
        """
        return self.__expected_map_position_end

    @property
    def expected_world_position_end(self) -> helper.Position2D:
        """
        The expected position of the robot, within the 'world' frame provided
        by the simulator, after completing the mission.
        """
        return self.__map_to_world(self.expected_map_position_end)

    def execute(self):
        """
        Executes this test in simulation and returns a summary of its outcome.
        """
        # callback used to listen for collision events
        # TODO: terminate immediately
        collided = False
        def collision_checker(event) -> None:
            if event.state == 1:
                rospy.loginfo('BUMPER: We hit something!')
                collided = True

        # FIXME timeout case
        def get_believed_position() -> helper.Position2D:
            odom = rospy.client.wait_for_message("odom",
                                                 nav_msgs.msg.Odometry,
                                                 timeout=1.0)
            point = odom.pose.pose.position
            return (point.x, point.y)

        def get_world_position() -> helper.Position2D:
            odom = rospy.client.wait_for_message("/base_pose_ground_truth",
                                                 nav_msgs.msg.Odometry,
                                                 timeout=1.0)
            pose = odom.pose.pose

            # GAZEBO CODE:
            # ROBOT_MODEL_NAME = 'mobile_base'
            # model_states = \
            #     rospy.client.wait_for_message("/gazebo/model_states",
            #                                   gazebo_msgs.msg.ModelStates,
            #                                   timeout=1.0)
            # pose = model_states.pose[model_states.name.index(ROBOT_MODEL_NAME)]

            point = pose.position # type: geometry_msgs.msg.Point
            return (point.x, point.y)

        # TODO compute parameters
        launch_parameters = {}
        launch_file = launch.EphemeralLaunchFile(self.__base_launch_file,
                                                 launch_parameters)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launcher = roslaunch.parent.ROSLaunchParent(uuid,
                                                    [launch_file.path],
                                                    is_core=True)

        try:
            launcher.start()

            rospy.init_node('mission_controller')
            rospy.loginfo('Launched mission_controller node')
            rospy.Subscriber('/mobile_base/events/bumper',
                             kobuki_msgs.msg.BumperEvent,
                             collision_checker, callback_args=[self])
            client = actionlib.SimpleActionClient('move_base',
                                                  move_base_msgs.msg.MoveBaseAction)

            # TODO: timeout
            while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
                rospy.loginfo("Waiting for move_base client action server to initialise")

            # specify the goal for move_base
            goal_x, goal_y = self.expected_map_position_end
            goal = move_base_msgs.msg.MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = \
                geometry_msgs.msg.Point(goal_x, goal_y, 0.0)
            goal.target_pose.pose.orientation = \
                geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)

            # TODO is this simulated time or wall clock time?
            rospy.loginfo('Sending goal information...')
            time_start = rospy.get_time()
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(self.time_limit))
            time_end = rospy.get_time()
            time_elapsed = time_end - time_start
            goal_reported_as_reached = \
                client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED

            # determine distance to the goal
            actual_believed_position_end = get_believed_position()
            actual_world_position_end = get_world_position()
            distance_to_goal = helper.euclidean(actual_world_position_end,
                                                self.expected_world_position_end)

            rospy.loginfo("Final believed position: %s",
                          actual_believed_position_end)
            rospy.loginfo("Final ground-truth position: %s",
                          actual_world_position_end)
            rospy.loginfo("Distance to goal: %.2f units", distance_to_goal)

            if collided:
                return outcome.Collision(time_elapsed, distance_to_goal)
            if distance_to_goal < self.__distance_threshold:
                return outcome.GoalReached(time_elapsed, distance_to_goal)

            return outcome.GoalNotReached(time_elapsed, distance_to_goal)

        finally:
            launcher.shutdown()
