#!/usr/bin/env python3
#
# Provided we trust the bump sensors, we can subscribe to /mobile_base/events/bumper
# to be informed of any bumps. For now, let's go ahead and trust this (then work
# on a better, more general solution for cases where we don't trust the bump
# sensor OR where the bump sensor is inadequate).
#
# * subscribe to bumper event topic
# * terminate test as soon as we collide
#
# Outcomes
#
# | Collision
# | ReachedGoal(time, distance, accuracy, power*) [should record node failures]
# | SystemCrashed
# | PowerExpired
#
import sys
import os

import helper
import mission


# TODO: accept the location of a JSON file as input
if __name__ == "__main__":
    pos_start = (0.0, 0.0)
    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    pos_target = (target_x, target_y)

    def map_to_world(pos: helper.Position2D) -> helper.Position2D:
        map_x, map_y = pos
        world_x = map_x - 40.0
        world_y = map_y + 26.5
        return (world_x, world_y)

    # build the mission
    # TODO: parameterise
    dir_catkin = '/home/mars/catkin_ws'
    dir_turtlebot_gazebo = os.path.join(dir_catkin,
                                        'src/turtlebot_simulator/turtlebot_gazebo')
    map_file = os.path.join(dir_turtlebot_gazebo,
                            'maps/playground.yaml')
    world_file = os.path.join(dir_turtlebot_gazebo,
                              'worlds/playground.world')
    base_launch_file = os.path.join(os.path.dirname(__file__),
                                    'robotest.launch')

    mission = mission.Mission(map_position_initial=pos_start,
                              map_position_end=pos_target,
                              base_launch_file=base_launch_file,
                              map_to_world=map_to_world,
                              map_file=map_file,
                              world_file=world_file,
                              time_limit=60)

    # execute!
    print(mission.execute())
