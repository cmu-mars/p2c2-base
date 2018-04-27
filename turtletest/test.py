#!/usr/bin/env python
from mission import Mission
from outcome import GoalReached
from helper import Position2D
import sys
import os


def map_to_world(pos: Position2D) -> Position2D:
    map_x, map_y = pos
    world_x = map_x - 40.0
    world_y = map_y + 26.5
    return (world_x, world_y)


def build_mission(start_x: float,
                  start_y: float,
                  end_x: float,
                  end_y: float
                  ) -> Mission:
    pos_start = (start_x, start_y)
    pos_target = (end_x, end_y)

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

    mission = Mission(map_position_initial=pos_start,
                      map_position_end=pos_target,
                      base_launch_file=base_launch_file,
                      map_to_world=map_to_world,
                      map_file=map_file,
                      world_file=world_file,
                      time_limit=90)
    return mission


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("USAGE: ./test.py [test-id]")
        exit(1)

    test_id = sys.argv[1]

    tests = {
        't1': build_mission(0.0, 0.0, 2.0, 1.0),
        't2': build_mission(0.0, 0.0, 3.0, -1.0),
        't3': build_mission(0.0, 0.0, 0.0, -1.0)
    }

    if test_id not in tests:
        print("ERROR: test not found: {}".format(test_id))
        exit(1)

    test = tests[test_id]
    outcome = test.execute()
    print("OUTCOME: {}".format(outcome))

    if isinstance(outcome, GoalReached):
        print("SUCCESS")
        exit(0)

    else:
        print("FAILURE")
        exit(1)
