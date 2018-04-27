"""
This module is used to provide data structures that describe the possible
outcomes of a test execution.
"""
from typing import Optional
import datetime


class TestOutcome(object):
    """
    Describes the outcome of a test execution.
    """
    def __init__(self,
                 passed: bool,
                 duration: datetime.timedelta,
                 distance_to_goal: Optional[float]
                 ) -> None:
        """
        Constructs a new test outcome.

        Parameters:
            duration: the wall-clock time taken to execute the test.
        """
        self.__passed = passed
        self.__duration = duration
        self.__distance_to_goal = distance_to_goal

    @property
    def duration(self) -> datetime.timedelta:
        """
        The wall-clock time taken to execute the test.
        """
        return self.__duration

    @property
    def distance_to_goal(self) -> Optional[float]:
        """
        The distance to the goal.
        """
        return self.__distance_to_goal

    @property
    def successful(self) -> bool:
        """
        A flag indicating whether or not the outcome of the test was determined
        to be a success.
        """
        return self.__passed


class Collided(TestOutcome):
    """
    The robot had a collision during the mission.
    """
    def __init__(self,
                 duration: datetime.timedelta,
                 distance_to_goal: float
                 ) -> None:
        super().__init__(False, duration, distance_to_goal)


class GoalReached(TestOutcome):
    """
    The robot reached the goal within the time limit.
    """
    def __init__(self,
                duration: datetime.timedelta,
                distance_to_goal: float
                ) -> None:
       super().__init__(True, duration, distance_to_goal)


class GoalNotReached(TestOutcome):
    """
    The robot failed to reach the goal within the time limit.
    """
    def __init__(self,
                duration: datetime.timedelta,
                distance_to_goal: float
                ) -> None:
       super().__init__(False, duration, distance_to_goal)
