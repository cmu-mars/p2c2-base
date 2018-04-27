from typing import Tuple
import math


Position2D = Tuple[float, float] # (x, y)


def euclidean(a: Position2D, b: Position2D) -> float:
    """
    Computes the Euclidean distance between two positions described in the same
    two-dimensional frame.
    """
    assert isinstance(a, tuple) and isinstance(b, tuple)
    assert len(a) != []
    assert len(a) == len(b)
    d = sum((x - y) ** 2 for (x, y) in zip(a, b))
    return math.sqrt(d)


