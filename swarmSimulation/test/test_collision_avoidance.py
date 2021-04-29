from swarmSimulation.systems.CollisionAvoidance import intercept
from simulator.components.Position import Position


def test_collision():
    a = Position(10, 10, 0, 5, 5)
    b = Position(10, 20, 0, 5, 5)
    assert intercept(a, b, 2) is False
    b = Position(10, 0, 0, 5, 5)
    assert intercept(a, b, 2) is False
    b = Position(0, 10, 0, 5, 5)
    assert intercept(a, b, 2) is False
    b = Position(20, 10, 0, 5, 5)
    assert intercept(a, b, 2) is False
    b = Position(10, 10, 0, 5, 5)
    assert intercept(a, b, 2) is True
    b = Position(17, 10, 0, 5, 5)
    assert intercept(a, b, 2) is True
    assert intercept(a, b, 1) is False
    b = Position(5, 5, 0, 5, 5)
    assert intercept(a, b, 0) is True
