import pytest
from pytest_bdd import scenarios, given, when, then

scenarios('../features/collision.feature')

@given("a map with a colliding wall", target_fixture="simulation")
def map_with_colliding_wall():
    pass

@given("a robot with a path that collides with the wall")
def robot_with_path_that_collides_with_wall():
    pass

@when("the robot follows the path ")
def robot_follows_the_path():
    pass

@when("collides with the wall")
def collides_with_the_all():
    pass

@then("receive a collision event saying it suffered the collision")
def receive_collision_event():
    pass