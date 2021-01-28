import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator

scenarios('../features/simulation.feature')

@given("a room of 10 x 10")
def room():
    pass

@given("a robot in 1,1")
def robot():
    pass

@when("the robot stay still")
def stay_still():
    pass

@then("the robot is in 1,1")
def check_position():
    pass