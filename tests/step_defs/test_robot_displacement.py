import pytest
from pytest_bdd import scenarios, given, when, then
from main import Simulator
from tests.aux.create_simulation import CreateSimulation

scenarios('../features/robot_displacement.feature')

@given("a map", target_fixture="simulation") 
def map():
    simulation = CreateSimulation('simulator/resources/map/simulation.json')
    return simulation
    #fechar a simulacao

@given("a 'room3' as a POI in 2,2")
def room3_poi():
    pass #setar a posicao do room3   #como definir um poi no mapa?

@given("a path to 'room3'")
def path():
    pass

@when("the robot receives a move-to 'room3'")
def movement(simulation):
    simulation.run()

@when("after some time")
def pass_time():
    pass

@then("the robot is in 'room3'")
def check_position():
    #pegar o robot e verificar a posicao dele
    pass