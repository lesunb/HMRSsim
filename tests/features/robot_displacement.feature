Feature: Robot Displacement

Scenario: The robot moves to a specific Position
Given a map with three rooms
"""
Map consisting of three rooms, a robot and a path that leads the robot to
the third room.
tests/data/room3.drawio
"""
And a Position 2,2 in 'room3'
When the robot receives a move-to Position 2,2 in 'room3'
And after run simulation
Then the robot is in Position 2,2 of 'room3'

Scenario: The robot moves along a path until it reaches its destination
Given a map with three rooms
And a path to 'room3'
When the robot receives a move-to end of path in 'room3'
And after run simulation
Then the robot is in the end of path in 'room3'
