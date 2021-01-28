Feature: Robot Displacement

Scenario: The robot moves to a specific POI
Given a map
And a 'room3' as a POI in 2,2
When the robot receives a move-to 'room3'
And after some time
Then the robot is in 'room3'

Scenario: The robot moves along a path until it reaches its destination
Given a map
And a path to 'room3'
When the robot receives a move-to 'room3'
And after some time
Then the robot is in 'room3'
