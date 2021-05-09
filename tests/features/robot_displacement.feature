Feature: Robot Displacement

# Tested Features: movement using Path, movement using GotoPos and movement using POI

# Acceptance Criteria - First Scenario: the robot is capable of following 
# the path and entering the room.
# Acceptance Criteria - Second Scenario: the robot is capable of go to a
# specific position given a GotoPos Event.

# Map: tests/data/three_room_map.drawio
# Map description: consists of three rooms, a robot and a path that leads
# the robot to the third room

Scenario: The robot moves along a path until it reaches its destination
    Given a map with three rooms
    And a robot with the ability to follow a path
    And a path from the robot to the center of 'room_three'
    When after run simulation
    Then the robot is in the center of 'room_three'

Scenario: The robot moves to a specific Position
    Given a map with three rooms
    And a robot with the ability to navigate
    And a gotoPos event to the center of 'room_three'
    When after run simulation
    Then the robot is in the center of 'room_three'

Scenario: The robot moves to a specific POI
    Given a map with three rooms
    And a POI named 'room_three_center' in the center of the room
    And a robot with the ability to navigate
    And a gotoPoi event to 'room_three_center'
    When after run simulation
    Then the robot is in the center of 'room_three'