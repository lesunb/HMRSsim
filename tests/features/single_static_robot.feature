Feature: Single Static Robot

# Tested Features: robot remains in the same position when
# there is no movement.

# Acceptance Criteria: the robot remains in the same place during the simulation

# Map: tests/data/one_room_map.drawio
# Map description: 'A map' has one room and a robot inside it.

Scenario: Single static robot
    Given a map with one room
    And a robot in position 1,1 inside the room
    When the robot stay still 
    Then the robot is in 1,1