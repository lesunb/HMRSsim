Feature: Single Static Robot

# Tested Features: robot remains in the same position when
# there is no movement.

# Acceptance Criteria: the robot remains in the same place during the simulation

# Map: tests/data/one_room_map.drawio
# Map description: 'A map' has one room and a robot inside it.

Scenario: Single static robot
    Given a map with one room
    And a robot with id 'robot' in position '1,1' inside the room
    When the robot stay still 
    Then the robot with id 'robot' is in '1,1'

Scenario: Multi static robots
    Given a map with one room
    And a robot with id 'robot' in position '1,1' inside the room
    And a robot with id 'robot2' in position '25,25' inside the room
    And a robot with id 'robot3' in position '42,42' inside the room
    When the robot stay still
    Then the robot with id 'robot' is in '1,1'
    Then the robot with id 'robot2' is in '25,25'
    Then the robot with id 'robot3' is in '42,42'