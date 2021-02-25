Feature: Collision

# Tested Features: simple collision detection

# Acceptance Criteria: the simulation must receive a collision event 
# when the robot collides with the wall

# Map: tests/data/collidable_wall_map.drawio
# Map description: Consists of one room with colidable walls and
# a robot that has a path that collides with the wall

Scenario: A collision event is received when a collision occurs
    Given a map with a collidable wall
    And a robot with ability to collide with the wall
    And a robot with the ability to follow a path
    And a path from the robot to the collidable wall
    When after run simulation
    Then the robot collides with the wall

Scenario: Two collision events is received when two collisions occurs
    Given a map with a collidable wall
    And a robot with ability to collide with the wall
    And a robot with the ability to follow a path
    And a path from the robot to the collidable wall
    And a path from the robot to the second collidable wall
    When after run simulation
    Then the robot collides with the wall
    Then the robot collides with the second wall