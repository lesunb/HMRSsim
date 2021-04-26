Feature: Collision

# Tested Features: simple collision detection

# Acceptance Criteria: the simulation must receive a collision event 
# when the robot collides with the wall

# Map: tests/data/collidable_wall_map.drawio
# Map description: Consists of one room with colidable walls and
# two robots that has a path that collides with the wall

Scenario: A collision event is received when a collision occurs
    Given a map with a collidable wall
    And all the simulations robots has the ability to collide with the wall
    And all the simulation robots has the ability to follow a path
    And the 'robot' has a collision component that stores the collision events
    And a path from the 'robot' to the 'collidable_wall'
    When after run simulation
    Then the 'robot' collides with the 'collidable_wall'

Scenario: Two collision events is received when two collisions occurs
    Given a map with a collidable wall
    And all the simulations robots has the ability to collide with the wall
    And all the simulation robots has the ability to follow a path
    And the 'robot' has a collision component that stores the collision events
    And a path from the 'robot' to the 'collidable_wall'
    And a path from the 'robot' to the 'second_collidable_wall'
    When after run simulation
    Then the 'robot' collides with the 'collidable_wall'
    And the 'robot' collides with the 'second_collidable_wall'

Scenario: Two collision events is received when two differents robots collides with a wall
    Given a map with a collidable wall
    And all the simulations robots has the ability to collide with the wall
    And all the simulation robots has the ability to follow a path
    And the 'robot' has a collision component that stores the collision events
    And the 'robot2' has a collision component that stores the collision events
    And a path from the 'robot' to the 'collidable_wall'
    And a path from the 'robot2' to the 'second_collidable_wall'
    When after run simulation
    Then the 'robot' collides with the 'collidable_wall'
    And the 'robot2' collides with the 'second_collidable_wall'