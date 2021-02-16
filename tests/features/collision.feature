Feature: Collision

Scenario: A collision event is received when a collision occurs
    Given a map with a colliding wall
    And a robot with a path that collides with the wall
    When the robot follows the path 
    And collides with the wall
    Then receive a collision event saying it suffered the collision
