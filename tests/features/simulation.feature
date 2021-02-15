Feature: Simulation

Scenario: Single static robot
    Given a room
    """
    A map with a room and a robot inside it.
    tests/data/room10x10.drawio
    """
    And a robot in 1,1
    When the robot stay still
    Then the robot is in 1,1