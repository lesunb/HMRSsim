Feature: Simulation

Scenario: Single static robot
    Given a room of 10 x 10
    And a robot in 1,1
    When the robot stay still
    Then the robot is in 1,1