Feature: Seer

Scenario: A seer is created containing the simulation snapshots
    Given a map with three rooms
    And a robot with the ability to follow a path
    And a path from the robot to the center of 'room_three'
    And the simulation has a Seer system to record all snapshots
    When after run simulation
    Then all snapshots were recorded by the Seer system in a file 