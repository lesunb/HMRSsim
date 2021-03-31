Feature: Approximation

Scenario: The robot searches for a person and finds them
    Given a map containing a robot with a camera
    And a robot with a camera component
    And an ability to detect other entities
    And a robot with approximation ability
    And a robot with the ability to navigate
    And a Go to command for the robot to pass through POIs 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, intersection2, robotHome'
    And a camera event to detect a 'person3' that is in the camera field of view
    When after run simulation
    Then the robot approximated the 'person3'

Scenario: The robot searches for a person and does not find them
    Given a map containing a robot with a camera
    And a robot with a camera component
    And an ability to detect other entities
    And a robot with approximation ability
    And a robot with the ability to navigate
    And a Go to command for the robot to pass through POIs 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, intersection2, robotHome'
    And a camera event to detect a 'person2' that is not in the camera field of view
    When after run simulation
    Then the robot did not approximated the 'person2'

