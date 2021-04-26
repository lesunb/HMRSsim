Feature: Approximation


# Tested Features: approaching a person detected by the camera 

# Acceptance Criteria: the simulation must receive the information that the robot approached the person 

# Map: tests/data/camera_map.drawio
# Map description: Consists of four patients room, three people around the map and three robots
# that can follow the different paths on the map to find these people  


Scenario: The robot searches for a person and finds them
    Given a map containing robots with camera components
    And a robot with id 'robot' that has a camera component
    And all the simulation robots has detection ability
    And all the simulation robots has approximation ability
    And all the simulation robots has the ability to navigate
    And the 'robot' robot pass through POIs 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, intersection2, robotHome'
    And a camera event for robot 'robot' detect a 'person3' that is in the camera field of view
    When after run simulation
    Then the 'robot' approximated the 'person3'

Scenario: The robot searches for a person and does not find them
    Given a map containing robots with camera components
    And a robot with id 'robot' that has a camera component
    And all the simulation robots has detection ability
    And all the simulation robots has approximation ability
    And all the simulation robots has the ability to navigate
    And the 'robot' robot pass through POIs 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, intersection2, robotHome'
    And a camera event for robot 'robot' detect a 'person2' that is not in the camera field of view
    When after run simulation
    Then the 'robot' did not approximated the 'person2'

Scenario: Multi-robots searches for specifics persons
    Given a map containing robots with camera components
    And a robot with id 'robot' that has a camera component
    And a robot with id 'robot2' that has a camera component
    And a robot with id 'robot3' that has a camera component
    And all the simulation robots has detection ability
    And all the simulation robots has approximation ability
    And all the simulation robots has the ability to navigate
    And the 'robot' robot pass through POIs 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, intersection2, robotHome'
    And the 'robot2' robot pass through POIs 'intersection1, intersection2, patientRoom1' 
    And the 'robot3' robot pass through POIs 'patientRoom3'
    And a camera event for robot 'robot' detect a 'person3' that is in the camera field of view
    And a camera event for robot 'robot2' detect a 'person2' that is in the camera field of view
    And a camera event for robot 'robot3' detect a 'person1' that is not in the camera field of view
    When after run simulation
    Then the 'robot' approximated the 'person3'
    Then the 'robot2' approximated the 'person2'
    Then the 'robot3' did not approximated the 'person1'

Scenario: The robot approaches all detected people
    Given a map containing robots with camera components
    And a robot with id 'robot' that has a camera component
    And all the simulation robots has detection ability
    And all the simulation robots has approximation ability
    And all the simulation robots has the ability to navigate
    And the 'robot' robot pass through POIs 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, intersection2, robotHome'
    And a camera event for robot 'robot' detect a 'person3' that is in the camera field of view
    And a camera event for robot 'robot' detect a 'person1' that is in the camera field of view
    When after run simulation
    Then the 'robot' approximated the 'person3'
    Then the 'robot' approximated the 'person1'