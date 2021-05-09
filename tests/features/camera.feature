Feature: Camera

# Tested Features: camera detection

# Acceptance Criteria: the simulation must receive the information that the person 
# was detected by the camera

# Map: tests/data/camera_map.drawio
# Map description: Consists of four patients room, three people around the map and three robots
# that can follow the different paths on the map to find these people  

Scenario: The camera receives information from the entities present in its field of view
    Given a map containing robots with camera components
    And a robot with id 'robot' that has a camera component
    And all the simulation robots has detection ability
    And a camera event for robot 'robot' detect a 'person1' that is in the camera field of view
    When after run simulation
    Then information about the entity 'person1' was detected by the 'robot' robot camera

Scenario: The camera does not receive information from entities that are not present in its field of view
    Given a map containing robots with camera components
    And a robot with id 'robot' that has a camera component
    And all the simulation robots has detection ability
    And a camera event for robot 'robot' detect a 'person2' that is not in the camera field of view
    When after run simulation
    Then information about the entity 'person2' was not detected by the 'robot' robot camera

Scenario: Multi-robot detects specific persons
    Given a map containing robots with camera components
    And a robot with id 'robot' that has a camera component
    And a robot with id 'robot2' that has a camera component
    And all the simulation robots has detection ability
    And a camera event for robot 'robot' detect a 'person1' that is in the camera field of view
    And a camera event for robot 'robot2' detect a 'person1' that is not in the camera field of view
    When after run simulation
    Then information about the entity 'person1' was detected by the 'robot' robot camera
    Then information about the entity 'person1' was not detected by the 'robot2' robot camera

Scenario: The robot detects more than one person
    Given a map containing robots with camera components
    And a robot with id 'robot' that has a camera component
    And all the simulation robots has detection ability
    And a camera event for robot 'robot' detect a 'person1' that is in the camera field of view
    And a camera event for robot 'robot' detect a 'person4' that is in the camera field of view
    When after run simulation
    Then information about the entity 'person1' was detected by the 'robot' robot camera
    Then information about the entity 'person4' was detected by the 'robot' robot camera
