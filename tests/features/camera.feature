Feature: Camera

Scenario: The camera receives information from the entities present in its field of view
    Given a map containing a robot with a camera
    And a robot with a camera
    And an ability to recognize other entities
    And a robot with the ability to navigate
    And a detectable entity named 'person1' in the cameras field of view
    And a camera event to capture detectable entities
    When after run simulation
    Then information about the entity 'person1' seen is captured by the camera

Scenario: The camera does not receive information from entities that are not present in its field of view
    Given a map containing a robot with a camera 
    And a robot with a camera
    And an ability to recognize other entities
    And a robot with the ability to navigate
    And a detectable entity named 'person2' that isnt in the cameras field of view
    And a camera event to capture detectable entities
    When after run simulation
    Then information about the entity 'person2' isnt captured by the camera