Feature: Camera

Scenario: The camera receives information from the entities present in its field of view
    Given a map containing a robot with a camera
    And a robot with a camera component
    And an ability to detect other entities
    And a detectable entity named 'person1' in the cameras field of view
    And a camera event to detect 'person1'
    When after run simulation
    Then information about the entity 'person1' is detected by the camera
    # evento de detecao da pessoa
    # deteccao de informaçoes adicionais (posicao da pessoa)

Scenario: The camera does not receive information from entities that are not present in its field of view
    Given a map containing a robot with a camera 
    And a robot with a camera component
    And an ability to detect other entities
    And a detectable entity named 'person2' that isnt in the cameras field of view
    And a camera event to detect 'person2'
    When after run simulation
    Then information about the entity 'person2' is not detected by the camera