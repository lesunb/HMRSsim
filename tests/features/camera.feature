Feature: Camera

Scenario: The camera receives information from the entities present in its field of view
    Given a map containing a robot with a camera
    And a robot with a camera component
    And all the simulation robots has detection ability
    And a camera event to detect a 'person1' that is in the camera field of view
    When after run simulation
    Then information about the entity 'person1' is detected by the camera
    # evento de detecao da pessoa
    # deteccao de informaçoes adicionais (posicao da pessoa)

Scenario: The camera does not receive information from entities that are not present in its field of view
    Given a map containing a robot with a camera 
    And a robot with a camera component
    And all the simulation robots has detection ability
    And a camera event to detect a 'person2' that is not in the camera field of view
    When after run simulation
    Then information about the entity 'person2' is not detected by the camera