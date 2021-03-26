Feature: Approximation

# robo segue o caminho e não encontra a pessoa, deixa o pickable onde achou e volta para o início do caminho
# robo segue o caminho e não encontra a pessoa, tenta novamente por x vezes e volta para o início do caminho caso não encontre
# robo segue o caminho e não encontra a pessoa, volta para o início do caminho e desiste.
# robo segue o caminho e não encontra a pessoa, tenta novamente por x vezes e caso não encontre, volta para o inicio do caminho e desiste
# robo segue o caminho e encontra a pessoa, envia uma notificação que encontrou a pessoa e volta para o início.

Scenario: The robot searches for a person and finds them
    Given a map containing a robot with a camera
    And a robot with a camera
    And an ability to detect other entities
    And a robot with approximation ability
    And a robot with the ability to navigate
    And a Go to command for the robot to pass through POIs 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, robotHome'
    And a detectable entity named 'person3' in the cameras field of view
    And a camera event to detect 'person3'
    When after run simulation
    Then the robot approximated the 'person3'

Scenario: The robot searches for a person and does not find them
    Given a map containing a robot with a camera
    And a robot with a camera
    And an ability to detect other entities
    And a robot with approximation ability
    And a robot with the ability to navigate
    And a Go to command for the robot to pass through POIs 'intersection1, intersection2, patientRoom2, patientRoom3, patientRoom4, robotHome'
    And a detectable entity named 'person2' that isnt in the cameras field of view
    And a camera event to detect 'person2'
    When after run simulation
    Then the robot did not approximated the 'person2'

#Scenario: The robot searches for a person, finds them and delivers a pickable
#    Given a map with a robot, a default path and a person
#    And a robot with a camera
#    And a robot with approach ability
#    And a robot with the ability to follow a script command
#    And a robot with the ability to grab pickables
#    And the robot has the ability to navigate
#    And the person is in the camera's field of view
#    And a script command 'Go to' 'medRoom' poi
#    And a script command 'Grab' 'medicine' pickable
#    And a command to find and approach the person
#    And a script command 'Drop' 'medicine' pickable to person
#    When after run simulation
#    Then the robot approached the person
#    Then the robot delivered the 'medicine' to person

# command Find 'person or robot'
