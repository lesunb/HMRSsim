Feature: Scripted Commands

# Tested Features: command scripts Go, Grab and Drop

# Acceptance Criteria 
#   First Scenario: the robot is capable of Go to a specific POI using a
# 'Go to' script command. 
#   Second Scenario: the robot is capable drag and drop a pickable in a
# specific POI.

# Map for first and second scenario: tests/data/med_and_patient_room_map.drawio
# Map description: 

Scenario: The robot Go to a POI using a script command
    Given a map with a medicine room and a patient room
    And a robot with the ability to follow a script command
    And a script command 'Go to' 'medRoom' poi
    When after run simulation
    Then the robot is in the 'medRoom' poi

Scenario: The robot grab and drop a medicine using a script command
    Given a map with a medicine room and a patient room
    And a robot with the ability to follow a script command
    And a robot with the ability to grab pickables
    And a script command 'Go to' 'medRoom' poi
    And a script command 'Grab' 'medicine' pickable
    And a script command 'Go to' 'patientRoom' poi
    And a script command 'Drop' 'medicine' pickable
    When after run simulation
    Then the 'medicine' is in the 'patientRoom' poi
