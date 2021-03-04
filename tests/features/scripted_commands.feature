Feature: Scripted Commands

# Tested Features: command scripts Go, Grab and Drop

# Acceptance Criteria 
#   First Scenario: the robot is capable of Go to a specific POI using a
# 'Go to' script command. 
#   Second Scenario: the robot is capable drag and drop a pickable in a
# specific POI.
#   Third Scenario: the robots are able to follow all the commands specified 
# on the drawio map and at the end, the medication must be in the correct room.

# Map for first and second scenario: tests/data/med_and_patient_room_map.drawio
# Map description: 

# Map for third scenario: tests/data/_map.drawio
# Map description:

Scenario: The robot Go to a POI using a script command
    Given a map with a medicine room and a patient room
    And a robot with the ability to follow a script command
    And a POI named 'medRoom'
    And a script command 'Go to medRoom' 
    When after run simulation
    Then the robot is in the medicine room

Scenario: The robot grab and drop a medicine using a script command
    Given a map with a medicine room and a patient room
    And a robot with the ability to follow a script command
    And a POI named 'medRoom'
    And a POI named 'patientRoom'
    And a pickable 'medicine' inside the medicine room
    And a script command 'Go to medRoom'
    And a script command 'Grab medicine'
    And a script command 'Go to patientRoom'
    And a script command 'Drop medicine'
    When after run simulation
    Then the medicine is in the patient room

Scenario: the robot performs all scripted commands of the drawio map
    Given a map with a medicine room and a patient room
    And a pickable 'medicine1' in the 'medRoom'
    And a pickable 'medicine2' in the 'medRoom'
    When after run simulation
    Then the 'medicine1' is in 'patientRoom1'
    Then the 'medicine2' is in 'patientRoom2'