Feature: Scripted Commands

# Tested Features: command scripts Go, Grab and Drop

# Acceptance Criteria 
#   First Scenario: the robot is capable of Go to a specific POI using a
# 'Go to' script command. 
#   Second Scenario: the robot is capable of drag and drop a pickable in a
# specific POI.
#   Third Scenario: the robot1 is capable of drag and drop a pickable and
# the robot2 is capable of go to a specific POI both in the same simulation.

# Map for first and second scenario: tests/data/med_and_patient_room_map.drawio
# Map description: 

Scenario: The robot Go to a POI using a script command
    Given a map with a medicine room and a patient room
    And all the simulation robots has the ability to follow a script command
    And all the simulation robots has the ability to navigate
    And a script command 'Go to' 'medRoom' poi for 'robot'
    When after run simulation
    Then the robot with id 'robot' is in the 'medRoom' poi

Scenario: The robot grab and drop a medicine using a script command
    Given a map with a medicine room and a patient room
    And all the simulation robots has the ability to follow a script command
    And a robot with id 'robot' has the ability to grab pickables
    And all the simulation robots has the ability to navigate
    And a script command 'Go to' 'medRoom' poi for 'robot'
    And a script command 'Grab' 'medicine' pickable for 'robot'
    And a script command 'Go to' 'patientRoom' poi for 'robot'
    And a script command 'Drop' 'medicine' pickable for 'robot'
    When after run simulation
    Then the 'robot' droped the 'medicine' in the 'patientRoom' poi

Scenario: Multi-robots using script command
    Given a map with a medicine room and a patient room
    And all the simulation robots has the ability to follow a script command
    And a robot with id 'robot' has the ability to grab pickables
    And all the simulation robots has the ability to navigate
    And a list of script commands 'Go medRoom, Grab medicine, Go patientRoom, Drop medicine' to robot with id 'robot'
    And a list of script commands 'Go patientRoom' to robot with id 'robot2'
    When after run simulation
    Then the 'robot' droped the 'medicine' in the 'patientRoom' poi
    And the robot with id 'robot2' is in the 'patientRoom' poi