Feature: Simple Scripted Command

# Tested Features: command scripts Go, Grab and Drop specifying the commands
# in drawio.

# Acceptance Criteria 
#   First Scenario: the robots are able to follow all the commands specified 
# on the drawio map and at the end, the medication must be in the correct room.

# Map: tests/data/med_and_patient_room_map_with_commands.drawio
# Map description: The map has three POIs (robotHome, medRoom and patientRoom), 
# and a pickable named medicine in the medicine room. The map also has a robot
# with this scripts: "Go medRoom", "Grab medicine", "Go patientRoom", "Drop medicine".

Scenario: the robot performs all scripted commands of the drawio map
    Given a map with a medicine room, a patient room and a robot with commands
    And the robot has the ability to use a script command
    And the robot has the ability to navigate
    When after run simulation
    Then the 'medicine' is in the 'patientRoom' poi