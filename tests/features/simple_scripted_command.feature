Feature: Simple Scripted Command

# Tested Features: command scripts Go, Grab and Drop specifying the commands
# in drawio.

# Acceptance Criteria 
#   First Scenario: the robots are able to follow all the commands specified 
# on the drawio map and at the end, the medication must be in the correct room.

# Map: tests/data/_map.drawio
# Map description:

Scenario: the robot performs all scripted commands of the drawio map
    Given a map with a medicine room, a patient room and a robot with commands
    When after run simulation
    Then the 'medicine' is in the 'patientRoom' poi
    # precisa ter as abilities