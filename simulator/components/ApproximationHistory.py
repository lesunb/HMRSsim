from typehints.component_types import Component


class ApproximationHistory(Component):

    def __init__(self, target_id: int):
        self.target_id = target_id
        self.destiny_position = None
        self.entity_final_approx_pos = None
        self.approximated = False

    def __str__(self):
        return f'ApproximationHistory[target_id={self.target_id}, destiny_position={self.destiny_position}, entity_final_approx_pos={self.entity_final_approx_pos}]'
