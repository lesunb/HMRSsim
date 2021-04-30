from simpy import Store
from simulator.typehints.component_types import Component


class ProximitySensor(Component):

    def __init__(self, sensor_range: float, sensor_type: str, reply_channel: Store = None):
        self.range = sensor_range
        self.type = sensor_type
        self.reply_channel = reply_channel

    def __str__(self):
        return f'{self.type}Sensor[range={self.range}; channel={self.reply_channel}]'
