from abc import abstractmethod

class RosService(object):

    @abstractmethod
    def get_listener_callback(self):
        pass

    @abstractmethod
    def get_name(self):
        pass

    @abstractmethod
    def get_service_type(self):
        pass