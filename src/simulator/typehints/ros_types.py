from abc import abstractmethod

class RosService(object):

    @abstractmethod
    def process(self):
        """
        This is executed in each step the Ros control system is executed
        """
        pass

class RosActionServer(RosService):

    @abstractmethod
    def get_result_callback(self):
        pass

    @abstractmethod
    def get_handle_accepted_goal_callback(self):
        pass

    @abstractmethod
    def get_name(self):
        pass

    @abstractmethod
    def get_service_type(self):
        pass