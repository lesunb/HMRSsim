from abc import abstractmethod

class RosService(object):

    @abstractmethod
    def get_name(self):
        pass

    @abstractmethod
    def process(self):
        """
        This is executed in each step the Ros control system is executed
        """
        pass

class RosTopicServer(RosService):

    @abstractmethod
    def get_listener_callback(self):
        pass


class RosActionServer(RosService):

    @abstractmethod
    def get_execute_callback(self):
        # TODO Rename this to get_execute_callback
        pass

    @abstractmethod
    def get_goal_callback(self):
        pass

    @abstractmethod
    def get_handle_accepted_goal_callback(self):
        pass

    @abstractmethod
    def get_cancel_callback(self):
        pass

    @abstractmethod
    def get_service_type(self):
        pass