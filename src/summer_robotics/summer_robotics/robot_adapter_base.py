from abc import ABC, abstractmethod


class RobotAdapterBase(ABC):

    @abstractmethod
    def move_to_pose(self, x, y, z):
        pass

    @abstractmethod
    def pick(self, object_id):
        pass

    @abstractmethod
    def place(self, object_id):
        pass

    @abstractmethod
    def go_home(self):
        pass

    @abstractmethod
    def publish_busy(self, command):
        pass

    @abstractmethod
    def publish_done(self, command):
        pass
