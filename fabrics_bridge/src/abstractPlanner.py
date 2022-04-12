from abc import ABC, abstractmethod
import yaml

class PlannerSettingIncomplete(Exception):
    pass

class AbstractPlanner(ABC):

    def __init__(self, exp, setupFile, required_keys):
        self._exp = exp
        # pass the dictionary directly instead of reading from yaml
        self._setup = setupFile
        # self._setupFile = setupFile
        self._required_keys = required_keys
        self.parseSetup()

    @abstractmethod
    def reset(self):
        pass

    def parseSetup(self):
        # pass the dictionary directly instead of reading from yaml

        # with open(self._setupFile, "r") as setupStream:
        #     self._setup = yaml.safe_load(setupStream)
        self.checkCompleteness()

    def plannerType(self):
        return self._setup['type']

    def checkCompleteness(self):
        incomplete = False
        missingKeys = ""
        for key in self._required_keys:
            if key not in self._setup.keys():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise PlannerSettingIncomplete("Missing keys: %s" % missingKeys[:-2])

    @abstractmethod
    def setGoal(self, motionPlanningGoal):
        pass

    @abstractmethod
    def setJointLimits(self, lower_limits, upper_limits):
        pass

    @abstractmethod
    def setSelfCollisionAvoidance(self, r_body):
        pass

    @abstractmethod
    def setObstacles(self, obstacles, r_body):
        pass

    @abstractmethod
    def concretize(self):
        pass

    def save(self, folderPath):
        with open(folderPath + "/planner.yaml", 'w') as file:
            yaml.dump(self._setup, file)

    @abstractmethod
    def computeAction(self, q, qdot):
        pass
