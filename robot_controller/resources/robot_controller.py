import pickle
from fmi2 import Fmi2FMU, Fmi2Status

class RobotController(Fmi2FMU):
    def __init__(self,reference_to_attr=None) -> None:
        super().__init__(reference_to_attr)
        self.theta = 0.0
        self.i = 0.0
        self.omega = 0.0

        self._update_outputs()

        

    def __repr__(self):
        return "Adder"

    def serialize(self):

        bytes = pickle.dumps(
            (
                self.u
            )
        )
        return Fmi2Status.ok, bytes

    def deserialize(self, bytes) -> int:
        (
            u
        ) = pickle.loads(bytes)
        self.u = u

        return Fmi2Status.ok
    
    def enter_initialization_mode(self) -> int:
        self.u = 0.0

    def _update_outputs(self):
        pass

    def do_step(self, current_time, step_size, no_step_prior):
        
        self._update_outputs()

        return Fmi2Status.ok
        

