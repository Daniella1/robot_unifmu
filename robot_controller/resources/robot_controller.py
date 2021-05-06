import pickle
from fmi2 import Fmi2FMU, Fmi2Status
import numpy as np
import matplotlib.pyplot as plt

class RobotController(Fmi2FMU):
    def __init__(self,reference_to_attr=None) -> None:
        super().__init__(reference_to_attr)
        self.theta = 0.0
        self.i = 0.0
        self.omega = 0.0

        self.previous_error = 0.0
        self.setpoint = 0.0
        self.Kp = 0.0
        self.Kd = 0.0
        self.Ki = 0.0
        self.I = 0.0


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
        self.I = 0.0
        self.Kp = 1
        self.Kd = 0.1
        self.Ki = 0.0
    
    def do_step(self, current_time, step_size, no_step_prior):
        
        error = self.setpoint - self.theta
        self.I = self.I + error * step_size
        D = (error - self.previous_error)/step_size
        self.u = self.Kp * error + self.Ki * self.I +  self.Kd * D 
        self.previous_error = error

        return Fmi2Status.ok
        



if __name__ == "__main__":

    r = RobotController()
    r.enter_initialization_mode()
    r.setpoint = np.pi/2
    r.exit_initialization_mode()
    
    step_size = 0.1
    t_eval = np.arange(0.0, 10.0 + step_size, step_size)
    results = [r.u]
    for cur_time in t_eval[:-1]:
        r.do_step(cur_time, step_size, True)
        results.append(r.u)

    results = np.array(results)
    r.terminate()

    plt.plot(t_eval, results)
    plt.show()