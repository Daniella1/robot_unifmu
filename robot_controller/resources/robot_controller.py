import pickle
from fmi2 import Fmi2FMU, Fmi2Status
import numpy as np
import matplotlib.pyplot as plt
#import pandas as pd

class Controller(Fmi2FMU):
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

        #self.results = [[0.0,0.0,0.0]]


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
        self.previous_error = 0.0
        self.u = 0.0
        self.I = 0.0
        return Fmi2Status.ok

    def exit_initialization_mode(self) -> int:
        error = self.setpoint - self.theta
        self.u = self.Kp * error
        self.previous_error = error
        return Fmi2Status.ok

    
    def do_step(self, current_time, step_size, no_step_prior):
        error = self.setpoint - self.theta
        self.I = self.I + error * step_size
        D = (error - self.previous_error)/step_size
        self.u = self.Kp * error + self.Ki * self.I +  self.Kd * D
        self.previous_error = error

        self.results.append([self.u, self.theta, self.setpoint])

        return Fmi2Status.ok
        
    
    def terminate(self) -> int:
        # results = np.array(self.results)
        # u = results[:,0]
        # theta = results[:,1]
        # setpoint = results[:,2]

        # step_size = 0.001
        # t_eval = np.arange(0.0, 10.0 + step_size, step_size)
        # df = pd.DataFrame(list(zip(u, theta, setpoint, t_eval[:-1])), columns=["u","theta","setpoint","t_eval"])
        # df.to_csv("cosim_data")
        return Fmi2Status.ok




if __name__ == "__main__":

    theta = 0.0
    theta_setpoint = 1.0

    c = Controller()
    c.enter_initialization_mode()
    c.setpoint = theta_setpoint
    c.Kp = 1.0
    c.Ki = 0.0
    c.Kd = 0.01#0.0001
    c.exit_initialization_mode()

    step_size = 0.001
    t_eval = np.arange(0.0, 10.0 + step_size, step_size)
    results = [[c.u, c.theta]]
    for cur_time in t_eval[:-1]:
        c.do_step(cur_time, step_size, True)

        theta = theta + c.u * step_size # for testing controller
        c.theta = theta

        results.append([c.u, c.theta])

    results = np.array(results)
    c.terminate()

    plt.plot(t_eval, results[:,0], label="u(t)")
    plt.plot(t_eval, results[:,1], label="theta(t)")
    plt.axhline(theta_setpoint,label="setpoint", ls="--",c="r")
    plt.legend()
    plt.xlabel("t [s]")
    plt.title("Controller")
    plt.show()

    # data = pd.read_csv("./resources/cosim_data")
    # u = data["u"]
    # theta = data["theta"]
    # setpoint = data["setpoint"]
    # t_eval = data["t_eval"]

    # plt.plot(t_eval[1:], u[1:], label="u(t)")
    # plt.plot(t_eval[1:], theta[1:], label="theta(t)")
    # plt.plot(t_eval[1:], setpoint[1:],label="setpoint", ls="--",c="r")
    # plt.legend()
    # plt.xlabel("t [s]")
    # plt.title("Co-simulation results")
    # plt.show()
