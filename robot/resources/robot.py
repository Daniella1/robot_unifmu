import pickle
import sys
from fmi2 import Fmi2FMU, Fmi2Status
from scipy.integrate import solve_ivp
import numpy as np
from multiprocessing import Process, Queue
import roboticstoolbox as rtb
import spatialmath as sm


def plotting_func(queue, ok_queue): 
    #sys.stderr = None
    #sys.stdout = None

    # Make and instance of the Swift simulator and open it
    from roboticstoolbox.backends.Swift import Swift
    env = Swift(realtime=False)
    env.launch()

    # Make a robot model and set its joint angles to the ready joint configuration
    robot = rtb.models.UR5()
    robot.q = robot.qr
    robot.base = sm.SE3(0, 0, 0.8)# * sm.SE3.Rz(np.pi / 2)

    # Add the robot to the simulator
    env.add(robot)
    #ok_queue.put(None)

    while True:
        q_or_end = queue.get()
        if q_or_end is None:
            return
        robot.q, step_size = q_or_end
        # Step the simulator by step_size
        env.step(step_size)  


class Robot(Fmi2FMU):

    # Params:
    # tau -  Motor load caused by arm weight and end effector (torque)
    # V - voltage
    # output theta, start=theta0 - motor position
    # output i - current
    # output omega, start=omega0 - motor velocity
    # input u - Motor torque signal: between -1 and 1.


    def __init__(self,reference_to_attr=None) -> None:
        super().__init__(reference_to_attr)

        self.l = 1.0 # length of center of mass
        self.m = 5.0 #mass of the arm
        self.g = 9.81 # gravity

        self.b = 5.0 # Motor shaft friction
        self.K = 7.45 # torque coefficient
        self.R = 0.15 # Resistance
        self.L = 0.036 # Inductance
        self.V_abs = 12.0 # Motor voltage
        self.J = self.m * self.l**2 # Motor shaft and arm inertia

        self.k1 = self.m*self.g*self.l

        self.u = 0.0
        self.theta = 0.0
        self.omega = 0.0
        self.i = 0.0

        self.use_visualization = False       
        self.queue = None
        self.ok_queue = None
        self.p = None
        

    def __repr__(self):
        return "Robot"

    def serialize(self):

        bytes = pickle.dumps(
            (1) #  do nothing
        )
        return Fmi2Status.ok, bytes

    def deserialize(self, bytes) -> int:
        a = pickle.loads(bytes)

        return Fmi2Status.ok

    def terminate(self) -> int:
        if self.use_visualization:
            self.queue.put(None)
            
            self.p.join()
            self.p.close()

        return Fmi2Status.ok

    def enter_initialization_mode(self) -> int:
        self.theta = 0.0
        self.omega = 0.0
        self.i = 0.0

        if self.use_visualization:
            self.queue = Queue()
            self.ok_queue = Queue()
            self.p = Process(target=plotting_func, args=(self.queue,self.ok_queue,))
            self.p.start()
            #self.ok_queue.get()
            
        return Fmi2Status.ok

    def _update_outputs(self, current_time, step_size):
        y0 = np.array([self.theta, self.omega, self.i])
        t_start = current_time
        t_end = current_time + step_size
        t_span = (t_start, t_end)

    
        def f(t,y):        
            theta, omega, i = y

            tau = self.k1*np.cos(theta)
            domega = (self.K * i - self.b * omega - tau) / self.J
            di = (self.V - self.R * i - self.K * omega) / self.L
            dtheta = omega

            return dtheta, domega, di

        res = solve_ivp(f, t_span, y0, t_eval=t_span, max_step=step_size/10.0)


        self.theta, self.omega, self.i = res.y[:,-1]


    def do_step(self, current_time, step_size, no_step_prior):
        self.V = self.u*self.V_abs
        
        self._update_outputs(current_time, step_size)

        if self.use_visualization:
            self.queue.put(([0, -self.theta, 0, 0, 0, 0], step_size))
        
        return Fmi2Status.ok
        

if __name__ == "__main__":

    r = Robot()
    r.u = 0.0
    r.use_visualization = True
    r.enter_initialization_mode()
    
    step_size = 0.1
    t_eval = np.arange(0.0, 10.0 + step_size, step_size)
    results = [[r.theta,r.omega]]
    for cur_time in t_eval[:-1]:
        r.do_step(cur_time, step_size, True)
        results.append([r.theta, r.omega])

    #results = np.array(results)
    r.terminate()
