import numpy as np

class MechanicalSystem:
    def __init__(self, st):
        """
        Initialize the mechanical system with a given sampling period.
        """
        self.st = st  # Sampling period
        self.x = np.zeros(2)  # State vector
        self.x0 = np.zeros(2)  # Initial state vector
        self.order = 2  # System order
        self.num_input = 1  # Number of system inputs
        self.num_output = 1  # Number of system outputs
        self.u0 = np.zeros(self.num_input)  # Initial input values
        self.u = self.u0.copy()  # Current input values
        self.sigma_y = 0  # Noise standard deviation
        self.t = 0  # Time variable
        self.umax = np.array([5])  # Maximum input value
        self.scenario = 1  # Scenario identifier
        
        # Define output and input names
        self.output_names = [f'output_{i+1}' for i in range(self.num_output)]
        self.input_names = [f'input_{i+1}' for i in range(self.num_input)]
    
    def initialize(self):
        """
        Reset the system to its initial conditions.
        """
        self.x = self.x0.copy()
        self.u = self.u0.copy()
        self.t = 0
    
    def set_scenario(self, scenario):
        """
        Set the current scenario for the system.
        """
        self.scenario = scenario
    
    def write_actuator_value(self, u):
        """
        Update the actuator input value.
        """
        self.u = np.array(u)
    
    def read_actuator_value(self):
        """
        Read the current actuator input value.
        """
        return self.u
    
    def read_sensor_value(self):
        """
        Read the sensor output value.
        """
        return self.output_function()
    
    def simulate(self):
        """
        Solve dx/dt = f(x, u) over the sampling period using numerical integration.
        """
        usat = self.saturation_control_action(self.u)  # Apply saturation control
        n = 10  # Number of integration steps
        dt = self.st / n  # Time step for integration
        for i in range(n):
            self.ode_solver(usat, dt, dt * (i + 1))
    
    def get_sampling_period(self):
        """
        Return the system's sampling period.
        """
        return self.st
    
    def get_output_number(self):
        """
        Return the number of system outputs.
        """
        return self.num_output
    
    def get_input_number(self):
        """
        Return the number of system inputs.
        """
        return self.num_input
    
    def get_umax(self):
        """
        Return the maximum allowed input value.
        """
        return self.umax
    
    def get_output_names(self):
        """
        Return the names of the system outputs.
        """
        return self.output_names
    
    def get_input_names(self):
        """
        Return the names of the system inputs.
        """
        return self.input_names
    
    def show(self):
        """
        Display information about the system's inputs and outputs.
        """
        print(f'This system has {self.num_output} outputs:')
        for name in self.output_names:
            print(f'- {name}')
        
        print(f'This system has {self.num_input} inputs:')
        for i, name in enumerate(self.input_names):
            print(f'- {name}, with maximum limit = {self.umax[i]}')
    
    def state_function(self, x, u, t):
        """
        Define the state transition function dx/dt = f(x, u).
        """
        return np.zeros_like(x)
    
    def output_function(self):
        """
        Define the system output function.
        """
        return self.x + self.sigma_y * np.random.randn(len(np.atleast_1d(self.sigma_y)))
    
    def ode_solver(self, u, st, t):
        """
        Solve the system's differential equation using the Runge-Kutta 4th Order method.
        """
        k1 = self.state_function(self.x, u, t)
        k2 = self.state_function(self.x + 0.5 * st * k1, u, t)
        k3 = self.state_function(self.x + 0.5 * st * k2, u, t)
        k4 = self.state_function(self.x + st * k3, u, t)
        self.x += (1/6) * (k1 + 2*k2 + 2*k3 + k4) * st
    
    def saturation_control_action(self, u):
        """
        Apply saturation limits to the control input.
        """
        usat = np.clip(u, -self.umax, self.umax)
        return usat
