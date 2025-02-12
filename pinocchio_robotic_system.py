import numpy as np
import pinocchio as pin
import yaml
from mechanical_system import MechanicalSystem  # Import the base class

class PinocchioRoboticSystem(MechanicalSystem):
    def __init__(self, st, model_name):
        """
        Initialize the Pinocchio-based robotic system with a given sampling period and model.
        """
        super().__init__(st)  # Call parent constructor
        
        # Load model from URDF file
        self.model = pin.buildModelFromUrdf(f'+{model_name}/model.urdf')
        self.data = self.model.createData()
        
        # Load simulation parameters from YAML file
        with open(f'+{model_name}/config.yaml', 'r') as file:
            config = yaml.safe_load(file)
        
        # Extract parameters from YAML configuration
        self.K = np.array(config['K'])  # Joint stiffness matrix
        self.D = np.array(config['D'])  # Joint damping matrix
        self.motor_inertia = np.diag(np.array(config['motor_inertia']))  # Motor inertia
        self.motor_viscous_term = np.array(config['motor_viscous_term'])  # Motor viscous friction
        self.motor_coulomb_term = np.array(config['motor_coulomb_term'])  # Motor Coulomb friction
        self.umax = np.array(config['umax'])  # Maximum control input
        self.x0 = np.array(config['x0'])  # Initial state
        noise = np.array(config['noise'])  # Measurement noise
        self.u = np.array(config['u0'])  # Control input initialization
        
        # Define joint and control-related properties
        self.njoints = int(self.model.nq)  # Number of joints
        self.n_controlled_joints = self.njoints  # Number of controlled joints
        self.num_input = self.n_controlled_joints
        self.num_output = 2 * self.n_controlled_joints
        self.order = self.njoints * 2  # System order
        
        # Define transformation matrices
        self.stateToOutput = np.block([
            [np.zeros((self.njoints, self.njoints)), np.eye(self.njoints), np.zeros((self.njoints, 2 * self.njoints))],
            [np.zeros((self.njoints, 3 * self.njoints)), np.eye(self.njoints)]
        ])
        
        self.u0 = np.zeros(self.n_controlled_joints)  # Initial control action
        self.u = self.u0.copy()
        
        # Define noise levels for sensor measurements
        self.sigma_y = np.concatenate([
            noise[0] * np.ones(self.n_controlled_joints),
            noise[1] * np.ones(self.n_controlled_joints)
        ])
        
        # Initialize state
        self.x = self.x0.copy()
        
        # Define output and input names
        self.output_names = [f'position_{i+1}' for i in range(self.n_controlled_joints)] + \
                            [f'velocity_{i+1}' for i in range(self.n_controlled_joints)]
        self.input_names = [f'torque_{i+1}' for i in range(self.n_controlled_joints)]
        
        self.payload = 0  # Initialize payload
    
    def full_joint_position(self):
        """
        Return the full joint position vector.
        """
        return self.x[:self.njoints]
    
    def set_payload(self, payload):
        """
        Set the payload value.
        """
        self.payload = payload
    
    def state_function(self, x, u, t):
        """
        Compute the state derivative dx/dt given state x and input u.
        """
        ql = x[:self.njoints]  # Link position
        qm = x[self.njoints:2*self.njoints]  # Motor position
        qld = x[2*self.njoints:3*self.njoints]  # Link velocity
        qmd = x[3*self.njoints:4*self.njoints]  # Motor velocity
        
        # Compute elastic torque
        tau_elastic = self.K * (qm - ql) + self.D * (qmd - qld)
        
        # Apply saturation to control input
        u = np.clip(u, -self.umax, self.umax)
        
        # Convert state to numpy arrays for Pinocchio
        q_numpy = np.array(ql)
        qp_numpy = np.array(qld)
        qpp_numpy = np.zeros_like(ql)
        
        # Compute nonlinear torque using inverse dynamics
        nonlinear_tau = pin.rnea(self.model, self.data, q_numpy, qp_numpy, qpp_numpy)
        inertia = pin.crba(self.model, self.data, q_numpy)
        
        # Solve for link acceleration
        qldd = np.linalg.solve(inertia, tau_elastic - nonlinear_tau)
        
        # Solve for motor acceleration
        qmdd = np.linalg.solve(
            self.motor_inertia, u - tau_elastic - self.motor_viscous_term * qmd - self.motor_coulomb_term * np.tanh(30 * qmd)
        )
        
        return np.concatenate([qld, qmd, qldd, qmdd])
    
    def output_function(self):
        """
        Compute the output of the system given the current state.
        """
        return self.stateToOutput @ self.x + self.sigma_y * np.random.randn(len(self.sigma_y))
