from abc import ABC, abstractmethod
import numpy as np
import ast 

class MotionLaw(ABC):
    def __init__(self, max_Dy, max_DDy, Tc):
        """
        Abstract class for motion laws in control systems.
        """
        assert isinstance(max_Dy, np.ndarray) and max_Dy.ndim == 1, "max_Dy must be a 1D numpy array"
        assert isinstance(max_DDy, np.ndarray) and max_DDy.ndim == 1, "max_DDy must be a 1D numpy array"
        assert isinstance(Tc, (int, float)), "Tc must be a scalar"
        
        self.Tc = Tc
        self.ndof = len(max_Dy)
        assert len(max_DDy) == self.ndof, "max_DDy must have the same length as max_Dy"

        self.s = 0.0  #Curvilinear abscissa
        self.time = 0.0 # time
        
        self.max_Dy = max_Dy
        self.max_DDy = max_DDy
        
        # Initialize state variables
        self.y0 = np.zeros(self.ndof)
        self.initialize()
    
    def set_initial_condition(self, y0):
        """ Set the initial condition """
        self.y0 = y0
        self.target_y = y0
        self.initialize()
    
    def initialize(self):
        """ Initialize state variables """
        self.y = self.y0.copy()
        self.Dy = np.zeros(self.ndof)
        self.DDy = np.zeros(self.ndof)
        
        self.target_y = self.y0.copy()
        self.target_Dy = np.zeros(self.ndof)
        self.target_DDy = np.zeros(self.ndof)
        
        self.instruction_list = []
        self._depending_instructions = True
        self.t_rest = 0
    
    def add_instructions(self, instructions):
        """ Add instructions to the instruction list """
        if not self.instruction_list:
            self.instruction_list = instructions
        else:
            self.instruction_list.extend(instructions)
    
    def depending_instructions(self):
        """ Check if there are depending instructions """
        return self._depending_instructions
    
    def compute_motion_law(self, measure=None):
        """ Compute the motion law based on the current position """
        y = self.y if measure is None else measure
        
        if np.max(np.abs(self.target_y - y)) < 1e-6:  # Tolerance check
            self.decode_instruction()
        
        self.update_function()
        self.time += self.Tc


        return self.y, self.Dy, self.DDy
    
    def decode_instruction(self):
        """ Decode the next instruction from the instruction list """
        if self.t_rest > 0:
            self.t_rest -= self.Tc
            return
        
        if not self.instruction_list:
            self._depending_instructions = False
            return
        
        self._depending_instructions = True
        cmd = self.instruction_list.pop(0)
        
        if cmd.startswith("move: "):

            target = np.array(ast.literal_eval(cmd.replace("move: ", "")))

            if target.ndim == 1 and len(target) == self.ndof:
                self.target_y = target
                self.target_Dy = np.zeros(self.ndof)
                self.target_DDy = np.zeros(self.ndof)
                self.compute_motion_law_time()
            else:
                print(f"Warning: Invalid command {cmd}, skipping.")
                self.decode_instruction()
        elif cmd.startswith("pause: "):
            self.t_rest = float(cmd.replace("pause: ", ""))
        else:
            print(f"Warning: Unrecognized command {cmd}, skipping.")
            self.decode_instruction()
    
    @abstractmethod
    def update_function(self):
        """ Abstract method to update the motion law """
        pass
    
    @abstractmethod
    def compute_motion_law_time(self):
        """ Abstract method to compute motion law timing intervals """
        pass
