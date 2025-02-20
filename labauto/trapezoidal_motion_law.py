import numpy as np
from labauto import MotionLaw

class TrapezoidalMotionLaw(MotionLaw):
    def __init__(self, max_Dy, max_DDy, Tc):
        """
        Class implementing the trapezoidal motion law, inheriting from MotionLaw.
        """
        super().__init__(max_Dy, max_DDy, Tc)
        
        self.t_acc = 0  # Acceleration phase duration
        self.t_cruise_vel = 0  # Constant velocity phase duration
        self.t_dec = 0  # Deceleration phase duration
        self.y_ini = None  # Initial position
        self.acc = None  # Acceleration value
        self.dec = None  # Deceleration value
        self.cruise_vel = None  # Constant velocity
        self.total_time = 0  # Total motion time
        self.tolerance = 1.0e-4
    
    def update_function(self):
        """
        Update the motion state based on the current time.
        """
        if self.time < self.t_acc:
            # Acceleration phase
            dt = self.time
            self.DDy = self.acc
            self.Dy = self.acc * dt
            self.y = self.y_ini + 0.5 * self.acc * dt**2
        elif self.time < self.t_acc + self.t_cruise_vel:
            # Constant velocity phase
            dt = self.time - self.t_acc
            self.DDy = np.zeros(self.ndof)
            self.Dy = self.cruise_vel
            self.y = (self.y_ini + 0.5 * self.acc * self.t_acc**2 + self.cruise_vel * dt)
        elif self.time < self.t_acc + self.t_cruise_vel + self.t_dec:
            # Deceleration phase
            dt = self.time - self.t_acc - self.t_cruise_vel
            self.DDy = self.dec
            self.Dy = self.cruise_vel + self.dec * dt
            self.y = (self.y_ini + 0.5 * self.acc * self.t_acc**2 + 
                      self.cruise_vel * self.t_cruise_vel + 
                      self.cruise_vel * dt + 0.5 * self.dec * dt**2)
        else:
            # Motion complete
            self.y = self.target_y
            self.Dy = np.zeros(self.ndof)
            self.DDy = np.zeros(self.ndof)
    
    def compute_motion_law_time(self):
        """
        Compute the timing parameters for trapezoidal motion.
        """
        self.time = 0
        distance = np.abs(self.target_y - self.y)
        
        if np.max(distance) < self.tolerance:
            self.t_acc = self.t_cruise_vel = self.t_dec = self.total_time = 0
            self.acc = self.dec = self.cruise_vel = np.zeros(self.ndof)
            self.y_ini = self.target_y
            return
        
        self.y_ini = self.y
        direction = np.sign(self.target_y - self.y)
        
        t_acc_joints = self.max_Dy / self.max_DDy
        t_dec_joints = t_acc_joints
        
        distance_during_acc = 0.5 * self.max_Dy * t_acc_joints
        distance_during_dec = 0.5 * self.max_Dy * t_dec_joints
        distance_during_accdec = distance_during_acc + distance_during_dec
        
        t_cruise_vel_joints = (distance - distance_during_accdec) / self.max_Dy
        
        for idx in range(self.ndof):
            if t_cruise_vel_joints[idx] < 0:
                t_acc_joints[idx] = np.sqrt(distance[idx] / self.max_DDy[idx])
                t_dec_joints[idx] = t_acc_joints[idx]
                t_cruise_vel_joints[idx] = 0
        
        t_tot = t_acc_joints + t_cruise_vel_joints + t_dec_joints
        idx_worst_case = np.argmax(t_tot)
        
        self.t_acc = t_acc_joints[idx_worst_case]
        self.t_cruise_vel = t_cruise_vel_joints[idx_worst_case]
        self.t_dec = t_dec_joints[idx_worst_case]
        
        self.acc = distance / (self.t_acc * (self.t_acc + self.t_cruise_vel))
        self.dec = -self.acc
        self.cruise_vel = self.acc * self.t_acc
        
        self.acc *= direction
        self.dec *= direction
        self.cruise_vel *= direction
        