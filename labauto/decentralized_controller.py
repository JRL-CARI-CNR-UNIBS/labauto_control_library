from labauto import BaseController
import numpy as np


class DecentralizedController(BaseController):
    """
    DecentralizedController - Controller for a multi-joint rigid body robot.

    This class implements a decentralized control structure where each joint has its
    own inner controller. It utilizes an inverse dynamics function for torque computation.
    """

    def __init__(self, Tc: float, joint_ctrls: list, inverse_dynamics_fcn=None):
        """
        Constructor to create a DecentralizedController object.

        :param Tc: Sampling time (must be a positive scalar).
        :param joint_ctrls: List of inner controllers for each joint (instances of BaseController).
        :param inverse_dynamics_fcn: Function for inverse dynamics computation (optional).
        """
        super().__init__(Tc)

        if not isinstance(joint_ctrls, list) or not all(isinstance(ctrl, BaseController) for ctrl in joint_ctrls):
            raise ValueError("joint_ctrls must be a list of BaseController instances")

        self.joint_ctrls = joint_ctrls
        self.njoints = len(joint_ctrls)
        self.inverse_dynamics_fcn = inverse_dynamics_fcn if inverse_dynamics_fcn else lambda q, dq, ddq: np.zeros(
            self.njoints)
        self.umax = np.array([ctrl.get_umax() for ctrl in joint_ctrls])

    def initialize(self):
        """Initialize all inner controllers."""
        for ctrl in self.joint_ctrls:
            ctrl.initialize()

    def set_umax(self, umax: list):
        """Set the maximum control action for each joint."""
        if len(umax) != self.njoints:
            raise ValueError("umax must have the same length as the number of joints")
        for idx, ctrl in enumerate(self.joint_ctrls):
            ctrl.set_umax(umax[idx])

    def starting(self, reference: np.ndarray, y: np.ndarray, u: np.ndarray, uff: np.ndarray):
        """Set the starting conditions for the controllers."""
        if len(reference) != 3 * self.njoints or len(y) != 2 * self.njoints or len(u) != self.njoints or len(
                uff) != self.njoints:
            raise ValueError("Incorrect input dimensions")

        q, dq = y[:self.njoints], y[self.njoints:]
        qref, dqref, ddqref = reference[:self.njoints], reference[self.njoints:2 * self.njoints], reference[
                                                                                                  2 * self.njoints:]
        precomputed_torque = self.inverse_dynamics_fcn(qref, dqref, ddqref)
        uff += precomputed_torque

        for idx in range(self.njoints):
            self.joint_ctrls[idx].starting([qref[idx], dqref[idx]], [q[idx], dq[idx]], u[idx], uff[idx])

    def compute_control_action(self, reference: np.ndarray, y: np.ndarray, uff: np.ndarray) -> np.ndarray:
        """Compute the control action based on the reference, state, and feedforward."""
        if len(reference) != 3 * self.njoints or len(y) != 2 * self.njoints or len(uff) != self.njoints:
            raise ValueError("Incorrect input dimensions")

        q, dq = y[:self.njoints], y[self.njoints:]
        qref, dqref, ddqref = reference[:self.njoints], reference[self.njoints:2 * self.njoints], reference[
                                                                                                  2 * self.njoints:]
        precomputed_torque = self.inverse_dynamics_fcn(qref, dqref, ddqref)
        uff += precomputed_torque

        u = np.zeros(self.njoints)
        for idx in range(self.njoints):
            u[idx] = self.joint_ctrls[idx].compute_control_action([qref[idx], dqref[idx]], [q[idx], dq[idx]], uff[idx])

        return u
