from labauto import BaseController


class CascadeController(BaseController):
    """
    CascadeController - Implements a cascade control structure with inner and outer loops.

    The outer loop controls position, while the inner loop controls velocity.
    This controller receives a reference signal, current state, and feedforward input,
    and computes the control action by combining the outputs of both loops.
    """

    def __init__(self, Tc: float, inner_ctrl: BaseController, outer_ctrl: BaseController):
        """
        Constructor to create a CascadeController object.

        :param Tc: Sampling time (must be a positive scalar).
        :param inner_ctrl: Inner loop controller (instance of BaseController).
        :param outer_ctrl: Outer loop controller (instance of BaseController).
        """
        super().__init__(Tc)

        if not isinstance(inner_ctrl, BaseController):
            raise ValueError("InnerCtrl must be an instance of BaseController")
        if not isinstance(outer_ctrl, BaseController):
            raise ValueError("OuterCtrl must be an instance of BaseController")

        self.inner_ctrl = inner_ctrl
        self.outer_ctrl = outer_ctrl
        self.set_umax(inner_ctrl.get_umax())

    def initialize(self):
        """Initialize both inner and outer controllers."""
        self.outer_ctrl.initialize()
        self.inner_ctrl.initialize()

    def set_umax(self, umax: float):
        """Set the maximum control action."""
        self._umax = umax
        self.inner_ctrl.set_umax(umax)

    def starting(self, reference: list, y: list, u: float, uff: float):
        """Set the starting conditions for the inner and outer loops."""
        if len(reference) != 2 or len(y) != 2:
            raise ValueError("Reference and state (y) must be 2-element lists")
        if not isinstance(u, (int, float)) or not isinstance(uff, (int, float)):
            print(f"u={u}, uff={uff}")
            raise ValueError("Control action (u) and feedforward (uff) must be scalars")

        self.outer_ctrl.starting(reference[0], y[0], y[1], reference[1])
        self.inner_ctrl.starting(y[1], y[1], u, uff)

    def compute_control_action(self, reference: list, y: list, uff: float) -> float:
        """Compute the control action based on reference, state, and feedforward."""
        if len(reference) != 2 or len(y) != 2:
            raise ValueError("Reference and state (y) must be 2-element lists")
        if not isinstance(uff, (int, float)):
            raise ValueError("Feedforward input (uff) must be a scalar")

        vel_ref = self.outer_ctrl.compute_control_action(reference[0], y[0], reference[1])
        return self.inner_ctrl.compute_control_action(vel_ref, y[1], uff)
