import numpy as np
from labauto import DecentralizedController
from labauto import CascadeController
from labauto import PIDController
import pinocchio as pin
from functools import partial


def loadController(Tc,controller_params,dynamic_params,model_name):
    # Load the tuned controller using parameters from YAML
    Kp_inner1, Ki_inner1, Kd_inner1 = controller_params['inner1']['Kp'], controller_params['inner1']['Ki'], \
    controller_params['inner1']['Kd']
    Kp_outer1, Ki_outer1, Kd_outer1 = controller_params['outer1']['Kp'], controller_params['outer1']['Ki'], \
    controller_params['outer1']['Kd']
    Kp_inner2, Ki_inner2, Kd_inner2 = controller_params['inner2']['Kp'], controller_params['inner2']['Ki'], \
    controller_params['inner2']['Kd']
    Kp_outer2, Ki_outer2, Kd_outer2 = controller_params['outer2']['Kp'], controller_params['outer2']['Ki'], \
    controller_params['outer2']['Kd']

    inner_ctrl1 = PIDController(Tc=Tc, Kp=Kp_inner1, Ki=Ki_inner1, Kd=Kd_inner1)
    outer_ctrl1 = PIDController(Tc=Tc, Kp=Kp_outer1, Ki=Ki_outer1, Kd=Kd_outer1)
    inner_ctrl2 = PIDController(Tc=Tc, Kp=Kp_inner2, Ki=Ki_inner2, Kd=Kd_inner2)
    outer_ctrl2 = PIDController(Tc=Tc, Kp=Kp_outer2, Ki=Ki_outer2, Kd=Kd_outer2)

    joint1_ctrl = CascadeController(Tc=Tc, inner_ctrl=inner_ctrl1, outer_ctrl=outer_ctrl1)
    joint2_ctrl = CascadeController(Tc=Tc, inner_ctrl=inner_ctrl2, outer_ctrl=outer_ctrl2)

    # Define inverse dynamics function
    model = pin.buildModelFromUrdf(f'{model_name}/model.urdf')
    model_data = model.createData()

    def pinocchio_inverse_dynamics(model, model_data, params, q, dq, ddq):
        Phi_dynamic = pin.computeJointTorqueRegressor(model, model_data, q, dq, ddq)
        dof = model.nq

        diag_dq = np.diag(dq)  # diag(dq)
        diag_sign_dq = np.diag(np.sign(dq))  # diag(sign(dq))
        Phi_friction = np.hstack((diag_dq, diag_sign_dq))
        Phi = np.hstack((Phi_dynamic, Phi_friction))
        return Phi @ params

    inverse_dynamics = partial(pinocchio_inverse_dynamics, model, model_data, dynamic_params)

    decentralized_ctrl = DecentralizedController(Tc=Tc, joint_ctrls=[joint1_ctrl, joint2_ctrl],
                                                 inverse_dynamics_fcn=inverse_dynamics)

    return decentralized_ctrl