import numpy as np
from labauto import DecentralizedController
from labauto import CascadeController
from labauto import PIDController
from labauto import FirstOrderLowPassFilter
from labauto import NotchFilter
from labauto import FIRFilter
import pinocchio as pin
from functools import partial


def loadFilters(Tc, params):
    filters = []
    for p in params:
        if p["type"] == "FirstOrderLowPassFilter":
            if not ("time_constant" in p.keys()):
                raise ValueError("FirstOrderLowPassFilter has no key time_constant")
            print(f'- FirstOrderLowPassFilter with time_constant {p["time_constant"]}')
            filters.append(FirstOrderLowPassFilter(Tc, p["time_constant"]))
        elif p["type"] == "NotchFilter":
            if not ("natural_frequency" in p.keys()):
                raise ValueError("NotchFilter has no key natural_frequency")
            if not ("poles_damping" in p.keys()):
                raise ValueError("NotchFilter has no key poles_damping")
            if not ("zeros_damping" in p.keys()):
                raise ValueError("NotchFilter has no key zeros_damping")
            print(f'- NotchFilter with natural_frequency {p["natural_frequency"]} zeros_damping {p["zeros_damping"]} poles_damping {p["poles_damping"]}')
            filters.append(NotchFilter(Tc, wn=p["natural_frequency"], xi_z=p["zeros_damping"], xi_p=p["poles_damping"]))

        elif p["type"] == "FIRFilter":
            if not ("coefficients" in p.keys()):
                raise ValueError("FIRFilter has no key coefficients")
            print(f'- FIRFilter with coefficients {p["coefficients"]}')
            filters.append(FIRFilter(Tc,fir_coef=p["coefficients"]))
        else:
            print(f'Type {p["type"]} is not supported')

    return filters

def loadPid(Tc, params):
    print(f"PID Kp={params['Kp']}, Ki={params['Ki']}, Kd={params['Kd']}")
    if "filters_on_derivative_error" in params.keys():
        print("filters_on_derivative_error:")
        filters_on_derivative_error=loadFilters(Tc,params["filters_on_derivative_error"])
    else:
        filters_on_derivative_error=[]

    if "filters_on_measure" in params.keys():
        print("filters_on_error_signal:")
        filters_on_measure = loadFilters(Tc, params["filters_on_measure"])
    else:
        filters_on_measure = []

    if "filters_on_error_signal" in params.keys():
        print("filters_on_error_signal:")
        filters_on_error_signal = loadFilters(Tc, params["filters_on_error_signal"])
    else:
        filters_on_error_signal = []

    return PIDController(Tc=Tc, Kp=params['Kp'], Ki=params['Ki'], Kd=params['Kd'],filters_on_derivative_error=filters_on_derivative_error, filters_on_measure=filters_on_measure,filters_on_error_signal=filters_on_error_signal)


def loadCascade(Tc, params):
    print(f"\n\nCreating controller {params['name']}")
    print("\n\nInner controller")
    inner_ctrl = loadPid(Tc, params['inner'])
    print("\n\nOuter controller")
    outer_ctrl = loadPid(Tc, params['outer'])
    return CascadeController(Tc=Tc, inner_ctrl=inner_ctrl, outer_ctrl=outer_ctrl)


def loadController(Tc, controller_params, dynamic_params, model_name):
    # Load the tuned controller using parameters from YAML
    cascade_controllers = []
    for cascade_ctrl_params in controller_params['cascade_controllers']:
        cascade_controllers.append(loadCascade(Tc, cascade_ctrl_params))

    # Define inverse dynamics function
    model = pin.buildModelFromUrdf(f'{model_name}/model.urdf')
    model_data = model.createData()

    def pinocchio_inverse_dynamics(model, model_data, params, q, dq, ddq):
        Phi_dynamic = pin.computeJointTorqueRegressor(model, model_data, q, dq, ddq)
        dof = model.nq

        diag_dq = np.diag(dq)
        diag_sign_dq = np.diag(np.tanh(100*dq))
        Phi_friction = np.hstack((diag_dq, diag_sign_dq))
        Phi = np.hstack((Phi_dynamic, Phi_friction))
        return Phi @ params

    print(f"dynamic_params={dynamic_params}")
    inverse_dynamics = partial(pinocchio_inverse_dynamics, model, model_data, dynamic_params)

    decentralized_ctrl = DecentralizedController(Tc=Tc, joint_ctrls=cascade_controllers,
                                                 inverse_dynamics_fcn=inverse_dynamics)

    return decentralized_ctrl
