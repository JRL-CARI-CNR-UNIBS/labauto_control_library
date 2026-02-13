import numpy as np
import mujoco
import os
import sys
import time

# Optional interactive visualization (requires a GUI)
try:
    import mujoco.viewer  # type: ignore
except Exception:
    mujoco.viewer = None  # type: ignore[attr-defined]


def assert_viewer_supported() -> None:
    """Raise a helpful error if mujoco.viewer is unavailable in this environment."""
    if sys.platform == "darwin":
        # On macOS, the MuJoCo viewer requires the mjpython executable.
        exe = os.path.basename(sys.executable).lower()
        if "mjpython" not in exe:
            raise RuntimeError("On macOS, run with: mjpython your_script.py (required for mujoco.viewer)")


# Robust import (works whether you're running inside the labauto package or standalone)
try:
    from labauto.mechanical_system import MechanicalSystem
except Exception:
    from mechanical_system import MechanicalSystem


class MuJoCoMechanicalSystem(MechanicalSystem):
    """MechanicalSystem-compatible MuJoCo wrapper.

    - Actuation is applied to motor-side joints via MuJoCo motor actuators (motor_1..3).
    - The measured joint position/velocity returned to the controller are motor-side:
      q_out = q_motor, dq_out = dq_motor
    """

    def __init__(
        self,
        xml_path: str,
        motor_actuators=("motor_1", "motor_2", "motor_3"),
        motor_joints=("joint_1", "joint_2", "joint_3"),
        spring_joints=("spring_1", "spring_2", "spring_3"),
        ee_site: str = "ee",
        sample_period: float = 0.001,
        simulating_steps: int = 5,
        enable_viewer: bool = True,
        viewer_decimation: int = 30,
    ):
        # Initialize base class with sampling period
        super().__init__(st=float(sample_period))

        self.xml_path = xml_path
        self.motor_actuators = list(motor_actuators)
        self.motor_joints = list(motor_joints)
        self.spring_joints = list(spring_joints)
        self.ee_site_name = ee_site

        self.model = None
        self.data = None
        self._act_ids = None
        self._motor_qadr = None
        self._motor_vadr = None
        self._spring_qadr = None
        self._spring_vadr = None
        self._ee_site_id = None

        # Sampling/simulation settings
        self.sample_period = float(sample_period)  # keep for backwards compatibility
        self.simulating_steps = int(simulating_steps)
        self.simulating_period = self.sample_period / self.simulating_steps

        # Viewer (optional)
        self.enable_viewer = bool(enable_viewer)
        self.viewer_decimation = int(viewer_decimation)
        self.viewer_count = 0
        self.viewer = None

        # MechanicalSystem fields (override the defaults from MechanicalSystem.__init__)
        self.num_input = len(self.motor_actuators)
        dof = len(self.motor_joints)
        self.num_output = 2 * dof  # [q1..qn, dq1..dqn]
        self.order = self.num_output

        self.u0 = np.zeros(self.num_input)
        self.u = self.u0.copy()

        self.x0 = np.zeros(self.order)
        self.x = self.x0.copy()

        # Names consistent with the gantry API
        self.output_names = [f"q_{i+1}" for i in range(dof)] + [f"dq_{i+1}" for i in range(dof)]
        self.input_names = [f"force_{i+1}" for i in range(self.num_input)]

        # umax will be updated after model load; keep a sane placeholder for pre-init calls
        self.umax = np.full(self.num_input, 5.0)
        self.sigma_y = np.concatenate([np.full(dof, 1e-4), np.full(dof, 3e-2)]).flatten()

    def initialize(self):
        # Reset base bookkeeping (x,u,t)
        super().initialize()

        # Load MuJoCo model/data
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.model.opt.timestep = self.simulating_period

        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        # Optional interactive visualization
        # Close any existing viewer (in case initialize is called more than once)
        if getattr(self, "viewer", None) is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
        self.viewer = None
        self.viewer_count = 0
        if self.enable_viewer and getattr(mujoco, "viewer", None) is not None:
            assert_viewer_supported()
            self.viewer = mujoco.viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=False,
                show_right_ui=False,
            )
            print("Viewer")
            # Set a sensible default camera distance
            try:
                with self.viewer.lock():
                    self.viewer.cam.distance = 10.0
                    self.viewer.cam.azimuth = 100.0     # deg
                    self.viewer.cam.elevation = -25.0   # deg
            except Exception:
                # If the viewer doesn't expose cam/lock on this platform/version, ignore.
                pass


        # Resolve actuator indices
        self._act_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in self.motor_actuators],
            dtype=int,
        )

        # Resolve joint qpos/dof addresses
        def _qv_addrs(joint_name: str):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            return int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid])

        motor_addrs = np.array([_qv_addrs(j) for j in self.motor_joints], dtype=int)
        spring_addrs = np.array([_qv_addrs(j) for j in self.spring_joints], dtype=int)
        self._motor_qadr, self._motor_vadr = motor_addrs[:, 0], motor_addrs[:, 1]
        self._spring_qadr, self._spring_vadr = spring_addrs[:, 0], spring_addrs[:, 1]

        # End-effector site (optional)
        try:
            self._ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.ee_site_name)
        except Exception:
            self._ee_site_id = None

        # Update umax from actuator force range so MechanicalSystem.saturation_control_action can be used if desired
        self.umax = self.get_umax()

        # Ensure starting ctrl is zeroed
        self.data.ctrl[:] = 0.0

        # Initialize x from current sensor reading
        self.x = self.read_sensor_value().copy()

    def get_sampling_period(self) -> float:
        return float(self.st)  # st is the authoritative sampling period in MechanicalSystem

    def get_umax(self) -> np.ndarray:
        if self.model is None or self._act_ids is None:
            return np.asarray(self.umax, dtype=float).copy()
        fr = np.asarray(self.model.actuator_forcerange)[self._act_ids]
        return np.max(np.abs(fr), axis=1)

    def read_sensor_value(self) -> np.ndarray:
        q_motor = self.data.qpos[self._motor_qadr]
        dq_motor = self.data.qvel[self._motor_vadr]

        q_out = q_motor  # motor side
        dq_out = dq_motor  # motor side

        y = np.concatenate([q_out, dq_out]).astype(float, copy=False)

        # Optional measurement noise hook (MechanicalSystem convention)
        if np.any(np.asarray(self.sigma_y) != 0):
            y = y + self.sigma_y * np.random.randn(y.size)

        # Keep MechanicalSystem state vector consistent with latest measurement
        self.x = y.copy()
        return y

    def read_actuator_value(self) -> np.ndarray:
        # Returns current actuator force (N) for motor actuators (preserves your previous behavior)
        return np.asarray(self.data.actuator_force)[self._act_ids].copy()

    def write_actuator_value(self, desired_force: np.ndarray):
        desired_force = np.asarray(desired_force, dtype=float).reshape(-1)
        if desired_force.size != len(self._act_ids):
            raise ValueError(f"Expected {len(self._act_ids)} actuator commands, got {desired_force.size}")

        # Store the commanded input in MechanicalSystem.u
        super().write_actuator_value(desired_force)

        # Optionally saturate by umax (keeps consistency with MechanicalSystem conventions)
        desired_force = self.saturation_control_action(desired_force)

        # Convert desired force (N) into motor 'ctrl' given actuator gear and ctrlrange.
        # For <motor>, MuJoCo uses: actuator_force ≈ gear * ctrl (then clamped if forcelimited).
        for i, act_id in enumerate(self._act_ids):
            gear = float(self.model.actuator_gear[act_id][0])
            lo, hi = self.model.actuator_ctrlrange[act_id]
            ctrl = desired_force[i] / gear if gear != 0 else 0.0
            self.data.ctrl[act_id] = float(np.clip(ctrl, lo, hi))

    def link_position(self) -> np.ndarray:
        if self._ee_site_id is None:
            return np.zeros(3)
        return np.asarray(self.data.site_xpos[self._ee_site_id]).copy()

    def simulate(self):
        # Step the physics at the internal timestep
        for _ in range(self.simulating_steps):
            mujoco.mj_step(self.model, self.data)

        # Optional viewer update (decimated)
        if getattr(self, "viewer", None) is not None:
            self.viewer_count += 1
            if self.viewer_count >= self.viewer_decimation:
                self.viewer_count = 0
                try:
                    self.viewer.sync()
                except Exception:
                    pass

        # Advance MechanicalSystem time and refresh x
        self.t += self.st
        self.x = self.read_sensor_value().copy()

    def close(self):
        """Close resources (viewer). Safe to call multiple times."""
        if getattr(self, "viewer", None) is not None:
            try:
                self.viewer.close()
                # Wait briefly for the viewer thread to stop
                while getattr(self.viewer, "is_running", lambda: False)():
                    time.sleep(0.01)
            except Exception:
                pass
            finally:
                self.viewer = None

    def __del__(self):
        # Best-effort cleanup (avoid raising in destructor)
        try:
            self.close()
        except Exception:
            pass

 
