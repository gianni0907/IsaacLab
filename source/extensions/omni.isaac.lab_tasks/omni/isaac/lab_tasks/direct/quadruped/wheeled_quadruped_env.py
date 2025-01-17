import os
import math
import torch
import gymnasium as gym
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import Articulation, ArticulationCfg
from omni.isaac.lab.envs import DirectRLEnv, DirectRLEnvCfg
from omni.isaac.lab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.utils import configclass

QUADRUPED_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.environ['HOME'] + "/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/classic/quadruped/quadruped_robot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.828), joint_pos={"robot1_rl_wheel_joint": 0.0, "robot1_rr_wheel_joint": 0.0,
                                          "robot1_front_left_thigh_joint": 0.0, "robot1_front_right_thigh_joint": 0.0}
    ),
    actuators={
        "rl_wheel_actuator": ImplicitActuatorCfg(
            joint_names_expr=["robot1_rl_wheel_joint"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "rr_wheel_actuator": ImplicitActuatorCfg(
            joint_names_expr=["robot1_rr_wheel_joint"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "fl_thigh_actuator": ImplicitActuatorCfg(
            joint_names_expr=["robot1_front_left_thigh_joint"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=1000.0,
            damping=0.005,
        ),
        "fr_thigh_actuator": ImplicitActuatorCfg(
            joint_names_expr=["robot1_front_right_thigh_joint"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=1000.0,
            damping=0.005,
        ),
    },
)

@configclass
class WheeledQuadrupedEnvCfg(DirectRLEnvCfg):
    """Configuration for the wheeled quadruped environment."""
    # env
    decimation = 2
    episode_length_s = 20.0
    action_scale = 1.0 # [N]
    action_space = 4
    observation_space = 10
    state_space = 0

    # simulation
    sim = sim_utils.SimulationCfg(dt=0.005, render_interval=decimation)

    # robot
    robot_cfg = QUADRUPED_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    rl_wheel_dof_name = "robot1_rl_wheel_joint"
    rr_wheel_dof_name = "robot1_rr_wheel_joint"
    fl_thigh_dof_name = "robot1_front_left_thigh_joint"
    fr_thigh_dof_name = "robot1_front_right_thigh_joint"

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)

    # reset
    minimum_com_height = 0.4
    limit_angle = math.pi/3

    # reward scales
    rew_scale_alive = 1.0
    rew_scale_terminated = -2.0
    rew_scale_com_height = 0.0

class WheeledQuadrupedEnv(DirectRLEnv):
    cfg: WheeledQuadrupedEnvCfg

    def __init__(self, cfg: WheeledQuadrupedEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        # Joint position command (deviation from default joint positions)
        
        self._actions = torch.zeros(self.num_envs, gym.spaces.flatdim(self.single_action_space), device=self.device)
        self._previous_actions = torch.zeros(
            self.num_envs, gym.spaces.flatdim(self.single_action_space), device=self.device
        )

        self._rl_wheel_dof_idx, _ = self.quadruped.find_joints(self.cfg.rl_wheel_dof_name)
        self._rr_wheel_dof_idx, _ = self.quadruped.find_joints(self.cfg.rr_wheel_dof_name)
        self._fl_thigh_dof_idx, _ = self.quadruped.find_joints(self.cfg.fl_thigh_dof_name)
        self._fr_thigh_dof_idx, _ = self.quadruped.find_joints(self.cfg.fr_thigh_dof_name)
        self.action_scale = self.cfg.action_scale

        self.base_height = self.quadruped.data.root_com_pos_w[:, 2]
        self.base_lin_vel = self.quadruped.data.root_com_lin_vel_b
        self.base_ang_vel = self.quadruped.data.root_com_ang_vel_b

    def _setup_scene(self):
        self.quadruped = Articulation(self.cfg.robot_cfg)
        # add ground plane
        spawn_ground_plane(prim_path="/World/ground",cfg=GroundPlaneCfg())
        # clone, filter, and replicate
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])
        # add articulation to scene
        self.scene.articulations["robot"] = self.quadruped
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)


    def _pre_physics_step(self, actions: torch.Tensor):
        self._actions = actions.clone()
        self._processed_actions = self.cfg.action_scale * self._actions

    def _apply_action(self):
        self.quadruped.set_joint_velocity_target(self._processed_actions[:, :2].unsqueeze(dim=2), joint_ids=[self._rl_wheel_dof_idx, self._rr_wheel_dof_idx])
        self.quadruped.set_joint_position_target(self._processed_actions[:, 2:].unsqueeze(dim=2), joint_ids=[self._fl_thigh_dof_idx, self._fr_thigh_dof_idx])

    def _get_observations(self) -> dict:
        self._previous_actions = self._actions.clone()
        obs = torch.cat(
            (
                self.base_lin_vel,
                self.base_ang_vel,
                self._actions,
            ),
            dim=-1,
        )
        observations = {"policy": obs}
        return observations
    
    def _get_rewards(self) -> torch.Tensor:
        total_reward = compute_rewards(
            self.cfg.rew_scale_alive,
            self.cfg.rew_scale_terminated,
            self.cfg.rew_scale_com_height,
            self.base_height,
            self.reset_terminated
        )
        return total_reward
    
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        out_of_bounds = self.base_height < self.cfg.minimum_com_height
        out_of_bounds = out_of_bounds | (torch.acos(-self.quadruped.data.projected_gravity_b[:, 2]).abs() > self.cfg.limit_angle)
        return out_of_bounds, time_out
    
    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = self.quadruped._ALL_INDICES
        super()._reset_idx(env_ids)

        self._actions[env_ids] = 0.0
        self._previous_actions[env_ids] = 0.0
        joint_pos = self.quadruped.data.default_joint_pos[env_ids]
        joint_vel = self.quadruped.data.default_joint_vel[env_ids]
        default_root_state = self.quadruped.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.base_height[env_ids] = default_root_state[:, 2]
        self.base_lin_vel[env_ids] = default_root_state[:, 7:10]
        self.base_ang_vel[env_ids] = default_root_state[:, 10:13]  

        self.quadruped.write_root_link_pose_to_sim(default_root_state[:, :7], env_ids)
        self.quadruped.write_root_com_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.quadruped.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

@torch.jit.script
def compute_rewards(
    rew_scale_alive: float,
    rew_scale_terminated: float,
    rew_scale_com_height: float,
    com_height: torch.Tensor,
    reset_terminated: torch.Tensor,
) -> torch.Tensor:
    alive_reward = rew_scale_alive * (1.0 - reset_terminated.float())
    terminated_reward = rew_scale_terminated * reset_terminated.float()
    com_height_reward = rew_scale_com_height * torch.square(com_height - 0.828)
    total_reward = alive_reward + terminated_reward + com_height_reward
    return total_reward