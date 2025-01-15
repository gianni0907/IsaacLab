from __future__ import annotations

import gymnasium as gym
import torch

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.envs import DirectRLEnv
from omni.isaac.lab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

from .kyon_env_cfg import KyonEnvCfg

class KyonEnv(DirectRLEnv):
    cfg: KyonEnvCfg

    def __init__(self, cfg: KyonEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.action_scale = self.cfg.action_scale

    def _setup_scene(self):
        self.kyon = Articulation(self.cfg.robot)
        # add ground plane
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        # clone, filter, and replicate
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])
        # add articulation to scene
        self.scene.articulations["kyon"] = self.kyon
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._actions = actions.clone()
        self._processed_actions = self.cfg.action_scale * self._actions + self._robot.data.default_joint_pos

    def _apply_action(self):
        self.kyon.set_joint_position_target(self._processed_actions)

    def _get_observations(self) -> dict:
        obs = torch.cat(
            (
                self.kyon.data.root_com_pos_w[:, 2],
            ),
            dim=-1,
        )
        observations = {"policy": obs}
        return observations
    
    def _get_rewards(self) -> torch.Tensor:
        rewards = {
            "rew_alive": self.cfg.rew_scale_alive * (1.0 - self.reset_terminated.float()),
            "rew_terminated": self.cfg.rew_scale_terminated * self.reset_terminated.float(),
            "rew_base_height": self.cfg.rew_scale_base_height * self.kyon.data.root_com_pos_w[:, 2],
        }

        reward = torch.sum(torch.stack(list(rewards.values())), dim=0)
        return reward
    
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self.base_height = self.kyon.data.root_com_pos_w[:, 2]
        
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        out_of_bounds = self.base_height < self.cfg.min_base_height
        return out_of_bounds, time_out
    
    def _reset_idx(self, env_ids):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.kyon._ALL_INDICES
        self.kyon.reset(env_ids)
        super()._reset_idx(env_ids)
        # Reset robot state
        joint_pos = self.kyon.data.default_joint_pos[env_ids]
        joint_vel = self.kyon.data.default_joint_vel[env_ids]
        default_root_state = self.kyon.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]
        self.kyon.write_root_link_pose_to_sim(default_root_state[:, :7], env_ids)
        self.kyon.write_root_com_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.kyon.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)