# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass


from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm

from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, ObservationsCfg
##
# Pre-defined configs
##
from isaaclab_assets.robots.anymal import ANYMAL_C_CFG
from source.isaaclab.isaaclab.managers.scene_entity_cfg import SceneEntityCfg
from source.isaaclab_tasks.isaaclab_tasks.manager_based.navigation import mdp  # isort: skip


@configclass
class CustomObservationsCfg(ObservationsCfg):
    """Observations specifications for the MDP."""

    @configclass
    class CriticCfg(ObservationsCfg.PolicyCfg):
        """Observations for critic group."""

        # observation terms (order preserved)
        gt_base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        gt_base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        gt_projected_gravity = ObsTerm(func=mdp.projected_gravity)
        gt_joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        gt_joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        gt_height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    critic: CriticCfg = CriticCfg()

@configclass
class CustomAnymalCRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    observations: CustomObservationsCfg = CustomObservationsCfg()
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # switch robot to anymal-c
        self.scene.robot = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


@configclass
class CustomAnymalCRoughEnvCfg_PLAY(CustomAnymalCRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
