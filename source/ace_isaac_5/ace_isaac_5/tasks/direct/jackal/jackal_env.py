# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import torch
from collections.abc import Sequence

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg, AssetBaseCfg, AssetBase
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane, spawn_from_usd
from isaaclab.sensors import TiledCamera

from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaaclab.utils.math as math_utils

def define_markers() -> VisualizationMarkers:
    """Define markers with various different shapes."""
    marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/myMarkers",
        markers={
                "forward": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.25, 0.25, 0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 1.0)),
                ),
                "command": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.25, 0.25, 0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                ),
        },
    )
    return VisualizationMarkers(cfg=marker_cfg)




from .jackal_env_cfg import JackalEnvCfg


class JackalEnv(DirectRLEnv):
    cfg: JackalEnvCfg

    def __init__(self, cfg: JackalEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.dof_idx, _ = self.robot.find_joints(self.cfg.dof_names)
        print(self.dof_idx)

    def _setup_scene(self):

        #self.scene.sensors.clear()
        self.robot = Articulation(self.cfg.robot_cfg)
        # add ground plane
        #spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        # clone and replicate
        self.scene.clone_environments(copy_from_source=False)
        # add articulation to scene
        self.scene.articulations["robot"] = self.robot
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone()

    def _apply_action(self) -> None:
        self.robot.set_joint_velocity_target(self.actions, joint_ids=self.dof_idx)

    def _get_observations(self) -> dict:

        self.velocity = self.robot.data.root_com_lin_vel_b 
        observations = {"policy": self.velocity}
        return observations

    def _get_rewards(self) -> torch.Tensor:

        total_reward = torch.linalg.norm(self.velocity, dim=-1, keepdim=True)
        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1

        return False, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):

        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.robot.write_root_state_to_sim(default_root_state, env_ids)