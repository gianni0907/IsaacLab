import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import DCMotorCfg, ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

##
# Configuration - Articulation.
##

KYON_BASE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/ggravina-iit.local/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/HHCM/kyon_base.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.02, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            "hip_roll_.*": 0.0,
            "hip_pitch_1": 0.7,
            "hip_pitch_2": -0.7,
            "hip_pitch_3": 0.7,
            "hip_pitch_4": -0.7,
            "knee_pitch_1": -1.4,
            "knee_pitch_2": 1.4,
            "knee_pitch_3": -1.4,
            "knee_pitch_4": 1.4,
        },
    ),
    actuators={
        "legs": DCMotorCfg(
            joint_names_expr=["hip_roll_.*", "hip_pitch_.*", "knee_pitch_.*"],
            saturation_effort=120.0,
            effort_limit=80.0,
            velocity_limit=7.5,
            stiffness=500.0,
            damping=10.0,
            friction=0.0,
        ),
    }
)

KYON_BASE_WHEELED_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/ggravina-iit.local/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/HHCM/wheeled_kyon_base.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.02, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            "hip_roll_.*": 0.0,
            "wheel_.*": 0.0,
            "hip_pitch_1": 0.7,
            "hip_pitch_2": -0.7,
            "hip_pitch_3": 0.7,
            "hip_pitch_4": -0.7,
            "knee_pitch_1": -1.4,
            "knee_pitch_2": 1.4,
            "knee_pitch_3": -1.4,
            "knee_pitch_4": 1.4,
        },
    ),
    actuators={
        "wheels": DCMotorCfg(
            joint_names_expr=["wheel_.*"],
            saturation_effort=120.0,
            effort_limit=80.0,
            velocity_limit=7.5,
            stiffness=0.0,
            damping=30.0,
            friction=0.0,
        ),
        "legs": DCMotorCfg(
            joint_names_expr=["hip_roll_.*", "hip_pitch_.*", "knee_pitch_.*"],
            saturation_effort=120.0,
            effort_limit=80.0,
            velocity_limit=7.5,
            stiffness=500.0,
            damping=10.0,
            friction=0.0,
        ),
    }
)