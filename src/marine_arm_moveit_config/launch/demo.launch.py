from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("marine_arm", package_name="marine_arm_moveit_config").to_moveit_configs()
    moveit_config.move_group_capabilities = {"capabilities": "", "disable_capabilities": ""}
    return generate_demo_launch(moveit_config)