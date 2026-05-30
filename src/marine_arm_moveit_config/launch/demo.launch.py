from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("marine_arm", package_name="marine_arm_moveit_config").to_moveit_configs()
    moveit_config.move_group_capabilities = {"capabilities": "", "disable_capabilities": ""}
    return generate_demo_launch(moveit_config)
# import launch.utilities.ensure_argument_type_impl

# # 1. 保存 ROS 2 原生的类型检查函数
# original_ensure = launch.utilities.ensure_argument_type_impl.ensure_argument_type

# # 2. 编写万能拦截器：用 *args 和 **kwargs 接收任意数量和名称的参数
# def patched_ensure(*args, **kwargs):
#     new_args = list(args)
    
#     # 检查按位置传递的第一个参数（即需要验证的值）
#     if len(new_args) > 0 and type(new_args[0]) is tuple and len(new_args[0]) == 0:
#         new_args[0] = ""
        
#     # 检查按关键字传递的 'argument' 参数
#     if 'argument' in kwargs and type(kwargs['argument']) is tuple and len(kwargs['argument']) == 0:
#         kwargs['argument'] = ""
        
#     # 带着修改后的参数，放行给原函数
#     return original_ensure(*new_args, **kwargs)

# # 3. 强行替换系统内存中的核心函数
# launch.utilities.ensure_argument_type_impl.ensure_argument_type = patched_ensure
# launch.utilities.ensure_argument_type = patched_ensure

# # =====================================================================

# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch

# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("marine_arm", package_name="marine_arm_moveit_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)