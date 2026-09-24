#代码首先初始化小车模型，并且创立回调函数，设置差速驱动控制器来控制两个驱动轮的目标转速（设置目
#加载场景
#加载机器人，查询左轮右轮的各自编号并且保存
#计算角速度和线速度，将前进速度和转向速度转换为左右轮的速度
#清理回调


import carb
import numpy as np

from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.controllers import DifferentialController

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._jetbot = None
        self._callback_name = "jetbot_send_actions"

        self._drive_controller=DifferentialController(
            name="circle_controller",
            wheel_radius=0.03,
            wheel_base=0.1125,
        )

        self._linear_speed=0.15
        self._circle_radius=0.5
        self._turn_direction=1.0 #其中左转为-1，右转为
    

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets root")
            raise RuntimeError("无法找到 Isaac Sim 资源根路径")

        # 4.5版本的机器人资源
        asset_path = (
            assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        )

        add_reference_to_stage(
            usd_path=asset_path,
            prim_path="/World/Fancy_Robot",
        )

        world.scene.add(
            Robot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                position=np.array([0.0, 0.0, 0.035]),
            )
        )

    async def setup_post_load(self):
        world = self.get_world()
        self._jetbot = world.scene.get_object("fancy_robot")

        # 按名称查找关节，不假定左右轮的索引顺序
        self._wheel_indices = np.array(
            [
                self._jetbot.get_dof_index("left_wheel_joint"),
                self._jetbot.get_dof_index("right_wheel_joint"),
            ],
            dtype=np.int32,
        )

        print("DOF names:", self._jetbot.dof_names)
        print("Joint positions:", self._jetbot.get_joint_positions())

        # 避免重复注册同名回调
        if world.physics_callback_exists(self._callback_name):
            world.remove_physics_callback(self._callback_name)

        world.add_physics_callback(
            self._callback_name,
            callback_fn=self.send_robot_actions,
        )

    def send_robot_actions(self, step_size):
        angular_speed=(
            self._turn_direction*self._linear_speed/self._circle_radius
        )

        #将机器人前进的速度转化为左轮和右轮的转速
        action=self._drive_controller.forward(
            command=np.array([
                self._linear_speed,
                angular_speed,
            ])
        )

        action.joint_indices=self._wheel_indices

        self._jetbot.apply_action(action)

        self._jetbot.apply_action(
            ArticulationAction(
                joint_velocities=velocities,
                joint_indices=self._wheel_indices,
            )
        )

    def world_cleanup(self):
        # 4.5 BaseSample 的清理钩子
        world = self.get_world()
        if world is not None:
            if world.physics_callback_exists(self._callback_name):
                world.remove_physics_callback(self._callback_name)

        self._jetbot = None