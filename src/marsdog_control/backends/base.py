from typing import Protocol, Set
from marsdog_control.core.types import ControlOutput, RobotState

class RobotBackend(Protocol):
    """
    通用机器人后端抽象。
    不管是实机还是仿真，都必须暴露统一的 URDF 空间状态读取和下发接口。
    """
    
    def read_state(self, online_ids: Set[int]) -> RobotState:
        """读取传感器和电机状态，返回纯 URDF 空间的 RobotState"""
        ...
        
    def send(self, output: ControlOutput) -> None:
        """接收纯 URDF 空间的 ControlOutput 并下发到具体实现"""
        ...
        
    def shutdown(self, reason: str = "") -> None:
        """关闭后端连接与资源"""
        ...