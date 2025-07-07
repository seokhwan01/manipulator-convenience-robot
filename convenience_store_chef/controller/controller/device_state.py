from controller.config import * 
from enum import Enum, auto


class DeviceState(Enum):
    IDLE = auto()              # 대기 상태
    COOKING = auto()           # 조리 중 (타이머 도는 중)
    WAITING_TO_UNLOAD = auto() # 조리 끝남 → 빼기 작업 대기

class DeviceStateManager:
    def __init__(self):
        self.states = {
            "microwave": DeviceState.IDLE,
            "cooker": DeviceState.IDLE,
        }

    def get_state(self, device: str) -> DeviceState:
        return self.states[device]

    def set_state(self, device: str, new_state: DeviceState):
        print(f"[{device}] 상태 전환: {self.states[device].name} → {new_state.name}")
        self.states[device] = new_state

    def is_idle(self, device: str) -> bool:
        return self.states[device] == DeviceState.IDLE

    def is_cooking(self, device: str) -> bool:
        return self.states[device] == DeviceState.COOKING

    def is_waiting_unload(self, device: str) -> bool:
        return self.states[device] == DeviceState.WAITING_TO_UNLOAD