from controller.config import * 
from enum import Enum, auto

class RobotState(Enum):
    IDLE = auto()          # 대기 상태
    BUSY = auto()          # 작업 중
    ERROR = auto()         # 에러 상태
    PAUSED = auto()        # 일시정지 상태 등 필요시 추가

class RobotStateManager:
    def __init__(self):
        self.state = RobotState.IDLE

    def set_state(self, new_state: RobotState):
        print(f"RobotState changed from {self.state} to {new_state}") # 상태 전환 로그 찍기
        self.state = new_state

    def is_idle(self) -> bool:
        return self.state == RobotState.IDLE

    def is_busy(self) -> bool:
        return self.state == RobotState.BUSY

    def is_error(self) -> bool:
        return self.state == RobotState.ERROR

    def get_state_str(self) -> str:
        return self.state.name.lower()