import os

import rclpy
from convenience_store_chef_interfaces.action import OrderService
from rclpy.action import ActionServer

from controller.config import * 
from controller.robot_state import RobotState, RobotStateManager
from controller.device_state import DeviceState, DeviceStateManager
from controller.inventory import Inventory
from controller.motion import MotionController 

from enum import Enum
from collections import deque
import time

import asyncio
import threading
import pygame

from ament_index_python.packages import get_package_share_directory

import DR_init

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from dataclasses import dataclass, field
import uuid

class TaskType(Enum):
    PUT = "put"
    TAKE = "take"

@dataclass
class Task:
    name: str                                                   # 김밥, 삼각김밥, 라면
    device: str                                                 # microwave, cooker
    task_type: TaskType                                         # TaskType.PUT, TaskType.TAKE
    wait_time: float = 0                                        # 조리 시간 (초)
    position: list = field(default_factory=list)                # 재고 위치 저장
    id: str = field(default_factory=lambda: str(uuid.uuid4()))  # 품목 별 고유 ID


####################################################################################

def convert_to_unload(task: Task) -> Task:
    return Task(
        name=task.name,
        device=task.device,
        task_type=TaskType.TAKE,
        position=task.position,
    )

def register_unload_timer(timer_dict, task: Task):
    if task.task_type != TaskType.PUT:
        raise ValueError("⚠ 꺼내는 작업에는 타이머 등록 불가능")
    
    end_time = time.time() + task.wait_time
    new_task = convert_to_unload(task)
    timer_dict[new_task.id] = (end_time, new_task)

####################################################################################
path = get_package_share_directory('controller')

class TaskManager:
    def __init__(self, node):
        self.node = node
        self.rc = MotionController(node)

        self.state_manager = RobotStateManager()
        self.device_state_manager = DeviceStateManager()
        self.stock_manager = Inventory()

        self.task_queue = deque()              # 처리할 작업 큐
        self.timer_dict = {}                   # task.id -> (end_time, task)

        self._action_server = ActionServer(
            node,
            OrderService,
            '/order_service',
            execute_callback=self.execute_order,
        )

        pygame.init()
        pygame.mixer.init()
        
        self.video_path = get_package_share_directory('controller')
    
    def execute_order(self, goal_handle):
        request = goal_handle.request
        feedback_msg = OrderService.Feedback()

        self.log_msg(f'[>] [Action Server] 주문: {request.items} {request.quantities}개')
        
        order_success = True
        fail_items = set()
        
        for item, qty in zip(request.items, request.quantities):
            for _ in range(qty):
                try:
                    pos = self.stock_manager.get_next(item)
                except ValueError as e:
                    self.log_msg(str(e)) # , level="warn"
                    order_success = False
                    fail_items.add(item)
                    continue

                task = Task(name=item, device='microwave' if item in ['김밥', '삼각김밥'] else 'cooker',
                            task_type=TaskType.PUT,
                            wait_time=GIMBAB_TIME if item in ['김밥', '삼각김밥'] else RAMEN_TIME,
                            position=pos)
                self.task_queue.append(task)
                feedback_msg.status = f"[✓] {item} 작업 추가됨"
                goal_handle.publish_feedback(feedback_msg)
                self.play_sound_async(os.path.join(self.video_path, 'resource', 'order_receive.wav'))

        for item in ['김밥', '삼각김밥', '라면']:
            remaining = self.stock_manager.get_remaining(item)
            self.log_msg(f"[>] {item}: {remaining}개 남음")

        result = OrderService.Result()
        if not order_success:
            failed_list = ', '.join(fail_items)
            result.success = False
            result.message = f"⚠ [재고 부족] 다음 항목 누락됨: {failed_list}"
        else:
            result.success = True
            result.message = "[✓] 주문 처리 성공"

        goal_handle.succeed()
        return result

    def step(self):
        if self.state_manager.is_error():
            self.log_msg("⚠ 로봇 에러 상태 ➡ 작업 중단") # , level="warn"
            return

        now = time.time()

        # 조리 끝 ➡ 상태 전환 및 '빼기' 작업 추가
        for task_id, (end_time, task) in list(self.timer_dict.items()):
            if now >= end_time:
                self.task_queue.append(task)
                self.device_state_manager.set_state(task.device, DeviceState.WAITING_TO_UNLOAD)
                del self.timer_dict[task_id]
                self.log_msg(f"[>]  {task.device} 조리 완료")

        # 작업 큐 순회하며 가능한 작업 수행 (➡ 유동적 작업 순서 결정 위함)
        for i, task in enumerate(self.task_queue):
            if task.task_type == TaskType.PUT:
                if not self.device_state_manager.is_idle(task.device):
                    continue
            elif task.task_type == TaskType.TAKE:
                if not self.device_state_manager.is_waiting_unload(task.device):
                    continue
            else:
                continue

            self.task_queue.remove(task)
            self.execute_task(task)
            break

    def execute_task(self, task: Task):        
        if self.state_manager.is_busy():
            self.log_msg("[>] 로봇이 다른 작업 수행 중 ➡ 작업 대기")
            return
        
        try:
            self.rc.init()
            self.state_manager.set_state(RobotState.BUSY)
            self.device_state_manager.set_state(task.device, DeviceState.COOKING)
            self.log_msg(f"{task.name}({task.device}) 시작 ➡ 로봇 상태: {self.state_manager.get_state_str()}")  # self.state_manager.state.name.lower()

            if task.task_type == TaskType.PUT:
                self.execute_put_task(task)
                self.play_sound_async(os.path.join(self.video_path, 'resource', 'order_excute.wav'))
            elif task.task_type == TaskType.TAKE:
                self.execute_take_task(task)
            else:
                raise ValueError(f"⚠ 알 수 없는 작업 유형: {task.task_type}")

            self.rc.init()

        except Exception as e:
            self.log_msg(f"⚠ 작업 실패: {e}")
            self.state_manager.set_state(RobotState.ERROR)

        finally:
            self.state_manager.set_state(RobotState.IDLE)
            self.device_state_manager.set_state(task.device, DeviceState.IDLE)
            self.log_msg("[>] 작업 완료 ➡ 로봇 대기")
            
    # "넣기" 작업
    def execute_put_task(self, task: Task):

        # 김밥, 삼각김밥
        if task.device == 'microwave':
            self.rc.open_microwave()
            self.rc.move_to_stock()
            self.rc.pick_food_from_stock(task.position)
            self.rc.move_to_device()
            self.rc.place_food_in_microwave()
            self.rc.close_microwave()
            self.rc.press_button()  
            
        # 라면
        elif task.device == 'cooker':
            self.rc.move_to_stock()
            self.rc.pick_food_from_stock(task.position)
            self.rc.move_to_device()
            self.rc.put_ramen()
            self.rc.pick_up_soup()
            self.rc.put_soup()
            self.rc.press_cooker_button()

        # 조리 시간(타이머) 등록
        register_unload_timer(self.timer_dict, task)

    # "빼기" 작업
    def execute_take_task(self, task: Task):

        # 김밥, 삼각김밥
        if task.device == 'microwave':
            self.rc.open_microwave()
            self.rc.pick_up_food()
            self.rc.serve_gimbab()
            self.rc.close_microwave()

        # 라면
        elif task.device == 'cooker':
            self.rc.pick_up_bowl()
            self.rc.serve_ramen()

    def play_sound_async(self, file_path):
        async def _play():
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                await asyncio.sleep(0.1)

        threading.Thread(target=lambda: asyncio.run(_play())).start()

    def log_msg(self, msg, level='info'):
        getattr(self.node.get_logger(), level)(msg)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('task_server', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    tm = TaskManager(node)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            tm.step()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
