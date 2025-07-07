import rclpy
import time
import json
import os
from ament_index_python.packages import get_package_share_directory

from controller.config import *
from dsr_msgs2.srv import MoveStop
from dsr_msgs2.srv import GetCurrentTcp
from dsr_msgs2.srv import GetCurrentTool

import DR_init
#추가
import asyncio
import threading
import pygame

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

INIT_POS = [0, 0, 90, 0, 90, 0]


class MotionController:
    def __init__(self, node):
        #추가
        pygame.init()
        pygame.mixer.init()
        self.node = node
        self.client = node.create_client(MoveStop, "/dsr01/motion/move_stop")
        self.tcp_client = node.create_client(GetCurrentTcp, "/dsr01/tcp/get_current_tcp")
        self.tool_client = node.create_client(GetCurrentTool, "/dsr01/tool/get_current_tool")
        
        path = get_package_share_directory('controller')
        json_path = os.path.join(path, 'resource', 'pose_data.json')
        
        self.pose_data = self.load_pose_data(json_path)
        
        self.video_path = get_package_share_directory('controller')

        from DSR_ROBOT2 import (
            movej, movejx, movel, movec, move_periodic, amove_periodic,
            set_tool, set_tcp, set_ref_coord,
            set_digital_output,
            DR_TOOL,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            DR_FC_MOD_REL, DR_FC_MOD_ABS,
            DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z,
            DR_BASE, DR_QSTOP,
            release_force,
            release_compliance_ctrl,
            movesx, movesj, moveb,
            wait,
            DR_MV_MOD_REL, DR_MVS_VEL_NONE,
        )
        from DR_common2 import posx, posj

        # 함수와 상수들 alias
        self.release_compliance_ctrl = release_compliance_ctrl
        self.release_force = release_force
        self.check_force_condition = check_force_condition
        self.task_compliance_ctrl = task_compliance_ctrl
        self.set_desired_force = set_desired_force
        self.set_digital_output = set_digital_output
        self.set_ref_coord = set_ref_coord
        self.wait = wait
        self.movec = movec
        self.movel = movel
        self.movesx = movesx
        self.movesj = movesj
        self.amove_periodic = amove_periodic
        self.posx = posx
        self.posj = posj
        self.movej = movej
        self.movejx = movejx
        self.moveb = moveb
        self.move_periodic = move_periodic
        self.grasp = self._grasp
        self.release = self._release

        self.DR_QSTOP = DR_QSTOP
        self.DR_BASE = DR_BASE
        self.DR_TOOL = DR_TOOL
        self.DR_AXIS_X = DR_AXIS_X
        self.DR_AXIS_Y = DR_AXIS_Y
        self.DR_AXIS_Z = DR_AXIS_Z
        self.DR_MV_MOD_REL = DR_MV_MOD_REL
        self.DR_FC_MOD_REL = DR_FC_MOD_REL

        self.tcp_call_service()
        self.tool_call_service()
        
        set_tool("C3_1")
        set_tcp("GripperSA_v1")
        set_ref_coord(0)

    def play_sound_async(self, file_path):
        async def _play():
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                await asyncio.sleep(0.1)

        threading.Thread(target=lambda: asyncio.run(_play())).start()


    def load_pose_data(self, filepath = "/convenience_store_chef/controller/controller/pose_data.json"):
        with open(filepath, 'r') as f:
            return json.load(f)
        
    def get_pose(self, task_name, pose_type, pose_key):
        try:
            raw_pose = self.pose_data[task_name][pose_type][pose_key]
        except KeyError:
            self.node.get_logger().error(f"[POSE ERROR] {task_name}/{pose_type}/{pose_key} not found.")
            raise

        if pose_type == "posx":
            return self.posx(raw_pose)
        elif pose_type == "posj":
            return self.posj(raw_pose)
        else:
            raise ValueError(f"Unsupported pose_type: {pose_type}")

    def get_pose_group(self, task_name, pose_type):
        pose_dict = self.pose_data[task_name][pose_type]
        return [self.posx(pose_dict[k]) if pose_type == "posx" else self.posj(pose_dict[k]) for k in pose_dict.keys()]

    def call_service(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for /move_stop service...')
        request = MoveStop.Request()
        request.stop_mode = self.DR_QSTOP
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        if response:
            self.node.get_logger().info('MoveStop service call success')
        else:
            self.node.get_logger().info('MoveStop service call failed')

    def tcp_call_service(self):
        while not self.tcp_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for /get_current_tcp service...')
        request = GetCurrentTcp.Request()
        future = self.tcp_client.call_async(request)
        future.add_done_callback(self.tcp_response_callback)
    
    def tcp_response_callback(self, future):
        response = future.result()
        if response.success:
            self.node.get_logger().info(f'tool: {response.info}')
            if response.info == '':
                self.node.get_logger().info(f'no tcp')
                self.node.destroy_node()
                rclpy.shutdown()
            
        else:
            self.node.get_logger().error(f'no tcp response!')

    def tool_call_service(self):
        while not self.tool_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for /get_current_tool service...')
        request = GetCurrentTool.Request()
        future = self.tool_client.call_async(request)
        future.add_done_callback(self.tool_response_callback)

    def tool_response_callback(self, future):
        response = future.result()
        if response.success:
            self.node.get_logger().info(f'tool: {response.info}')
            if response.info == '':
                self.node.get_logger().info(f'no tool')
                self.node.destroy_node()
                rclpy.shutdown()
            
        else:
            self.node.get_logger().error(f'no tool response!')

    def _grasp(self):
        self.set_digital_output(1, 1) #open
        self.set_digital_output(2, 0) #close
        time.sleep(0.5)

    def _release(self):
        self.set_digital_output(1, 0)
        self.set_digital_output(2, 1)
        time.sleep(0.5)
    
    def detecting(self):
        self.play_sound_async(os.path.join(self.video_path, 'resource', 'order_complete.wav'))
        
        self.task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.1)
        self.set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=self.DR_FC_MOD_REL)
        while True:
            if self.check_force_condition(self.DR_AXIS_X, min=10, ref=self.DR_BASE) == 0:
                self.node.get_logger().info('X detection!')
                self.node.get_logger().warn(f"{self.check_force_condition(self.DR_AXIS_X, min=10, ref=self.DR_BASE)}")
                result = 'x'
                break
            
            elif self.check_force_condition(self.DR_AXIS_Y, min=10, ref=self.DR_BASE) == 0:
                self.node.get_logger().info('y detection!')
                self.node.get_logger().warn(f"{self.check_force_condition(self.DR_AXIS_Y, min=10, ref=self.DR_BASE)}")
                result = 'y'
                break
            
            elif self.check_force_condition(self.DR_AXIS_Z, min=10, ref=self.DR_BASE) == 0:
                self.node.get_logger().info('z detection!')
                self.node.get_logger().warn(f"{self.check_force_condition(self.DR_AXIS_Z, min=10, ref=self.DR_BASE)}")
                result = 'z'
                break

        self.release_force()
        self.release_compliance_ctrl()
        
        return result

    def init(self):
        self.movej(READY_POS, vel=VELOCITY, acc=ACC)
        self.release()

    def move_to_stock(self):
        poses = self.get_pose_group("move_to_stock", "posx")

        self.movesx(poses, vel=[300, DEGREE_VELOCITY], acc=[300, DEGREE_ACC])

    def move_to_device(self):
        poses = self.get_pose_group("move_to_device", "posx")

        self.movesx(poses, vel=[300, DEGREE_VELOCITY], acc=[300, DEGREE_ACC])
    
    def pick_food_from_stock(self, food_stock_pick):
        self.movel(self.posx(food_stock_pick), vel=VELOCITY, acc=ACC)
        
        self.grasp()
        self.wait(0.5)
        
        self.task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.1)
        self.set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DR_FC_MOD_REL)
        while True:
            if self.check_force_condition(self.DR_AXIS_Z, min=8, ref=self.DR_BASE) == 0:
                break
        self.release_force()
        self.release_compliance_ctrl()

        self.movel(self.posx([0, 0, 10, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        self.release()
        self.wait(0.5)

        self.movel(self.posx([0, 0, -40, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        self.grasp()
        self.wait(0.5)

        self.movel(self.posx([0, 0, 200, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)

    # 김밥, 삼각김밥
    ####################################################################################

    def open_microwave(self):
        poses = self.get_pose_group("open_microwave", "posj")

        self.movesj(poses[:2], vel=VELOCITY, acc=ACC)
        self.grasp()
        self.wait(0.5)
        
        self.movej(poses[2], vel=VELOCITY, acc=ACC)
        self.release()
        self.wait(0.5)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        
        self.movesj(poses[3:], vel=VELOCITY, acc=ACC)
        self.init()

    def place_food_in_microwave(self):
        posj_paths = self.get_pose_group("place_food_in_microwave", "posj")

        self.movesj(posj_paths[:4], vel=VELOCITY, acc=ACC)
        
        self.release()
        self.wait(0.5)

        self.movej(posj_paths[4], vel=VELOCITY, acc=ACC)
        self.init()
        
    def close_microwave(self):
        posj_paths = self.get_pose_group("close_microwave", "posj")
        posx_paths = self.get_pose_group("close_microwave", "posx")
        
        self.movesj(posj_paths, vel=VELOCITY, acc=ACC)

        self.movesx(posx_paths, vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        self.init()

    def press_button(self):
        paths = self.get_pose_group("press_button", "posj")
        
        self.movesj(paths, vel=VELOCITY, acc=ACC)

        self.grasp()
        self.wait(0.5)

        self.set_ref_coord(self.DR_TOOL)
        self.task_compliance_ctrl(stx=[100]*6)

        time.sleep(0.1)
        self.set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DR_TOOL)
        while True:
            if self.check_force_condition(self.DR_AXIS_Z, min=8, ref=self.DR_TOOL) == 0:
                break

        self.release_force()
        self.release_compliance_ctrl()

        self.set_ref_coord(self.DR_BASE)
        self.movej(paths[0], vel=VELOCITY, acc=ACC)
        self.init()

    def pick_up_food(self):
        paths = self.get_pose_group("pick_up_food", "posj")
        
        self.movesj(paths[:4], vel=VELOCITY, acc=ACC)
        
        self.grasp()
        self.wait(0.5)

        self.movej(paths[4], vel=VELOCITY, acc=ACC)
    
    def serve_gimbab(self):
        paths = self.get_pose_group("serve_gimbab", "posj")

        self.movesj(paths, vel=VELOCITY, acc=ACC)

        self.detecting()

        self.release()
        self.wait(0.5)

        self.init()

    # 라면
    ####################################################################################
    def put_ramen(self):
        poses = self.get_pose_group("put_ramen", "posx")
        
        self.movesx(poses, vel=[300, DEGREE_VELOCITY], acc=[300, DEGREE_ACC])
        self.release()
        self.wait(0.5)

    def pick_up_soup(self):
        poses = self.get_pose_group("pick_up_soup", "posj")
        
        self.movesj(poses[:5], vel=VELOCITY, acc=ACC)
        self.grasp()
        self.wait(0.5)
        self.movej(poses[5], vel=VELOCITY, acc=ACC)

    def put_soup(self):
        posx_paths = self.get_pose_group("put_soup", "posx")
        posj_paths = self.get_pose_group("put_soup", "posj")

        self.movesx(posx_paths[:4], vel=[300, DEGREE_VELOCITY], acc=[300, DEGREE_ACC])
        
        self.movej([0, 0, 0, 0, 0, -90], vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        self.wait(0.5)
        self.movec(posx_paths[4], posx_paths[5], vel=VELOCITY, acc=ACC, angle=[1080.0, 0.0])
        self.movej([0, 0, 0, 0, 0, 90], vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        
        self.movesj(posj_paths, vel=VELOCITY, acc=ACC)
        
        self.release()
        self.wait(0.5)

        self.movesx(posx_paths[6:], vel=[300, DEGREE_VELOCITY], acc=[300, DEGREE_ACC])
        self.init()

    def press_cooker_button(self):
        paths = self.get_pose_group("press_cooker_button", "posj")
        
        self.movesj(paths, vel=VELOCITY, acc=ACC)

        self.grasp()

        self.set_ref_coord(self.DR_TOOL)
        self.task_compliance_ctrl(stx=[100]*6)
        self.wait(0.5)
        self.set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DR_TOOL)
        while True:
            if self.check_force_condition(self.DR_AXIS_Z, min=8, ref=self.DR_TOOL) == 0:
                break

        self.release_force()
        self.release_compliance_ctrl()
        self.set_ref_coord(self.DR_BASE)

        self.movej(paths[0], vel=VELOCITY, acc=ACC)
        self.init()

    def pick_up_bowl(self):
        paths = self.get_pose_group("pick_up_bowl", "posj")
        
        self.movesj(paths[:3], vel=VELOCITY, acc=ACC)
        
        self.grasp()
        self.wait(0.5)

        self.movej(paths[3], vel=25, acc=25)
    
    def serve_ramen(self):
        paths = self.get_pose_group("serve_ramen", "posj")
        
        self.movesj(paths, vel=VELOCITY, acc=ACC)
        
        self.detecting()

        self.release()
        self.wait(0.5)

        self.init()

####################################################################################

def main():
    rclpy.init()
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    test = MotionController(node)
    test.init()
    test.detecting()
    node.destroy_node()

if __name__ == "__main__":
    main()
    rclpy.shutdown()