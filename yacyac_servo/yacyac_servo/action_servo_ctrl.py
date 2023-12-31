from typing import Optional
import rclpy as rp
import time as time
from rclpy.executors import Executor

from rclpy.node import Node
from rclpy.action import ActionServer

from yacyac_interface.msg import Servo
from yacyac_interface.action import Supply as SupplyAction
from yacyac_interface.srv import TTS
#include "yacyac_interface/srv/tts.hpp"
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from std_msgs.msg import Int8

# 목적 포지션을 받으면 그에 맞게 서보를 움직임
# 목적 포지션에 도달하면 도착 메시지를 보냄



class ServoCtrl(Node): 
    def __init__(self):
        super().__init__("servo_ctrl")
        # 약 개수 입력받음 
        self._action_server = ActionServer(self, SupplyAction, "/yacyac/supply_action", self.execute_callback)
        # 해당 포지션으로 다이나믹셀의 포지션을 이동시킴
        self.pub = self.create_publisher(SetPosition, "/set_position", 20)
        # 현재 포지션을 서비스 클라이언트로 받아옴 
        self.cli = self.create_client(GetPosition, "/get_position")
        # tts client
        self.cli_io = self.create_client(TTS, "/yacyac/io")
        self.req_io = TTS.Request()
        
        while not self.cli_io.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        # 현재 알약 배출개수 정보 pub
        self.pub_yac = self.create_publisher(Int8, "/yacyac/supply/process", 10)

        self.cmd_pose_id = Servo()
        self.cnt = 0
        
        self.current_yac = 0


        # 큰약 0, 1
        # 목적 포지션 list
        # 1023 to 300 degree
        # 시계방향으로 돌아가는 포지션 리스트
        self.big_position_set =  [0, 164, 328, 492, 656, 820] 

        # 중간약 2, 3, 4
        # 목적 포지션 list
        # 1023 to 300 degree
        # 시계방향으로 돌아가는 포지션 리스트
        self.middle_position_set =  [60, 224, 388, 552, 716, 880] 
        # 반 시계방향으로 돌아가는 포지션 리스트 

        # 작은약 5, 6, 7
        # 목적 포지션 list
        # 1023 to 300 degree
        # 시계방향으로 돌아가는 포지션 리스트
        self.small_position_set =  [140, 314, 478, 642, 806, 960]

        # 방향 flag
        self.position_flag = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        # 현재 포지션 index
        self.position_cnt =  [0, 0, 0, 0, 0, 0, 0, 0, 0]



        # 현재 약 배출 개수 
        self.yac_cnt = 0

        print ("init positioning...")
        for i in range(8):
            # 근처 포지션으로 이동합니다.
            self.init_position(i)
            # 원점 포지션으로 이동합니다. 
            # self.reset_position(i)
        print ("init positioning done!!!")


    def execute_callback(self, goal_handle):  
        print('제조 정보를 입력 받았습니다...')
        # self.req_io.tts_str_t = '제조 정보를 입력 받았습니다'

        # future = self.cli_io.call_async(self.req_io)
        # rp.spin_until_future_complete(self, future)

        supply_list = list(goal_handle.request.yac_supply_list)
        print("약 제조 리스트")
        print("list : ", supply_list)
        
        yac_sum = sum(supply_list)
        for idx in range(len(supply_list)):
            if supply_list[idx] != 0:
                self.control_position(idx, supply_list[idx], goal_handle)
        self.current_yac = 0
        goal_handle.succeed()

        result = SupplyAction.Result()
        print(yac_sum)
        result.sequence = int(yac_sum)
        print('총 ', yac_sum, '개의 약을 제조했습니다.')

        # self.req_io.tts_str_t = '제조가 완료 되었습니다.'

        # future = self.cli_io.call_async(self.req_io)
        # rp.spin_until_future_complete(self, future)

      
        
        return result



    def init_position(self, id):
        req = GetPosition.Request()
        req.id = id

        future = self.cli.call_async(req)
        rp.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            # print("response")
            closest_idx = 0

            min_diff = 10000

            
            cmd_pose = SetPosition()
            cmd_pose.id = id
            if id < 2 :
                for i, val in enumerate(self.big_position_set):
                    diff = abs(response.position - val)
                    if diff < min_diff:
                        min_diff = diff
                        closest_idx = i
                        cmd_pose.position = self.big_position_set[closest_idx]
                        
                if closest_idx > 3:
                    self.position_flag[id] = 1
                else:
                    self.position_flag[id] = 0
              
                # print("ccw")
                # 리스트의 요소값과 같은 값이 있으면 그 인덱스를 반환

                for i, val in enumerate(self.big_position_set):
                    if val == cmd_pose.position:
                        self.position_cnt[id] = i 
                        break
   
            elif  1 < id < 5 :
                for i, val in enumerate(self.middle_position_set):
                    diff = abs(response.position - val)
                    if diff < min_diff:
                        min_diff = diff
                        closest_idx = i
                        cmd_pose.position = self.middle_position_set[closest_idx]
                        
                if closest_idx > 3:
                    self.position_flag[id] = 1
                else:
                    self.position_flag[id] = 0
              
                # print("ccw")
                # 리스트의 요소값과 같은 값이 있으면 그 인덱스를 반환

                for i, val in enumerate(self.middle_position_set):
                    if val == cmd_pose.position:
                        self.position_cnt[id] = i 
                        break 


            else :
                for i, val in enumerate(self.small_position_set):
                    diff = abs(response.position - val)
                    if diff < min_diff:
                        min_diff = diff
                        closest_idx = i
                        cmd_pose.position = self.small_position_set[closest_idx]
                        
                if closest_idx > 3:
                    self.position_flag[id] = 1
                else:
                    self.position_flag[id] = 0
              
                # print("ccw")
                # 리스트의 요소값과 같은 값이 있으면 그 인덱스를 반환

                for i, val in enumerate(self.small_position_set):
                    if val == cmd_pose.position:
                        self.position_cnt[id] = i 
                        break
                
            self.pub.publish(cmd_pose)
            # time.sleep(0.1)
        else:
            self.get_logger().info(f"Service call for ID {id} failed")


    def control_position(self, id, servo, goal_handle):
        # 제조할 약의 개수 
        yac_num = servo
        # 배출한 약의 개수 
        yac_cnt = 0
        while yac_num != 0:
            # 포지션이 끝에 도달하면 방향을 바꿈
            # 역방향 진행
            yac_num -= 1
            yac_cnt += 1
            self.current_yac += 1
            msg = Int8()
            msg.data = self.current_yac
            self.pub_yac.publish(msg)
            # yac_cnt 
            print(id, "번 약 ", servo, "개중 ", yac_cnt, "개 배출중...")
            feedback_msg = SupplyAction.Feedback()
            self.yac_cnt += 1
            feedback_msg.partial_sequence.append(self.yac_cnt)
            goal_handle.publish_feedback(feedback_msg)
            # self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
       
            # print(id, "번 약 ", servo, "개중 ", yac_cnt, "개 배출중...")

            if id < 2:
                print("큰약")
                if self.position_flag[id] == 1:
                    cmd_pose = SetPosition()
                    self.position_cnt[id] += 1
                    if self.position_cnt[id] == len(self.big_position_set):
                        self.position_cnt[id] = 4
                        self.position_flag[id] = 0
                        cmd_pose.position = self.big_position_set[self.position_cnt[id]]
                        cmd_pose.id = id
                        self.pub.publish(cmd_pose)
                    else:    
                        cmd_pose.position = self.big_position_set[self.position_cnt[id]]
                        cmd_pose.id = id 
                        self.pub.publish(cmd_pose)
                    # print(self.position_cnt)
                else :
                    cmd_pose = SetPosition()
                    self.position_cnt[id] -= 1
                    if self.position_cnt[id] < 0:
                        self.position_cnt[id] = 1
                        self.position_flag[id] = 1
                        cmd_pose.position = self.big_position_set[self.position_cnt[id]]
                        cmd_pose.id = id
                        self.pub.publish(cmd_pose)
                    else:    
                        cmd_pose.position = self.big_position_set[self.position_cnt[id]]
                        cmd_pose.id = id 
                        self.pub.publish(cmd_pose)
                    # print(self.position_cnt)

                        
            elif 1 < id < 5:
                print("중간약")
                if self.position_flag[id] == 1:
                    cmd_pose = SetPosition()
                    self.position_cnt[id] += 1
                    if self.position_cnt[id] == len(self.middle_position_set):
                        self.position_cnt[id] = 4
                        self.position_flag[id] = 0
                        cmd_pose.position = self.middle_position_set[self.position_cnt[id]]
                        cmd_pose.id = id
                        self.pub.publish(cmd_pose)
                    else:    
                        cmd_pose.position = self.middle_position_set[self.position_cnt[id]]
                        cmd_pose.id = id 
                        self.pub.publish(cmd_pose)
                    # print(self.position_cnt)
                else :
                    cmd_pose = SetPosition()
                    self.position_cnt[id] -= 1
                    if self.position_cnt[id] < 0:
                        self.position_cnt[id] = 1
                        self.position_flag[id] = 1
                        cmd_pose.position = self.middle_position_set[self.position_cnt[id]]
                        cmd_pose.id = id
                        self.pub.publish(cmd_pose)
                    else:    
                        cmd_pose.position = self.middle_position_set[self.position_cnt[id]]
                        cmd_pose.id = id 
                        self.pub.publish(cmd_pose)
                    # print(self.position_cnt)
            else:
                print("작은약")
                if self.position_flag[id] == 1:
                    cmd_pose = SetPosition()
                    self.position_cnt[id] += 1
                    if self.position_cnt[id] == len(self.small_position_set):
                        self.position_cnt[id] = 4
                        self.position_flag[id] = 0
                        cmd_pose.position = self.small_position_set[self.position_cnt[id]]
                        cmd_pose.id = id
                        self.pub.publish(cmd_pose)
                    else:    
                        cmd_pose.position = self.small_position_set[self.position_cnt[id]]
                        cmd_pose.id = id 
                        self.pub.publish(cmd_pose)
                    # print(self.position_cnt)
                else :
                    cmd_pose = SetPosition()
                    self.position_cnt[id] -= 1
                    if self.position_cnt[id] < 0:
                        self.position_cnt[id] = 1
                        self.position_flag[id] = 1
                        cmd_pose.position = self.small_position_set[self.position_cnt[id]]
                        cmd_pose.id = id
                        self.pub.publish(cmd_pose)
                    else:    
                        cmd_pose.position = self.small_position_set[self.position_cnt[id]]
                        cmd_pose.id = id 
                        self.pub.publish(cmd_pose)
                    # print(self.position_cnt)
            time.sleep(0.5)
        print(id, "번 약 배출이 완료되었습니다.")

        
    def reset_position(self, id):
        cmd_pose = SetPosition()
        cmd_pose.id = id
        if id < 2:
            cmd_pose.position = self.big_position_set[0]
        elif 1 < id < 5:
            cmd_pose.position = self.middle_position_set[0]
        else:
            cmd_pose.position = self.small_position_set[0]
        self.pub.publish(cmd_pose)



def main(args=None):
    rp.init(args=args)

    servo_node = ServoCtrl()
    rp.spin(servo_node)

    servo_node.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    print("start servo_node")
    main()
