import rclpy as rp
import time as time

from rclpy.node import Node

from yacyac_interface.msg import Servo
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition

# 목적 포지션을 받으면 그에 맞게 서보를 움직임
# 목적 포지션에 도달하면 도착 메시지를 보냄



class ServoCtrl(Node):
    def __init__(self):
        super().__init__("servo_ctrl")
        self.sub = self.create_subscription(Servo, "/yacyac/servo", self.callback_yac, 10)
        self.pub = self.create_publisher(SetPosition, "/set_position", 20)
        self.cli = self.create_client(GetPosition, "/get_position")

        self.cmd_pose_id = Servo()
        self.cnt = 0
        # 목적 포지션 list
        self.position_set =  [0, 146, 292, 438, 584, 730, 876, 1022]
        # 닫는 포지션 리스트 
        self.close_position_set = [0, 292, 584, 876]
        # 방향 flag
        self.position_flag = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        # 현재 포지션 index
        self.position_cnt =  [0, 0, 0, 0, 0, 0, 0, 0, 0]
        print ("init positioning...")
        for i in range(8):
            self.init_position(i)
            # self.reset_position(i+1)
        print ("init positioning done!!!")


    def init_position(self, id):
        req = GetPosition.Request()
        req.id = id
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        future = self.cli.call_async(req)
        rp.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            
            closest_idx = 0

            min_diff = 10000
            for i, val in enumerate(self.close_position_set):
                diff = abs(response.position - val)
                if diff < min_diff:
                    min_diff = diff
                    closest_idx = i
            
            cmd_pose = SetPosition()
            cmd_pose.id = id
            cmd_pose.position = self.close_position_set[closest_idx]
            # 리스트의 요소값과 같은 값이 있으면 그 인덱스를 반환
            for i, val in enumerate(self.position_set):
                if val == cmd_pose.position:
                    self.position_cnt[id] = i
                    break
            # closest_idx이 중앙값을 넘는경우 
            if closest_idx > 2:
                # 반시계 방향으로 돌아야함
                self.position_flag[closest_idx] = 1
            else:
                # 시계 방향으로 돌아야함
                self.position_flag[closest_idx] = 0

            self.pub.publish(cmd_pose)
            time.sleep(0.1)
        else:
            self.get_logger().info(f"Service call for ID {id} failed")


    def control_position(self, id, servo):
        # 제조할 약의 개수 
        yac_num = servo * 2
        # 배출한 약의 개수 
        yac_cnt = 0
        while yac_num != 0:
            # 포지션이 끝에 도달하면 방향을 바꿈
            # 역방향 진행
            yac_num -= 1
            yac_cnt += 1
            # yac_cnt 
            if yac_cnt % 2 == 0:
                print(id, "번 약 ", servo, "개중 ", yac_cnt // 2, "개 배출중...")
            if self.position_flag[id] == 0:
                cmd_pose = SetPosition()
                self.position_cnt[id] -= 1
                if self.position_cnt[id] < 0:
                    self.position_cnt[id] = 1
                    self.position_flag[id] = 1
                
                cmd_pose.position = self.position_set[self.position_cnt[id]]
                cmd_pose.id = id 
                self.pub.publish(cmd_pose)

            else :
                cmd_pose = SetPosition()
                self.position_cnt[id] += 1
                if self.position_cnt[id] >= len(self.position_set)-1:
                    self.position_cnt[id] = len(self.position_set) - 1
                    self.position_flag[id] = 0
                
                cmd_pose.position = self.position_set[self.position_cnt[id]]
                cmd_pose.id = id 
                self.pub.publish(cmd_pose)

            time.sleep(0.5)
        print(id, "번 약 배출이 완료되었습니다.")
  
    def reset_position(self, id):
        cmd_pose = SetPosition()
        cmd_pose.id = id
        cmd_pose.position = self.position_set[0]
        self.pub.publish(cmd_pose)
        time.sleep(0.1)

    def callback_yac(self, msg):
        servo_list = list(msg.servo_list)
        print("start position control")
        print("list : ", servo_list)
        for idx in range(len(servo_list)):
            if servo_list[idx] != 0:
                self.control_position(idx, servo_list[idx])
        



def main(args=None):
    rp.init(args=args)

    servo_node = ServoCtrl()
    rp.spin(servo_node)

    servo_node.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    print("start servo_node")
    main()
